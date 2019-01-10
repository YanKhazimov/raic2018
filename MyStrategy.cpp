#include "MyStrategy.h"
#include <vector>
#include <iostream>
#include <fstream>
#include <math.h>
#include <string>
#include <numeric>
#include <tuple>

using namespace model;

MyStrategy::MyStrategy()
    : me(nullptr), rules(nullptr), game(nullptr), action(nullptr), m_role(Role::Unassigned)
{
}

const int TICKS = 60;
double MyStrategy::brakeDistance(double initialSpeed)
{
    double result = 0.0;
    double v = initialSpeed;
    while (v > 0.0) {
        result += v / TICKS;
        v -= rules->ROBOT_ACCELERATION / TICKS;
    }
    return result;
}

double sign(double x)
{
    return static_cast<double>((0.0 < x) - (x < 0.0));
}

bool eq(double a, double b)
{
    return fabs(a - b) <= std::numeric_limits<double>::epsilon();
}

double distanceXZ(p3d a, p3d b)
{
    return sqrt((a.x - b.x)*(a.x - b.x) + (a.z - b.z)*(a.z - b.z));
}

double length(p3d a)
{
    return sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
}

p3d normalize(p3d v)
{
    double l = length(v);
    return p3d(v.x/l, v.y/l, v.z/l);
}

double dot(const p3d& a, const p3d& b)
{
    std::vector<double> av = {a.x, a.y, a.z};
    std::vector<double> bv = {b.x, b.y, b.z};
    return std::inner_product(av.begin(), av.end(), bv.begin(), 0.0);
}

MyStrategy::futurePoint MyStrategy::hitPoint(const MyStrategy::futurePoint& center)
{
    //double xzRatio = fabs(center.v.z) / std::max(0.0001, fabs(center.v.x));

    p3d hit(center.pos.x - sign(center.v.x) * rules->BALL_RADIUS/2, // clear along Vx
            center.pos.y - rules->BALL_RADIUS/2,
            center.pos.z - rules->BALL_RADIUS/2);
    return futurePoint(hit, center.v, center.t);
}

p3d MyStrategy::getGoalieDefaultPosition(const Ball& ball)
{
    p3d ballPosition(ball.x, ball.y, ball.z);
    const Arena& arena = rules->arena;

    // by bissectrisa
    double d1 = distanceXZ(ballPosition, {-arena.goal_width/2, 0, -arena.depth/2});
    double d2 = distanceXZ(ballPosition, {arena.goal_width/2, 0, -arena.depth/2});

    double goalieX = -arena.goal_width/2 + arena.goal_width * d1 / (d1 + d2);

    double goalieZ = -arena.depth/2;
    int ballTimeToGoalLine = static_cast<int>((ball.z - (-rules->arena.depth/2)) / (ball.velocity_z / TICKS));
    if (ballTimeToGoalLine < 0)
        ballTimeToGoalLine = 100;

    int myTimeToGoalLine = interceptionTime(futurePoint(p3d(goalieX, 0.0, -rules->arena.depth/2), p3d(), 0), me);
    if (myTimeToGoalLine < ballTimeToGoalLine)
        goalieZ -= arena.goal_depth - arena.bottom_radius;

    p3d result {goalieX, 0.0, goalieZ};

    return result;
}

void MyStrategy::setSpeed(double value, p3d normal)
{
    p3d speedVec = normalize(normal) * value;
    action->target_velocity_x = speedVec.x;
    action->target_velocity_y = speedVec.y;
    action->target_velocity_z = speedVec.z;
}

void MyStrategy::runTo(p3d to)
{
    if (distanceXZ(to, p3d(me->x, me->y, me->z)) > brakeDistance(rules->ROBOT_MAX_GROUND_SPEED))
    {
        setSpeed(rules->ROBOT_MAX_GROUND_SPEED, p3d(to.x - me->x, 0.0, to.z - me->z));
    }
    else
    {
        action->target_velocity_x = 0.0;
        action->target_velocity_z = 0.0;
    }
}

void MyStrategy::sprintTo(p3d to, bool jump)
{
    setSpeed(rules->ROBOT_MAX_GROUND_SPEED, p3d(to.x - me->x, 0.0, to.z - me->z));
    if (jump && distanceXZ(to, p3d(me->x, me->y, me->z)) < rules->BALL_RADIUS + me->radius*1.5)
        action->jump_speed = rules->ROBOT_MAX_JUMP_SPEED;
}

void MyStrategy::C_bullyGoalie()
{
    p3d targetPos { 0, 0, -rules->arena.depth };

    for (Robot r: game->robots)
    {
        if (r.player_id == me->player_id)
            continue;

        if (r.z > targetPos.z)
        {
            targetPos = { r.x, r.y, r.z };
        }
    }

    sprintTo(targetPos, false);
}

void MyStrategy::C_bullyAttacker()
{
    p3d targetPos { 0, 0, rules->arena.depth };

    for (Robot r: game->robots)
    {
        if (r.player_id == me->player_id)
            continue;

        if (r.z < targetPos.z)
        {
            targetPos = { r.x, r.y, r.z };
        }
    }

    sprintTo(targetPos, false);
}

void MyStrategy::getRole()
{
    if (me->id == m_clearerId)
    {
        m_role = Role::Goalie;
        return;
    }

    auto teammate = getTeammate();
    if (teammate.first == m_clearerId)
    {
        m_role = Role::Attacker;
        return;
    }

    const Robot* mate = &game->robots[teammate.second];

    if (mate->z < me->z ||
            (eq(mate->z, me->z) && mate->velocity_z < me->velocity_z) ||
            (eq(mate->z, me->z) && eq(mate->velocity_z, me->velocity_z) && mate->id < me->id))
    {
        m_role = Role::Attacker;
    }
    else
    {
        m_role = Role::Goalie;
    }
}

std::pair<int, int> MyStrategy::getTeammate()
{
    for (int i = 0; i < game->robots.size(); ++i)
    {
        if (game->robots[i].player_id != me->player_id || game->robots[i].id == me->id)
            continue;

        return { game->robots[i].id, i };
    }
}

void MyStrategy::getBehindObject(p3d pos, double r)
{
    p3d destination(pos.x, pos.y, pos.z - r);

    if (me->x < pos.x)
        destination.x -= r + me->radius;
    else
        destination.x += r + me->radius;

    addSphere(sphere(destination, me->radius, p3d(0.0, 1.0, 0.0)));

    sprintTo(destination, false);
}

bool isRolling(const model::Ball& ball)
{
    return fabs(ball.velocity_y) < 7.0 && ball.y < 2 * ball.radius;
}

//bool isCentering(const Rules& rules, const Game& game)
//{
//    if (!isRolling(game->ball))
//        return false;

//    if (game->ball.velocity_z < 0.2)//rules->ROBOT_MAX_GROUND_SPEED / 2)
//        return false;

//    p3d ball(game->ball.x,game->ball.y, game->ball.z);

//    bool z = true, x = true;
//    while ((x = fabs(ball.x) < rules->arena.width/2 - rules->arena.bottom_radius) &&
//           (z = ball.z < rules->arena.depth/2 - rules->arena.bottom_radius))
//    {
//        ball.x += game->ball.velocity_x / TICKS;
//        ball.z += game->ball.velocity_z / TICKS;
//    }

//    return ball.z > rules->arena.depth/2 - rules->arena.corner_radius &&
//            fabs(ball.x) > rules->arena.goal_width/2 &&
//            (!x || (!z && sign(ball.x) == sign(game->ball.x) && fabs(game->ball.x) > rules->arena.goal_width/2));
//}

std::pair<p3d, double> dan_to_plane(p3d point, p3d point_on_plane, p3d plane_normal)
{
    return {
        plane_normal,
                dot(point - point_on_plane, plane_normal)
    };
}

std::pair<p3d, double> dan_to_sphere_inner(p3d point, p3d sphere_center, double sphere_radius)
{
    return {
        normalize(sphere_center - point),
            sphere_radius - length(point - sphere_center)
    };
}

std::pair<p3d, double> dan_to_sphere_outer(p3d point, p3d sphere_center, double sphere_radius)
{
    return {
        normalize(point - sphere_center),
                length(point - sphere_center) - sphere_radius
    };
}

std::pair<p3d, double> minDan(std::pair<p3d, double> a, std::pair<p3d, double> b)
{
    return a.second < b.second ? a : b;
}

std::pair<p3d, double> dan_to_arena_quarter(p3d point, const Arena& arena)
{
    // Ground
    std::pair<p3d, double> dan = dan_to_plane(point, p3d(0.0, 0.0, 0.0), p3d(0.0, 1.0, 0.0));
    // Ceiling
    dan = minDan(dan, dan_to_plane(point, p3d(0.0, arena.height, 0.0), p3d(0.0, -1.0, 0.0)));
    // Side x
    dan = minDan(dan, dan_to_plane(point, p3d(arena.width/2, 0.0, 0.0), p3d(-1.0, 0.0, 0.0)));
    // Side z (goal)
    dan = minDan(dan, dan_to_plane(point,
                                   p3d(0.0, 0.0, arena.depth/2 + arena.goal_depth),
                                   p3d(0.0, 0.0, -1.0)));
    // Side z
    p3d v = p3d(point.x, point.y, 0.0) - p3d(arena.goal_width/2 - arena.goal_top_radius,
                                             arena.goal_height - arena.goal_top_radius,
                                             0.0);
    if (point.x >= arena.goal_width/2 + arena.goal_side_radius ||
            point.y >= arena.goal_height + arena.goal_side_radius ||
            (v.x > 0.0 && v.y > 0.0 && length(v) >= arena.goal_top_radius + arena.goal_side_radius))
        dan = minDan(dan, dan_to_plane(point, p3d(0.0, 0.0, arena.depth/2), p3d(0.0, 0.0, -1.0)));

    // Side x & ceiling (goal)
    if (point.z >= (arena.depth / 2) + arena.goal_side_radius)
    {
        // x
        dan = minDan(dan, dan_to_plane(point, p3d(arena.goal_width/2, 0.0, 0.0), p3d(-1.0, 0.0, 0.0)));
        // y
        dan = minDan(dan, dan_to_plane(point, p3d(0.0, arena.goal_height, 0.0), p3d(0.0, -1.0, 0.0)));
    }

    // Goal back corners
    // ...

    // Corner
    if (point.x > arena.width/2 - arena.corner_radius &&
            point.z > arena.depth/2 - arena.corner_radius)
        dan = minDan(dan, dan_to_sphere_inner(point,
                                              p3d(arena.width/2 - arena.corner_radius, point.y, arena.depth/2 - arena.corner_radius),
                                              arena.corner_radius));

    // Goal outer corner
    if (point.z < arena.depth/2 + arena.goal_side_radius)
    {
        // Side x
        if (point.x < arena.goal_width/2 + arena.goal_side_radius)
            dan = minDan(dan, dan_to_sphere_outer(point,
                                                  p3d(arena.goal_width/2 + arena.goal_side_radius, point.y, arena.depth/2 + arena.goal_side_radius),
                                                  arena.goal_side_radius));
        // Ceiling
        if (point.y < arena.goal_height + arena.goal_side_radius)
            dan = minDan(dan, dan_to_sphere_outer(point,
                                                  p3d(point.x, arena.goal_height + arena.goal_side_radius, arena.depth/2 + arena.goal_side_radius),
                                                  arena.goal_side_radius));
        // Top corner
        p3d o(arena.goal_width/ - arena.goal_top_radius, arena.goal_height - arena.goal_top_radius, 0.0);
        p3d v = p3d(point.x, point.y, 0.0) - o;
        if (v.x > 0.0 && v.y > 0.0)
        {
            o = o + normalize(v) * (arena.goal_top_radius + arena.goal_side_radius);
            dan = minDan(dan, dan_to_sphere_outer(point,
                                                  p3d(o.x, o.y, arena.depth/2 + arena.goal_side_radius),
                                                  arena.goal_side_radius));
        }
    }

    // Goal inside top corners
    // ...

    // Bottom corners
    if (point.y < arena.bottom_radius)
    {
        // Side x
        if (point.x > arena.width/2 - arena.bottom_radius)
            dan = minDan(dan, dan_to_sphere_inner(point,
                                                  p3d(arena.width/2 - arena.bottom_radius, arena.bottom_radius, point.z),
                                                  arena.bottom_radius));
        // Side z
        if (point.z > arena.depth/2 - arena.bottom_radius &&
                point.x >= arena.goal_width/2 + arena.goal_side_radius)
            dan = minDan(dan, dan_to_sphere_inner(point,
                                                  p3d(point.x, arena.bottom_radius, arena.depth/2 - arena.bottom_radius),
                                                  arena.bottom_radius));
        // Side z (goal)
        if (point.z > arena.depth/2 + arena.goal_depth - arena.bottom_radius)
            dan = minDan(dan, dan_to_sphere_inner(point,
                                                  p3d(point.x, arena.bottom_radius, arena.depth/2 + arena.goal_depth - arena.bottom_radius),
                                                  arena.bottom_radius));
        // Goal outer corner
        p3d o(arena.goal_width/2 + arena.goal_side_radius, arena.depth/2 + arena.goal_side_radius, 0.0);
        p3d v = p3d(point.x, point.z, 0.0) - o;
        if (v.x < 0.0 && v.y < 0.0 && length(v) < arena.goal_side_radius + arena.bottom_radius)
        {
            o = o + normalize(v) * (arena.goal_side_radius + arena.bottom_radius);
            dan = minDan(dan, dan_to_sphere_inner(point,
                                                  p3d(o.x, arena.bottom_radius, o.y),
                                                  arena.bottom_radius));
        }
        // Side x (goal)
        if (point.z >= arena.depth/2 + arena.goal_side_radius &&
                point.x > arena.goal_width/2 - arena.bottom_radius)
            dan = minDan(dan, dan_to_sphere_inner(point,
                                                  p3d(arena.goal_width/2 - arena.bottom_radius, arena.bottom_radius, point.z),
                                                  arena.bottom_radius));
        // Corner
        if (point.x > arena.width/2 - arena.corner_radius &&
                point.z > arena.depth/2 - arena.corner_radius)
        {
            p3d corner_o(arena.width/2 - arena.corner_radius,
                         arena.depth/2 - arena.corner_radius,
                         0.0);
            p3d n = p3d(point.x, point.z, 0.0) - corner_o;
            double dist = length(n);
            if (dist > arena.corner_radius - arena.bottom_radius)
            {
                n.x = n.x / dist;
                n.y = n.y / dist;
                n.z = n.z / dist;
                p3d o2 = corner_o + n * (arena.corner_radius - arena.bottom_radius);
                dan = minDan(dan, dan_to_sphere_inner(point,
                                                      p3d(o2.x, arena.bottom_radius, o2.y),
                                                      arena.bottom_radius));
            }
        }
    }

    // Ceiling corners
    if (point.y > arena.height - arena.top_radius)
    {
        // Side x
        if (point.x > arena.width/2 - arena.top_radius)
            dan = minDan(dan, dan_to_sphere_inner(point,
                                                  p3d(arena.width/2 - arena.top_radius, arena.height - arena.top_radius, point.z),
                                                  arena.top_radius));
        // Side z
        if (point.z > arena.depth/2 - arena.top_radius)
            dan = minDan(dan, dan_to_sphere_inner(point,
                                                p3d(point.x, arena.height - arena.top_radius, arena.depth/2 - arena.top_radius),
                                                arena.top_radius));
        // Corner
        if (point.x > arena.width/2 - arena.corner_radius &&
                point.z > arena.depth/2 - arena.corner_radius)
        {
            p3d corner_o(arena.width/2 - arena.corner_radius, arena.depth/2 - arena.corner_radius, 0.0);
            p3d dv = p3d(point.x, point.z, 0.0) - corner_o;
            if (length(dv) > arena.corner_radius - arena.top_radius)
            {
                p3d n = normalize(dv);
                p3d o2 = corner_o + n * (arena.corner_radius - arena.top_radius);
                dan = minDan(dan, dan_to_sphere_inner(point,
                                                      p3d(o2.x, arena.height - arena.top_radius, o2.y),
                                                      arena.top_radius));
            }
        }
    }

    return dan;
}

std::pair<p3d, double> dan_to_arena(p3d point, const Arena& arena)
{
    bool negate_x = point.x < 0;
    bool negate_z = point.z < 0;
    if (negate_x)
        point.x = -point.x;
    if (negate_z)
        point.z = -point.z;
    auto result = dan_to_arena_quarter(point, arena);
    if (negate_x)
        result.first.x = -result.first.x;
    if (negate_z)
        result.first.z = -result.first.z;
    return result;
}

void MyStrategy::simulateRoll(p3d& ballpos, p3d& ballv, const p3d& normal)
{
    double dist = dot(ballv, normalize(normal));
    p3d planeProj = ballpos + ballv - normalize(normal) * dist;
    p3d vAlong = normalize(planeProj - ballpos) * length(ballv);

    p3d xxx = normalize(normal) * (rules->GRAVITY / TICKS);
    vAlong.y -= (rules->GRAVITY / TICKS - xxx.y);

    ballv = vAlong;
    ballpos = ballpos + ballv * (1.0 / TICKS);
}

void MyStrategy::simulateBounce(p3d& ballPos, p3d& ballv)
{
    ballPos.x += ballv.x / TICKS;
    ballPos.y += ballv.y / TICKS - rules->GRAVITY / TICKS / TICKS / 2;
    ballPos.z += ballv.z / TICKS;

    ballv.y -= rules->GRAVITY / TICKS;

    std::pair<p3d, double> newDan = dan_to_arena(ballPos, rules->arena);
    if (newDan.second < rules->BALL_RADIUS)
    {
        double vAlongDan = dot(ballv, newDan.first);
        ballv = ballv - normalize(newDan.first) * vAlongDan * (1.0 + rules->BALL_ARENA_E);
        ballPos = ballPos + normalize(newDan.first) * (rules->BALL_RADIUS - newDan.second) * (1.0 + rules->BALL_ARENA_E);
    }
}

int MyStrategy::timeToElevate(double height)
{
    int ticksToGoUp = 0;
    double myY = me->y;
    double myVY = me->velocity_y;
    while (myY < height)
    {
        myVY = std::min(myVY + rules->ROBOT_ACCELERATION / TICKS, rules->ROBOT_MAX_JUMP_SPEED) - rules->GRAVITY / TICKS;
        myY += myVY / TICKS;
        if (myVY < 0.0)
        {
            return -1;
        }
        ++ticksToGoUp;
    }

    return ticksToGoUp;
}

bool MyStrategy::inGoalSector(p3d ballPos, int& xshift)
{
    p3d directionToBall = normalize(ballPos - p3d(me->x, me->y, me->z));
    p3d rightTopCorner = p3d(-rules->arena.goal_width/2 + game->ball.radius, rules->arena.goal_height - game->ball.radius, rules->arena.depth/2);
    p3d directionToRightTopCorner = normalize(rightTopCorner - p3d(me->x, me->y, me->z));
    p3d leftBottomCorner = p3d(rules->arena.goal_width/2 - game->ball.radius, game->ball.radius, rules->arena.depth/2);
    p3d directionToLeftBottomCorner = normalize(leftBottomCorner - p3d(me->x, me->y, me->z));

    xshift = 0;
    if (directionToRightTopCorner.x > directionToBall.x)
        xshift = -1;
    if (directionToBall.x > directionToLeftBottomCorner.x)
        xshift = 1;

    return directionToRightTopCorner.x <= directionToBall.x && directionToBall.x <= directionToLeftBottomCorner.x
            && directionToRightTopCorner.y >= directionToBall.y && directionToBall.y >= directionToLeftBottomCorner.y
            && directionToBall.z > 0.0;
}

void MyStrategy::addSphere(sphere s)
{
#ifdef debugging_spheres_yan
    m_spheres.push_back(s);
#endif
}

void MyStrategy::simulateTick(p3d& ballpos, p3d& ballv)
{
    std::pair<p3d, double> dan = dan_to_arena(ballpos, rules->arena);
    bool rolling = dan.second < game->ball.radius + 0.01 && dot(ballv, dan.first) < 0.0;
    if (rolling)
    {
        simulateRoll(ballpos, ballv, dan.first);
    }
    else
    {
        // bouncing
        simulateBounce(ballpos, ballv);
    }
}

bool MyStrategy::pickShootingPoint(int ticks, futurePoint& bestTarget, int& bestPace, int& bestElevationTime)
{
    p3d ballpos(game->ball.x,game->ball.y, game->ball.z);
    p3d ballv(game->ball.velocity_x, game->ball.velocity_y, game->ball.velocity_z);
    int checkStep = 10;
    bestPace = -100;
    bestElevationTime = -100;
    for (int t = 0; t < ticks; ++t)
    {
        simulateTick(ballpos, ballv);

        if (t % checkStep == 0)
        {
            int xshift;
            if (inGoalSector(ballpos, xshift))
            {
                checkStep = 2;
                addSphere(sphere(ballpos, rules->BALL_RADIUS, p3d(0.0, 0.0, 1.0)));
                futurePoint target(ballpos, ballv, t + 1);

                int pace, elevationTime;
                std::tie(pace, elevationTime) = measureShot(target);

                if (elevationTime != -1 &&
                        (abs(pace) < abs(bestPace) || (abs(pace) == abs(bestPace) && pace > bestPace)))
                {
                    bestPace = pace;
                    bestElevationTime = elevationTime;
                    bestTarget = target;
                }
            }
            else
            {
                checkStep = 10;
                addSphere(sphere(ballpos, rules->BALL_RADIUS, p3d(1.0, 0.0, 0.0)));
            }
        }
    }
    return abs(bestPace) < criticalPaceDiff && bestElevationTime != -1;
}

void MyStrategy::act(const Robot& _me, const Rules& _rules, const Game& _game, Action& _action)
{
    me = &_me; rules = &_rules; game = &_game; action = &_action;

    if (m_tick_spheres != game->current_tick)
    {
        m_spheres.clear();
        m_tick_spheres = game->current_tick;
        //m_text = "";
    }

    getRole();

    if (m_role == Role::Attacker)
    {
        //C_attack();
        return;
    }

    C_defend();
}

std::string MyStrategy::logSphere(double x, double y, double z, double r, p3d rgb)
{
    std::string sphere;

    sphere += "{\"Sphere\":{";
    sphere += "\"x\":" + std::to_string(x) + ",";
    sphere += "\"y\":" + std::to_string(y) + ",";
    sphere += "\"z\":" + std::to_string(z) + ",";
    sphere += "\"radius\":" + std::to_string(r) + ",";
    sphere += "\"r\":" + std::to_string(rgb.x) + ",";
    sphere += "\"g\":" + std::to_string(rgb.y) + ",";
    sphere += "\"b\":" + std::to_string(rgb.z) + ",";
    sphere += "\"a\":" + std::to_string(0.5) + "}}";

    return sphere;
}

std::string MyStrategy::logText(std::string text)
{
    return "{\"Text\":\"" + text + "\"}";
}

std::string MyStrategy::custom_rendering()
{
    m_json = "[";

    for (sphere sph: m_spheres)
        m_json += logSphere(sph.x, sph.y, sph.z, sph.r, sph.rgb) + ",";

    m_json += logText(m_text);

    m_json += "]";

    std::ofstream ofs("js.txt");
    ofs << m_json;
    ofs.close();

    return m_json;
}

std::pair<int, int> MyStrategy::pickInterceptionPoint(const std::vector<futurePoint>& interceptionPoints)
{
    int iTime = 0;
    for (int i = 0; i < static_cast<int>(interceptionPoints.size()); ++i)
    {
        futurePoint iPoint = interceptionPoints[i];
        iTime = interceptionTime(iPoint, me);
        if (iTime <= iPoint.t)
            return { i, iTime };
    }

    return { static_cast<int>(interceptionPoints.size() - 1), iTime };
}

bool MyStrategy::interceptBounceAt(const futurePoint& point)
{
    int sprintTime = 0, elevationTime = 0;
    if (canReachInTime(point, sprintTime, elevationTime))
    {
        double desiredSpeed = rules->ROBOT_MAX_GROUND_SPEED * sprintTime / point.t;

        setSpeed(desiredSpeed, p3d(point.pos.x - me->x, 0.0, point.pos.z - me->z));

        if (elevationTime >= point.t)
            action->jump_speed = rules->ROBOT_MAX_JUMP_SPEED;

        return true;
    }

    return false;
}

std::pair<int, int> MyStrategy::measureShot(futurePoint target)
{
    int elevationTime = timeToElevate(target.pos.y);
    if (elevationTime == -1)
        return { -criticalPaceDiff, -1 };

    int t = interceptionTime(target, me);
    int pace = target.t - t;
    //m_text = std::to_string(pace);

    return { pace, elevationTime };
}

void MyStrategy::intercept(const std::vector<futurePoint>& interceptionPoints, bool homeOnly)
{
    int interceptionTime = 0, iPointIndex = -1;
    std::tie(iPointIndex, interceptionTime) = pickInterceptionPoint(interceptionPoints);

    if (iPointIndex < 0 || iPointIndex >= interceptionPoints.size())
        return;

    futurePoint iPoint = interceptionPoints[static_cast<unsigned int>(iPointIndex)];
    if (homeOnly && iPoint.pos.z > 0.0)
        return;

    if (m_role == Role::Goalie)
        m_clearerId = me->id;

    if (isRolling(game->ball))
    {
        if (iPointIndex == 1)
            iPoint = interceptionPoints[0];
        sprintTo(iPoint.pos, true);
    }
    else
    {
        // ball is bouncing
        futurePoint pointToHit = hitPoint(iPoint);
        interceptBounceAt(pointToHit);
    }
}

void MyStrategy::C_attack()
{
    auto teammate = getTeammate();
    if (m_clearerId == teammate.first)
    {
        action->target_velocity_x = game->robots[teammate.second].velocity_x;
        action->target_velocity_y = game->robots[teammate.second].velocity_y;
        action->target_velocity_z = game->robots[teammate.second].velocity_z;
        return;
    }

    if (me->z + me->radius > game->ball.z)
    {
        getBehindObject(p3d(game->ball.x, game->ball.y, game->ball.z), game->ball.radius);
        //m_text = "getting behind";
    }
    else
    {
        //if (isCentering(rules, game))
        futurePoint shootAt;
        int shootingPace, elevationTime;
        if (pickShootingPoint(200, shootAt, shootingPace, elevationTime))
        {
            addSphere(sphere(shootAt.pos, rules->BALL_RADIUS*1.1, p3d(0.0, 1.0, 0.0)));
            if (abs(shootingPace) < criticalPaceDiff)
            {
                addSphere(sphere(shootAt.pos, rules->BALL_RADIUS, p3d(0.0, 1.0, 0.0)));

                if (shootingPace <= 0)
                    setSpeed(rules->ROBOT_MAX_GROUND_SPEED, p3d(shootAt.pos.x - me->x, 0.0, shootAt.pos.z - me->z));
                else
                    setSpeed(length(p3d(me->velocity_x, 0.0, me->velocity_z)) * (shootAt.t - shootingPace) / shootAt.t, p3d(shootAt.pos.x - me->x, 0.0, shootAt.pos.z - me->z));

                if (elevationTime >= shootAt.t)
                {
                    action->jump_speed = rules->ROBOT_MAX_JUMP_SPEED;
                }
            }
            else
            {
                m_text = "DEBUG ME!";
            }
        }
        else
        {
            std::vector<futurePoint> points;
            getInterceptionPoints(game->ball, 2.0, points);
            intercept(points, false);
        }
    }
}

void MyStrategy::C_defend()
{
    if (m_clearerId == me->id)
        m_clearerId = -1;

    p3d pos = getGoalieDefaultPosition(game->ball);
    runTo(pos);

    std::vector<futurePoint> interceptionPoints;
    bool onTarget = ballGoesToGoal(game->ball, interceptionPoints);
    if (onTarget)
    {
        intercept(interceptionPoints, true);
    }
    else
    {
        // if I'm fastest to the rolling ball and it's close - charge it
        if (!isRolling(game->ball))
            return;

        p3d ballPos(game->ball.x, game->ball.y, game->ball.z);
        double myDistance = distanceXZ(ballPos, p3d(me->x, me->y, me->z));

        if (game->ball.z < -rules->arena.depth / 2 + rules->arena.depth / 8 && fabs(game->ball.x) > rules->arena.goal_width / 2 * 1.2)
            return;

        for (const Robot& r: game->robots)
        {
            if (/*r.id == me->id && */r.player_id == me->player_id)
                continue;

            double rDist = distanceXZ(ballPos, p3d(r.x, r.y, r.z));
            if (myDistance > rDist)
                return;
        }

        m_clearerId = me->id;
        sprintTo(ballPos, true);
    }
    m_text = std::to_string(m_clearerId);
}

void MyStrategy::getInterceptionPoints(Ball ball, double secondsForward, std::vector<futurePoint>& points)
{
    double GRAVITY = rules->GRAVITY;

    p3d ballpos(ball.x, ball.y, ball.z);
    p3d ballv(ball.velocity_x, ball.velocity_y, ball.velocity_z);

    int ticks = static_cast<int>(secondsForward * TICKS);

    bool newInterval = true; // участок монотонности
    for (int t = 1; t < ticks; ++t)
    {
        if (ballv.y * (ballv.y - GRAVITY / TICKS) < 0.0)
        {
            // going down
            newInterval = true;
        }

        if (newInterval &&
                ((ballv.y < 0.0 && ballpos.y < 2 * ball.radius) || (ballv.y > 0.0 && ballpos.y < 2 * ball.radius)))
        {
            points.push_back(futurePoint(ballpos, ballv, t));
            newInterval = false;
        }

        if (ballpos.y + ballv.y / TICKS - GRAVITY / TICKS / TICKS / 2 < ball.radius)
        {
            // going up
            newInterval = true;
        }

        simulateTick(ballpos, ballv);
    }

    for (int i = 0; i < points.size(); ++i)
    {
        addSphere(sphere(points[i].pos, ball.radius, p3d(0.0, 0.0, 1.0)));
    }

    points.push_back(futurePoint(ballpos, ballv, ticks));
    addSphere(sphere(points.back().pos, ball.radius, p3d(1.0, 0.0, 1.0)));
}

bool MyStrategy::ballGoesToGoal(const Ball &ball, std::vector<futurePoint>& interceptionPoints)
{
    interceptionPoints.clear();
    bool result = false;

    double secondsToGoalLine = (ball.z + rules->arena.depth/2) / -ball.velocity_z;
    if (secondsToGoalLine > 10.0)
        return result;

    if (ball.velocity_z >= 0.0) // от ворот
        return result;

//    if (ball.velocity_x < 0 && ball.x < -rules->arena.goal_width/2) // прокатился налево
//        return result;

//    if (ball.velocity_x > 0 && ball.x > rules->arena.goal_width/2) // прокатился направо
//        return result;

//    if (fabs(ball.x) >= rules->arena.width/2 - rules->arena.bottom_radius) // над радиусом - посчитаем позже
//        return result;

    getInterceptionPoints(ball, secondsToGoalLine, interceptionPoints);

    result = fabs(interceptionPoints.back().pos.x) <= rules->arena.goal_width/2 - ball.radius/2 &&
            fabs(interceptionPoints.back().pos.y) <= rules->arena.goal_height - ball.radius;

    return result;
}

int MyStrategy::interceptionTime(futurePoint at, const Robot* robot)
{
    double myDistance = distanceXZ(p3d(robot->x, robot->y, robot->z), at.pos);
    p3d normal(at.pos.x - robot->x, 0.0, at.pos.z - robot->z);
    double normalLength = length(normal);
    double vAlong = dot(normal, p3d(robot->velocity_x, 0, robot->velocity_z)) / normalLength;
    double distanceCovered = 0.0;
    int takesTicks = 0;
    while (distanceCovered < myDistance)
    {
        vAlong += rules->ROBOT_ACCELERATION * (1.0 / TICKS);
        vAlong = std::min(vAlong, rules->ROBOT_MAX_GROUND_SPEED);
        distanceCovered += vAlong * (1.0 / TICKS);
        ++takesTicks;
    }

    return takesTicks;
}

bool MyStrategy::canReachInTime(futurePoint at, int& sprintTime, int& elevationTime)
{
    sprintTime = interceptionTime(at, me);
    if (sprintTime > at.t)
        return false; // too far

    elevationTime = timeToElevate(at.pos.y);
    if (elevationTime == -1)
        return false; // too high

    if (elevationTime > at.t)
        return false; // jumping too slow

    return true;
}

p3d::p3d(double _x, double _y, double _z) : x(_x), y(_y), z(_z) {}
p3d::p3d() : x(0.0), y(0.0), z(0.0) {}
p3d p3d::operator-(const p3d &other) const
{
    return p3d(x - other.x, y - other.y, z - other.z);
}
p3d p3d::operator+(const p3d &other) const
{
    return p3d(x + other.x, y + other.y, z + other.z);
}
p3d p3d::operator*(const double &mult) const
{
    return p3d(mult * x, mult * y, mult * z);
}
sphere::sphere(p3d pos, double _r, p3d _rgb) : x(pos.x), y(pos.y), z(pos.z), r(_r ), rgb(_rgb){}
sphere::sphere(double _x, double _y, double _z, double _r, p3d _rgb) : x(_x), y(_y), z(_z), r(_r ), rgb(_rgb){}
sphere::sphere() : x(0.0), y(0.0), z(0.0), r(0.0), rgb() {}

MyStrategy::futurePoint::futurePoint() {}
MyStrategy::futurePoint::futurePoint(p3d _pos, p3d _v, int _t) : pos(_pos), v(_v), t(_t){}
