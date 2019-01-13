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
        v -= rules->ROBOT_ACCELERATION / TICKS;
        result += v / TICKS;
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
    return l == 0.0 ? p3d() : p3d(v.x/l, v.y/l, v.z/l);
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
    double myV = length(p3d(me->velocity_x, me->velocity_y, me->velocity_z));
    if (distanceXZ(to, p3d(me->x, me->y, me->z)) > brakeDistance(myV/*rules->ROBOT_MAX_GROUND_SPEED*/))
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
//    if (me->id == m_clearerId)
//    {
//        m_role = Role::Goalie;
//        return;
//    }

    int teammateIdx = getTeammateIdx();
//    if (teammateIdx == m_clearerId)
//    {
//        m_role = Role::Attacker;
//        return;
//    }

    const Robot* mate = &game->robots[teammateIdx];

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

int MyStrategy::getTeammateIdx()
{
    for (int i = 0; i < game->robots.size(); ++i)
    {
        if (game->robots[i].player_id != me->player_id || game->robots[i].id == me->id)
            continue;

        return i;
    }
}

void MyStrategy::getBehindObject(p3d pos, double r)
{
    p3d destination(pos.x, pos.y, pos.z - r);

    if (me->x < pos.x)
        destination.x -= r + me->radius;
    else
        destination.x += r + me->radius;

    //addSphere(sphere(destination, me->radius, p3d(0.0, 1.0, 0.0)));

    sprintTo(destination, false);
}

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

double MyStrategy::maxElevation()
{
    double myY = me->y + me->radius/2;
    double myVY = rules->ROBOT_MAX_JUMP_SPEED;
    while(myVY > 0.0)
    {
        myVY -= rules->GRAVITY / TICKS;
        myY += myVY / TICKS;
    }

    return myY;
}

int MyStrategy::timeToElevate(double height)
{
    int ticksToGoUp = 0;
    double myY = me->y + me->radius/2;
    if (!me->touch)
    {
        return -1;
    }
    double myVY = rules->ROBOT_MAX_JUMP_SPEED;
    while(myY < height && myVY > 0.0)
    {
        myVY -= rules->GRAVITY / TICKS;
        myY += myVY / TICKS;
        ++ticksToGoUp;
    }

    return myY < height ? -1 : ticksToGoUp;
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
            //&& directionToRightTopCorner.y >= directionToBall.y && directionToBall.y >= directionToLeftBottomCorner.y
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
    int checkStep = 5;
    bestPace = -100;
    bestElevationTime = -100;
    for (int t = 1; t < ticks; ++t)
    {
        simulateTick(ballpos, ballv);

        if (t % checkStep == 0)
        {
            int xshift;
            if (inGoalSector(ballpos, xshift))
            {
                checkStep = 1;
                addSphere(sphere(ballpos, rules->BALL_RADIUS, p3d(1.0, 1.0, 0.0)));
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
                checkStep = 5;
                //addSphere(sphere(ballpos, rules->BALL_RADIUS, p3d(1.0, 0.0, 0.0)));
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
        m_text = "";
    }

    getRole();

    if (m_role == Role::Attacker)
    {
        C_attack();
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
    int reach = canReachInTime(point, sprintTime, elevationTime);
    if (reach == 0)
    {
        double desiredSpeed = rules->ROBOT_MAX_GROUND_SPEED * sprintTime / point.t;
        setSpeed(desiredSpeed, p3d(point.pos.x - me->x, 0.0, point.pos.z - me->z));

        if (elevationTime >= point.t)
            action->jump_speed = rules->ROBOT_MAX_JUMP_SPEED;

        return true;
    }
    else if (reach == -1)
    {
        double desiredSpeed = rules->ROBOT_MAX_GROUND_SPEED * sprintTime / point.t;
        setSpeed(desiredSpeed, p3d(point.pos.x - me->x, 0.0, point.pos.z - me->z));

        if (m_role == Role::Goalie && 30 >= point.t)
            action->jump_speed = rules->ROBOT_MAX_JUMP_SPEED;
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

    //m_clearerId = me->id;
    if (isRolling(p3d(game->ball.x, game->ball.y, game->ball.z)))
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
        //addSphere(sphere(iPoint.pos, rules->BALL_RADIUS, p3d(1.0, 0.0, 1.0)));
    }
}

p3d MyStrategy::getShotTarget()
{
    double mult = (game->players[0].score + game->players[1].score) % 2 == 0 ? 1 : -1;
    p3d goalTarget(0.0,//(-rules->arena.goal_width/2 + rules->BALL_RADIUS) * mult,
                   rules->arena.goal_height - rules->BALL_RADIUS,
                   rules->arena.depth/2 + rules->BALL_RADIUS);

    return goalTarget;
}

MyStrategy::futurePoint MyStrategy::getMoveTarget()
{
    p3d ballpos(game->ball.x,game->ball.y, game->ball.z);
    p3d ballv(game->ball.velocity_x, game->ball.velocity_y, game->ball.velocity_z);
    return futurePoint(p3d(ballpos.x, game->ball.radius, ballpos.z), ballv, 1);
}

p3d MyStrategy::getRunupPosition(const MyStrategy::futurePoint& shotPos, p3d shotTarget)
{
    p3d v = shotTarget - shotPos.pos;
    v.y += fabs(v.y);

    double runupDist = 0.0;
    timeToChangeSpeed(normalize(v) * rules->ROBOT_MAX_GROUND_SPEED, p3d(), runupDist);

    v = normalize(v) * runupDist;
    p3d runFrom = shotPos.pos - v;
    runFrom.y = me->radius;

    return runFrom;
}

int MyStrategy::timeToRun(p3d to, p3d pos, p3d v)
{
    int t = 0;
    while (t < 1000 && distanceXZ(to, pos) > brakeDistance(length(v)))
    {
        v = v + (to - pos) * (rules->ROBOT_ACCELERATION / TICKS);
        if (length(v) > rules->ROBOT_MAX_GROUND_SPEED)
        {
            v = normalize(v) * rules->ROBOT_MAX_GROUND_SPEED;
        }

        pos = pos + v * (1.0 / TICKS);

        ++t;
    }

    double dist = distanceXZ(to, pos);
    while (t < 1000 && distanceXZ(to, pos) <= dist)
    {
        dist = distanceXZ(to, pos);
        double newV = length(v) - (rules->ROBOT_ACCELERATION / TICKS);
        v = normalize(v) * newV;

        pos = pos + v * (1.0 / TICKS);

        ++t;
    }

    return t;
}

int MyStrategy::timeToChangeSpeed(p3d targetSpeed, p3d initialSpeed, double& dist)
{
    double targetLength = length(targetSpeed);
    double vAlong = dot(targetSpeed, initialSpeed) / length(targetSpeed);
    int t = 0;
    dist = 0.0;
    while (t < 1000 && vAlong < targetLength)
    {
        vAlong += rules->ROBOT_ACCELERATION / TICKS;
        dist += vAlong / TICKS;
        ++t;
    }

    return t;
}

void MyStrategy::alignShot()
{
    p3d goalTarget = getShotTarget();
    addSphere(sphere(goalTarget, rules->BALL_RADIUS, p3d(1.0, 0.0, 1.0)));

    futurePoint target;
    p3d runFrom;
    p3d ballpos(game->ball.x,game->ball.y, game->ball.z);
    p3d ballv(game->ball.velocity_x, game->ball.velocity_y, game->ball.velocity_z);
    for (int t = 1; t < 500; ++t)
    {
        simulateTick(ballpos, ballv);
        futurePoint ballSim(ballpos, ballv, t);

        p3d v = goalTarget - ballSim.pos;
        v.y = sqrt(v.x*v.x + v.z*v.z);
        double r1r2 = rules->BALL_RADIUS + rules->ROBOT_RADIUS;
        p3d hitOffset = normalize(v) * r1r2;
        futurePoint ptarget(ballSim.pos - hitOffset, ballv, t);

        runFrom = getRunupPosition(ptarget, goalTarget);
        int runupTime = hasRunUp ? 0 : timeToRun(runFrom, p3d(me->x, me->y, me->z), p3d(me->velocity_x, me->velocity_y, me->velocity_z));
        if (runupTime > t)
            continue;

        double dist = 0.0;
        int shotTime = timeToChangeSpeed(normalize(ptarget.pos - runFrom) * rules->ROBOT_MAX_GROUND_SPEED, p3d(), dist);
        if (runupTime + shotTime > t)
            continue;

        target = ptarget;
        addSphere(sphere(target.pos, rules->ROBOT_RADIUS, p3d(1.0, 0.0, 0.0)));
        addSphere(sphere(runFrom, rules->ROBOT_RADIUS, p3d(0.0, 0.0, 1.0)));
        addSphere(sphere(ballSim.pos, rules->BALL_RADIUS, p3d(0.0, 1.0, 1.0)));

        double k = (rules->MAX_HIT_E + rules->MIN_HIT_E) / 2;
        p3d robotV = normalize(ptarget.pos - runFrom) * rules->ROBOT_MAX_GROUND_SPEED;
        p3d newBallV = ballSim.v + (robotV - ballSim.v) * ((1 + k) * rules->ROBOT_MASS / (rules->ROBOT_MASS + rules->BALL_MASS));

//        for (int tAfter = 1; tAfter < 200 && ballSim.pos.z < rules->arena.depth/2 - rules->BALL_RADIUS; ++tAfter)
//        {
//            simulateTick(ballSim.pos, newBallV);

//            addSphere(sphere(ballSim.pos, rules->BALL_RADIUS, p3d(0.0, 1.0, 0.0)));
//        }

        break;

        // (ballv + myMaxV) * hitE = VtoTarget
    }

    p3d runupDirection(target.pos - runFrom);
    double distanceToStart = length(runFrom - p3d(me->x, me->y, me->z));
    double myV = length(p3d(me->velocity_x, me->velocity_y, me->velocity_z));
    double breakD = brakeDistance(myV);
    if (!hasRunUp)
    {
        if (distanceToStart > breakD + myV / TICKS)
        {
            setSpeed(rules->ROBOT_MAX_GROUND_SPEED, runFrom - p3d(me->x, me->y, me->z));
            m_text = std::to_string(distanceToStart) + " " + std::to_string(breakD) + " toStart"
                    + std::to_string(length(p3d(me->velocity_x, me->velocity_y, me->velocity_z)));
        }
        else
        {
            if (distanceToStart < 0.2)
            {
                hasRunUp = true;
            }
            setSpeed(0, /*rules->ROBOT_MAX_GROUND_SPEED,*/ runupDirection);
            m_text = std::to_string(distanceToStart) + " " + std::to_string(breakD) + " toStart, braking"
                    + std::to_string(length(p3d(me->velocity_x, me->velocity_y, me->velocity_z)));
        }
        m_text = "";
    }
    else
    {
//        double vlength = length(p3d(me->velocity_x, 0.0, me->velocity_z));
//        p3d toStart(runFrom - p3d(me->x, me->y, me->z));
//        double vAlong = dot(toStart, p3d(me->velocity_x, 0.0, me->velocity_z)) / length(toStart);

//        p3d aligningV = (normalize(p3d(me->velocity_x, 0.0, me->velocity_z)) * -vAlong) + (normalize(v) * rules->ROBOT_MAX_GROUND_SPEED);

        p3d directionToBall = normalize(target.pos - p3d(me->x, me->y, me->z));
        p3d v = goalTarget - target.pos;
        p3d nv = normalize(v/*aligningV*/);
        if (directionToBall.x < nv.x)
            directionToBall.x -= fabs(directionToBall.x - nv.x)/2;
        else if (directionToBall.x > nv.x)
            directionToBall.x += fabs(directionToBall.x - nv.x)/2;

        setSpeed(rules->ROBOT_MAX_GROUND_SPEED, directionToBall);
        bool jump = true;
        if (jump && distanceXZ(target.pos, p3d(me->x, me->y, me->z)) < rules->BALL_RADIUS + me->radius)
        {
            action->jump_speed = rules->ROBOT_MAX_JUMP_SPEED;
            //hasRunUp = false;
        }
        m_text = "to ball";
    }
    return;
}

void MyStrategy::C_attack()
{
    //if (m_clearerId == getTeammateIdx())
    {
//        int teammateIdx = getTeammateIdx();
//        const Robot* teammate = &game->robots[teammateIdx];
//        if (me->z > teammate->z)
//        {
//            getBehindObject(p3d(teammate->x, teammate->y, teammate->z), teammate->radius);
//        }
//        else
//        {
//            p3d pos = getGoalieDefaultPosition(game->ball);
//            runTo(pos);
//        }
        //m_text = "bullying attacker";
        //C_bullyAttacker();

        //return;
    }

    if (me->z + me->radius > game->ball.z)
    {
        getBehindObject(p3d(game->ball.x, game->ball.y, game->ball.z), game->ball.radius);
        //m_text = "getting behind";
    }
    else
    {
        if (game->ball.z < 0.0 || fabs(game->ball.x) > rules->arena.width/2 - rules->arena.bottom_radius)
        {
            sprintTo(p3d(game->ball.x, game->ball.y, game->ball.z), false);
            return;
        }
        futurePoint shootAt;
        int shootingPace, elevationTime;
        if (pickShootingPoint(200, shootAt, shootingPace, elevationTime))
        {
            m_text = "SHOOTING !";
            //addSphere(sphere(shootAt.pos, rules->BALL_RADIUS*1.1, p3d(0.0, 1.0, 0.0)));
            if (abs(shootingPace) < criticalPaceDiff)
            {
                //addSphere(sphere(shootAt.pos, rules->BALL_RADIUS, p3d(0.0, 1.0, 0.0)));

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
            m_text = "aligning !";
            hasRunUp = false;
            alignShot();
//            std::vector<futurePoint> points;
//            getInterceptionPoints(game->ball, 2.0, points);
//            intercept(points, false);
        }
    }
}

void MyStrategy::C_defend()
{
//    if (m_clearerId == me->id)
//        m_clearerId = -1;

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
        if (!isRolling(p3d(game->ball.z, game->ball.y, game->ball.z)))
            return;

        p3d ballPos(game->ball.x, game->ball.y, game->ball.z);
        double myDistance = distanceXZ(ballPos, p3d(me->x, me->y, me->z));

        if (game->ball.z < -rules->arena.depth / 2 + rules->arena.depth / 8 && fabs(game->ball.x) > rules->arena.goal_width / 2 * 1.2)
            return;

        for (const Robot& r: game->robots)
        {
            if (r.id == me->id && r.player_id == me->player_id)
                continue;

            if (myDistance >= distanceXZ(ballPos, p3d(r.x, r.y, r.z)))
                return;
        }

        //m_clearerId = me->id;
        sprintTo(ballPos, true);
    }
}

void MyStrategy::getInterceptionPoints(const Ball &ball, double secondsForward, std::vector<futurePoint>& points)
{
    int ticks = static_cast<int>(secondsForward * TICKS);

    double maxH = maxElevation();

    p3d ballpos(game->ball.x,game->ball.y, game->ball.z);
    p3d ballv(game->ball.velocity_x, game->ball.velocity_y, game->ball.velocity_z);
    for (int t = 1; t < ticks; ++t)
    {
        simulateTick(ballpos, ballv);
        bool rolling = isRolling(ballpos);

        if (ballpos.y < maxH && (!rolling || t % 5 == 0))
        {
            points.push_back(futurePoint(ballpos, ballv, t));
        }
    }

    simulateTick(ballpos, ballv);
    points.push_back(futurePoint(ballpos, ballv, ticks));

    for (auto ip: points)
    {
        addSphere(sphere(ip.pos, ball.radius, p3d(0.0, 0.0, 1.0)));
    }
}

bool MyStrategy::ballGoesToGoal(const Ball &ball, std::vector<futurePoint>& interceptionPoints)
{
    interceptionPoints.clear();

    if (ball.velocity_z >= 0.0) // от ворот
        return false;

    if (ball.velocity_x < 0 && ball.x < -rules->arena.goal_width/2) // прокатился налево
        return false;

    if (ball.velocity_x > 0 && ball.x > rules->arena.goal_width/2) // прокатился направо
        return false;

    if (fabs(ball.x) >= rules->arena.width/2 - rules->arena.bottom_radius) // над радиусом - посчитаем позже
        return false;

    double secondsToGoalLine = (ball.z + rules->arena.depth/2) / -ball.velocity_z;

    if (secondsToGoalLine > 30.0)
        return false;

    getInterceptionPoints(ball, secondsToGoalLine, interceptionPoints);

    bool result = !interceptionPoints.empty() &&
            fabs(interceptionPoints.back().pos.x) <= rules->arena.goal_width/2 - ball.radius/2 &&
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

int MyStrategy::canReachInTime(futurePoint at, int& sprintTime, int& elevationTime)
{
    if (at.t == 0)
        return -2;

    sprintTime = interceptionTime(at, me);
    if (sprintTime > at.t)
        return -2; // too far

    elevationTime = timeToElevate(at.pos.y);
    if (elevationTime == -1)
    {
        return -1; // too high
    }

    if (elevationTime > at.t)
        return -1; // jumping too slow

    return 0;
}

bool MyStrategy::isRolling(const p3d& ballPos)
{
    std::pair<p3d, double> newDan = dan_to_arena(ballPos, rules->arena);
    return eq(newDan.second, rules->BALL_RADIUS);
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
bool p3d::operator==(const p3d &other)
{
    return eq(x, other.x) && eq(y, other.y) && eq(z, other.z);
}
bool p3d::operator!=(const p3d &other)
{
    return !(*this == other);
}
sphere::sphere(p3d pos, double _r, p3d _rgb) : x(pos.x), y(pos.y), z(pos.z), r(_r ), rgb(_rgb){}
sphere::sphere(double _x, double _y, double _z, double _r, p3d _rgb) : x(_x), y(_y), z(_z), r(_r ), rgb(_rgb){}
sphere::sphere() : x(0.0), y(0.0), z(0.0), r(0.0), rgb() {}

MyStrategy::futurePoint::futurePoint() {}
MyStrategy::futurePoint::futurePoint(p3d _pos, p3d _v, int _t) : pos(_pos), v(_v), t(_t){}
bool MyStrategy::futurePoint::operator==(const MyStrategy::futurePoint &other)
{
    return pos == other.pos && v == other.v && t == other.t;
}
bool MyStrategy::futurePoint::operator!=(const MyStrategy::futurePoint &other)
{
    return !(*this == other);
}
