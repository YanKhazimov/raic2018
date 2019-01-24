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
    : me(nullptr), rules(nullptr), game(nullptr), action(nullptr), m_role(Role::Unassigned),
      m_plannedAttackerTarget(), m_plannedGoalieTarget()
{
}

const int TICKS = 60;

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

double MyStrategy::brakeDistance(double initialSpeed)
{
    double result = 0.0;
    double v = initialSpeed;
    while (v > 0.0 && !eq(v, 0.0)) {
        double curV = v;
        v = std::max(0.0, v - rules->ROBOT_ACCELERATION / TICKS);
        result += length(deltaPos(p3d(curV, 0, 0), p3d(v, 0, 0), 100));
        //result += v / TICKS;
    }
    return result;
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

    if (fabs(goalieX) > rules->arena.goal_width/2 - 5.0)
        goalieX *= (rules->arena.goal_width/2 - 5.0) / fabs(goalieX);

    double goalieZ = -arena.depth/2 - rules->BALL_RADIUS;

//    double closestEnemy = 40;
//    for (Robot r: game->robots)
//    {
//        if (r.player_id == me->player_id)
//            continue;
//        if (r.z < closestEnemy)
//            closestEnemy = r.z;
//    }

//    if (closestEnemy > -20)
//        goalieZ -= (arena.goal_depth - arena.bottom_radius)/2;

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

void MyStrategy::getBehindNextLanding()
{
    auto tooFar = [&](p3d ballpos) -> bool
    {
        return ballpos.y > 2 * game->ball.radius ||
                fabs(ballpos.x) > rules->arena.width/2 - rules->arena.bottom_radius;
    };

    p3d ballpos(game->ball.x,game->ball.y, game->ball.z);
    p3d ballv(game->ball.velocity_x, game->ball.velocity_y, game->ball.velocity_z);
    int t = 1;
    while(tooFar(ballpos) || sprintTime(ballpos, me) > t)
    {
        simulateTick(ballpos, ballv);
        ++t;
    }
    //p3d positionBehind = alignHitTo(p3d(0, 0, rules->arena.depth/2), ballpos);
    if (me->x < ballpos.x)
        ballpos.x -= game->ball.radius + me->radius;
    else
        ballpos.x += game->ball.radius + me->radius;

    runTo(ballpos);
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
    double myY = me->y;// + me->radius/2;
    double myVY = rules->ROBOT_MAX_JUMP_SPEED;
    while(myVY > 0.0)
    {
        double curV = myVY;
        myVY -= rules->GRAVITY / TICKS;
        double deltaY = length(deltaPos(p3d(curV, 0, 0), p3d(myVY, 0, 0), 100));
        myY += deltaY;
        //myY += myVY / TICKS;
    }

    return myY;
}

std::pair<bool, int> MyStrategy::timeToElevate(double height)
{
    int ticks = 0;
    double myY = me->y;
    double myVY = me->touch ? rules->ROBOT_MAX_JUMP_SPEED : me->velocity_y;
    while(myY < height && myVY > 0.0)
    {
        double curVY = myVY;
        myVY -= rules->GRAVITY / TICKS;
        double deltaY = length(deltaPos(p3d(curVY, 0, 0), p3d(myVY, 0, 0), 100));
        myY += deltaY;
        //myY += myVY / TICKS;
        ++ticks;
    }

    return { myY >= height, ticks };
}

//int MyStrategy::timeToElevate(double height)
//{
//    int ticksToGoUp = 0;
//    double myY = me->y;
//    double myVY = rules->ROBOT_MAX_JUMP_SPEED;
//    while(myY < height && myVY > 0.0)
//    {
//        myVY -= rules->GRAVITY / TICKS;
//        myY += myVY / TICKS;
//        ++ticksToGoUp;
//    }

//    return myY < height ? -1 : ticksToGoUp;
//}

bool MyStrategy::inGoalSector(p3d ballPos)
{
    p3d directionToBall = normalize(ballPos - p3d(me->x, me->y, me->z));
    p3d rightTopCorner = p3d(-rules->arena.goal_width/2 + game->ball.radius, rules->arena.goal_height - game->ball.radius, rules->arena.depth/2);
    p3d directionToRightTopCorner = normalize(rightTopCorner - p3d(me->x, me->y, me->z));
    p3d leftBottomCorner = p3d(rules->arena.goal_width/2 - game->ball.radius, game->ball.radius, rules->arena.depth/2);
    p3d directionToLeftBottomCorner = normalize(leftBottomCorner - p3d(me->x, me->y, me->z));

    return directionToRightTopCorner.x <= directionToBall.x && directionToBall.x <= directionToLeftBottomCorner.x
            //&& directionToRightTopCorner.y >= directionToBall.y && directionToBall.y >= directionToLeftBottomCorner.y
            && ballPos.y < maxElevation()
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

p3d MyStrategy::getBestGoalTarget(p3d ballpos)
{
    int cornerSign = (fabs(ballpos.x) < rules->arena.goal_width/2 - rules->BALL_RADIUS) ?
                0 : static_cast<int>(-sign(ballpos.x));
    return p3d((rules->arena.goal_width/2 - rules->BALL_RADIUS) * cornerSign,
               rules->arena.goal_height - game->ball.radius,
               rules->arena.depth/2 + rules->arena.goal_depth);
}

bool MyStrategy::pickShootingPoint(int ticks, futurePoint& bestTarget, futurePoint& bestBall, int& bestPace, int& bestElevationTime)
{
    p3d ballpos(game->ball.x,game->ball.y, game->ball.z);
    p3d ballv(game->ball.velocity_x, game->ball.velocity_y, game->ball.velocity_z);
    bestPace = -100;
    bestElevationTime = -100;
    double maxH = maxElevation();
    for (int t = 1; t < ticks; ++t)
    {
        simulateTick(ballpos, ballv);

        if (t % 10 == 0)
            addSphere(sphere(ballpos, rules->BALL_RADIUS, p3d(1.0, 1.0, 0.0)));

        if (fabs(ballpos.x) >= rules->arena.width/2 - rules->arena.bottom_radius)
            continue;

        p3d goalTarget = getBestGoalTarget(ballpos);
        p3d moveTarget = alignHitTo(goalTarget, ballpos,
                                    distanceXZ(ballpos, goalTarget) > rules->arena.depth/2);

        if (moveTarget.z > rules->arena.depth/2 &&
                fabs(moveTarget.x) > rules->arena.width/2 - rules->arena.bottom_radius)
            continue;

        if (moveTarget.y > maxH)
            continue;

        p3d v1 = (goalTarget - ballpos).to2d();
        p3d v2 = (moveTarget - p3d(me->x, 0.0, me->z)).to2d();
        double angle = acos(dot(v1, v2) / length(v1) / length(v2));
        if (ballpos.z > rules->arena.depth/2 && angle > M_PI_4)
            continue;

        futurePoint target(moveTarget, ballv, t);

        int pace, elevationTime;
        std::tie(pace, elevationTime) = measureShot(target);

        if (elevationTime != -1 &&
                (abs(pace) < abs(bestPace) || (abs(pace) == abs(bestPace) && pace > bestPace)))
        {
            bestPace = pace;
            bestElevationTime = elevationTime;
            bestTarget = target;
            bestBall = futurePoint(ballpos, ballv, t);
        }

        if (abs(bestPace) < 2)
            break;
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

std::pair<int, int> MyStrategy::setInterceptionPoint(const std::vector<futurePoint>& interceptionPoints)
{
    int iTime = 0;
    for (int i = 0; i < static_cast<int>(interceptionPoints.size()); ++i)
    {
        futurePoint iPoint = interceptionPoints[i];
        iTime = sprintTime(iPoint.pos, me);
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
        {
            if (m_clearerId == me->id)
                m_clearerId = -1;
            action->jump_speed = rules->ROBOT_MAX_JUMP_SPEED;
        }

        return true;
    }
    else if (reach == -1)
    {
        double desiredSpeed = rules->ROBOT_MAX_GROUND_SPEED * sprintTime / point.t;
        setSpeed(desiredSpeed, p3d(point.pos.x - me->x, 0.0, point.pos.z - me->z));

        if (m_role == Role::Goalie && 30 >= point.t)
        {
            if (m_clearerId == me->id)
                m_clearerId = -1;
            action->jump_speed = rules->ROBOT_MAX_JUMP_SPEED;
        }

        return true;
    }

    return false;
}

std::pair<int, int> MyStrategy::measureShot(futurePoint target)
{
    int elevationTime = timeToElevate(target.pos.y).second;
    if (elevationTime == -1)
        return { -criticalPaceDiff, -1 };

    int t = sprintTime(target.pos, me);//interceptionTime(target.pos, me, elevationTime);//
    int pace = target.t - t;

    return { pace, elevationTime };
}

void MyStrategy::intercept(const std::vector<futurePoint>& interceptionPoints, bool homeOnly)
{
    int interceptionTime = 0, iPointIndex = -1;
    std::tie(iPointIndex, interceptionTime) = setInterceptionPoint(interceptionPoints);

    if (iPointIndex < 0 || iPointIndex >= interceptionPoints.size())
        return;

    futurePoint iPoint = interceptionPoints[static_cast<unsigned int>(iPointIndex)];
    if (homeOnly && iPoint.pos.z > -rules->arena.depth/4)
        return;

    m_clearerId = me->id;
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

p3d MyStrategy::alignHitTo(p3d target, p3d ball, bool under45)
{
    p3d v = target - ball;
    if (under45)
        v.y = sqrt(v.x*v.x + v.z*v.z);
    double r1r2 = rules->BALL_RADIUS + rules->ROBOT_RADIUS/2; //!
    p3d hitOffset = normalize(v) * r1r2;
    return ball - hitOffset;
}

void MyStrategy::alignShot()
{
    p3d goalTarget = getShotTarget();
    //addSphere(sphere(goalTarget, rules->BALL_RADIUS, p3d(1.0, 0.0, 1.0)));

    futurePoint target;
    p3d runFrom;
    p3d ballpos(game->ball.x,game->ball.y, game->ball.z);
    p3d ballv(game->ball.velocity_x, game->ball.velocity_y, game->ball.velocity_z);
    double maxH = maxElevation();
    for (int t = 1; t < 500; ++t)
    {
        simulateTick(ballpos, ballv);
        if (fabs(ballpos.x) >= rules->arena.width/2 - rules->arena.bottom_radius)
            continue;
        futurePoint ballSim(ballpos, ballv, t);

        futurePoint ptarget(alignHitTo(goalTarget, ballSim.pos), ballv, t);
        if (ptarget.pos.y > maxH)
            continue;

        runFrom = getRunupPosition(ptarget, goalTarget);
        if (fabs(runFrom.x) >= rules->arena.width/2 - rules->arena.bottom_radius)
            continue;
        int runupTime = hasRunUp ? 0 : timeToRun(runFrom, p3d(me->x, me->y, me->z), p3d(me->velocity_x, me->velocity_y, me->velocity_z));
        if (runupTime > t)
            continue;

        double dist = 0.0;
        int shotTime = timeToChangeSpeed(normalize(ptarget.pos - runFrom) * rules->ROBOT_MAX_GROUND_SPEED, p3d(), dist);
        if (runupTime + shotTime > t)
            continue;

        target = ptarget;
        //addSphere(sphere(target.pos, rules->ROBOT_RADIUS, p3d(1.0, 0.0, 0.0)));
        //addSphere(sphere(runFrom, rules->ROBOT_RADIUS, p3d(1.0, 1.0, 1.0)));
        //addSphere(sphere(ballSim.pos, rules->BALL_RADIUS, p3d(0.0, 0.0, 0.0)));

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
    if (eq(game->ball.velocity_x, 0.0) && eq(game->ball.x, 0.0) &&
            eq(game->ball.velocity_z, 0.0) && eq(game->ball.z, 0.0)) // kickoff
    {
        sprintTo(p3d(), true);
        action->use_nitro = true;
        return;
    }

//    if (m_clearerId == getTeammate().first)
//    {
//        return;
//    }

    if (me->z + me->radius > game->ball.z && game->ball.velocity_z < 0)
    {
        getBehindNextLanding();
        return;
    }

    m_text = "";
    if (m_plannedAttackerTarget.isValid())
    {
        // we have a planned target
        p3d ballpos(game->ball.x, game->ball.y, game->ball.z);
        p3d ballv(game->ball.velocity_x, game->ball.velocity_y, game->ball.velocity_z);
        for (int t = game->current_tick + 1; t <= m_plannedAttackerTarget.tick; ++t)
        {
            simulateTick(ballpos, ballv);
        }
        addSphere(sphere(m_plannedAttackerTarget.ball, rules->BALL_RADIUS, p3d(0.0, 0.0, 0.0)));
        addSphere(sphere(m_plannedAttackerTarget.me, rules->ROBOT_RADIUS, p3d(1.0, 0.0, 1.0)));
        double error = length(ballpos - m_plannedAttackerTarget.ball);

        if (error < 0.01)
        {
            m_plannedAttackerTarget.ball = ballpos;

            futurePoint target(m_plannedAttackerTarget.me, p3d(), m_plannedAttackerTarget.tick - game->current_tick);
            int pace, elevationTime;
            std::tie(pace, elevationTime) = measureShot(target);
            if (pace > 0)
            {
                m_text = "waiting " + std::to_string(pace);
                return;
            }

            m_text = "shooting " + std::to_string(pace);
            setSpeed(rules->ROBOT_MAX_GROUND_SPEED, p3d(m_plannedAttackerTarget.me.x - me->x, 0.0, m_plannedAttackerTarget.me.z - me->z));
            if (abs(pace) < 2 & me->velocity_z > 0 && (m_plannedAttackerTarget.elevationTime >= m_plannedAttackerTarget.tick - game->current_tick))
            {
                action->jump_speed = rules->ROBOT_MAX_JUMP_SPEED;
                if (m_plannedAttackerTarget.tick - game->current_tick < 2)
                    action->use_nitro = true;
            }
            return;
        }
        addSphere(sphere(ballpos, rules->BALL_RADIUS, p3d(1.0, 1.0, 1.0)));
    }

    futurePoint shootFrom;
    futurePoint ballFrom;
    int shootingPace, elevationTime;

    if (!me->touch)
        return;

    m_plannedAttackerTarget.invalidate();

    if (pickShootingPoint(200, shootFrom, ballFrom, shootingPace, elevationTime))
    {
        m_text = "New shot " + std::to_string(shootingPace);
        addSphere(sphere(ballFrom.pos, rules->BALL_RADIUS, p3d(0.0, 0.0, 0.0)));
        addSphere(sphere(shootFrom.pos, rules->ROBOT_RADIUS, p3d(1.0, 0.0, 1.0)));

        m_plannedAttackerTarget = PlannedShot(ballFrom.pos, shootFrom.pos, game->current_tick + shootFrom.t, elevationTime);
        setSpeed(rules->ROBOT_MAX_GROUND_SPEED, p3d(m_plannedAttackerTarget.me.x - me->x, 0.0, m_plannedAttackerTarget.me.z - me->z));
    }
    else
    {
        m_text = "aligning !";
        return;
        hasRunUp = false;
        alignShot();
    }
}

bool MyStrategy::isConsistent(const MyStrategy::PlannedShot &plannedShot)
{
    p3d ballpos(game->ball.x, game->ball.y, game->ball.z);
    p3d ballv(game->ball.velocity_x, game->ball.velocity_y, game->ball.velocity_z);
    for (int t = game->current_tick + 1; t <= plannedShot.tick; ++t)
    {
        simulateTick(ballpos, ballv);
    }
    double error = length(ballpos - m_plannedGoalieTarget.ball);
    return error < 1;//0.01;
}

void MyStrategy::C_defend()
{
    double mel = maxElevation();
    if (m_clearerId == me->id)
        m_clearerId = -1;

//    std::vector<futurePoint> interceptionPoints;
//    bool onTarget = ballGoesToGoal(game->ball, interceptionPoints);
//    if (onTarget)
//    {
//        intercept(interceptionPoints, true);
//        return;
//    }

    m_text = "";
    if (!me->touch)
    {
        m_text = "just flying";
        m_plannedGoalieTarget.invalidate();
        return;
    }

    if (!m_plannedGoalieTarget.isValid() || !isConsistent(m_plannedGoalieTarget))
    {
        m_plannedGoalieTarget.invalidate();
        bool goalLine = false;
        bool newPlan = setInterceptionPoint(goalLine);

        if (!newPlan)
        {
            p3d pos = getGoalieDefaultPosition(game->ball);
            runTo(pos);
            return;
        }
        else
        {
            m_text = "NEW PLAN ";
        }
    }

    // going for the planned shot
    addSphere(sphere(m_plannedGoalieTarget.ball, rules->BALL_RADIUS, p3d(0.0, 0.0, 0.0)));
    addSphere(sphere(m_plannedGoalieTarget.me, rules->ROBOT_RADIUS, p3d(1.0, 0.0, 1.0)));

    action->target_velocity_x = m_interceptionPlan[game->current_tick].targetV.x;
    action->target_velocity_y = m_interceptionPlan[game->current_tick].targetV.y;
    action->target_velocity_z = m_interceptionPlan[game->current_tick].targetV.z;
    action->jump_speed = m_interceptionPlan[game->current_tick].jump ? rules->ROBOT_MAX_JUMP_SPEED : 0.0;

    double speedError = length(p3d(me->velocity_x, me->velocity_y, me->velocity_z) - m_interceptionPlan[game->current_tick].curV);
    double posError = length(p3d(me->x, me->y, me->z) - m_interceptionPlan[game->current_tick].curPos);

    std::string stage = m_interceptionPlan[game->current_tick].stage;
    m_text += stage + " " + std::to_string(speedError) + " " + std::to_string(posError);
}

bool MyStrategy::fasterOpponent(p3d ballpos, int t)
{
    for (Robot r: game->robots)
    {
        if (r.player_id == me->player_id)
            continue;

        p3d oppPos(r.x, r.y, r.z);
        p3d oppAligned = alignHitTo(oppPos + (ballpos - oppPos) * 2, ballpos);
        int interceptTime = sprintTime(oppAligned, &r); // change to interception time?
        if (interceptTime <= t)
            return true;
    }

    return false;
}

bool MyStrategy::setInterceptionPoint(bool& goalLine)
{
    p3d ballpos(game->ball.x, game->ball.y, game->ball.z);
    p3d ballv(game->ball.velocity_x, game->ball.velocity_y, game->ball.velocity_z);

    PlannedShot interceptionPoint;
    p3d goalBack(0, 0, -rules->arena.depth/2 - rules->arena.goal_depth + rules->arena.bottom_radius);

    for (int t = 1; t < 300; ++t)
    {
        p3d prevBallpos = ballpos;
        simulateTick(ballpos, ballv);

        if (t % 10 == 1)
           addSphere(sphere(ballpos, game->ball.radius, p3d(0.0, 1.0, 0.0)));

        if (ballpos.z + game->ball.radius < -rules->arena.depth/2)
        {
            goalLine = true;
            p3d shootFrom = alignHitTo(goalBack + (prevBallpos - goalBack) * 2, prevBallpos);

            bool res = makeInterceptionPlan(shootFrom, game->current_tick + t - 1, true);

            m_plannedGoalieTarget = PlannedShot(prevBallpos, shootFrom, game->current_tick + t - 1, timeToElevate(shootFrom.y).second);
            return true;
        }

        if (fabs(ballpos.x) > rules->arena.goal_width/2 + rules->arena.goal_side_radius)
            continue;

        if (ballpos.z > 0)// || fasterOpponent(ballpos, t))
            continue;

        p3d shootFrom = alignHitTo(goalBack + (ballpos - goalBack) * 2, ballpos);

        if (makeInterceptionPlan(shootFrom, game->current_tick + t, false))
        {
            m_plannedGoalieTarget = PlannedShot(ballpos, shootFrom, game->current_tick + t, timeToElevate(shootFrom.y).second);
            return true;
        }
    }

    return false;
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

    //    for (auto ip: points)
    //    {
    //        addSphere(sphere(ip.pos, ball.radius, p3d(0.0, 0.0, 1.0)));
    //    }

    simulateTick(ballpos, ballv);
    points.push_back(futurePoint(ballpos, ballv, ticks));
    //addSphere(sphere(ballpos, ball.radius, p3d(0.0, 0.0, 1.0)));
}

bool MyStrategy::ballGoesToGoal(const Ball &ball, std::vector<futurePoint>& interceptionPoints)
{
    interceptionPoints.clear();

    if (ball.velocity_z >= 0.0) // от ворот
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

int MyStrategy::sprintTime(p3d at, const Robot* robot)
{
    double myDistance = distanceXZ(p3d(robot->x, robot->y, robot->z), at);
    p3d normal(at.x - robot->x, 0.0, at.z - robot->z);
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

int MyStrategy::interceptionTime(p3d at, const Robot *robot, int elevationTime)
{
    double myDistance = distanceXZ(p3d(robot->x, robot->y, robot->z), at);
    p3d normal(at.x - robot->x, 0.0, at.z - robot->z);
    double normalLength = length(normal);
    p3d myV(robot->velocity_x, 0, robot->velocity_z);
    double vAlong = eq(normalLength, 0.0) ? length(myV) : (dot(normal, myV) / normalLength);
    double distanceCovered = 0.0;
    int takesTicks = 0;
    while (distanceCovered + vAlong / TICKS * elevationTime < myDistance)
    {
        ++takesTicks;

        double vCur = vAlong;
        vAlong += rules->ROBOT_ACCELERATION * (1.0 / TICKS);
        vAlong = std::min(vAlong, rules->ROBOT_MAX_GROUND_SPEED);

        double deltaP = length(deltaPos(p3d(vCur, 0, 0), p3d(vAlong, 0, 0), 100));

        distanceCovered += deltaP;
    }

    return takesTicks;
}

p3d MyStrategy::deltaPos(p3d fromV, p3d toV, int mt)
{
    p3d deltaP;
    p3d deltaV = (toV - fromV) * (1.0 / mt);

    p3d a1 = (fromV + deltaV) * (1.0 / TICKS / mt);
    p3d an = (fromV + deltaV * mt) * (1.0 / TICKS / mt);

    return (a1 + an) * (static_cast<double>(mt) / 2.0);

    for (int i = 0; i < mt; ++i)
    {
        fromV = fromV + deltaV;
        deltaP = deltaP + fromV * (1.0 / TICKS / mt);
    }
    return deltaP;
}

bool MyStrategy::makeInterceptionPlan(p3d at, int targetTick, bool must)
{
    m_interceptionPlan.clear();

    std::pair<bool, int> elevationTime = timeToElevate(at.y);
    if (!elevationTime.first)
    {
        if (must)
            at.y = maxElevation();
        else
            return false;
    }

    p3d myPos(me->x, me->y, me->z);
    p3d myV(me->velocity_x, me->velocity_y, me->velocity_z);

    m_interceptionPlan[game->current_tick].curV = myV;
    m_interceptionPlan[game->current_tick].curPos = myPos;

    // 1. Stop
    int stopTime = 0;
    while (!eq(length(myV), 0.0)) {
        p3d targetV = myV * (-1.0);
        m_interceptionPlan[game->current_tick + stopTime].targetV = targetV;
        m_interceptionPlan[game->current_tick + stopTime].jump = false;
        m_interceptionPlan[game->current_tick + stopTime].stage = "Stop";
        p3d myVCur = myV;
        ++stopTime;
        myV = normalize(myV) * std::max(0.0, (length(myV) - rules->ROBOT_ACCELERATION * (1.0 / TICKS)));
        myPos = myPos + deltaPos(myVCur, myV, 100);
        //myPos = myPos + myV * (1.0 / TICKS);
        m_interceptionPlan[game->current_tick + stopTime].curPos = myPos;
        m_interceptionPlan[game->current_tick + stopTime].curV = myV;
    }

    // 2. Run-up
    double flyDist = elevationTime.second * rules->ROBOT_MAX_GROUND_SPEED / TICKS;
    double accelDist = brakeDistance(rules->ROBOT_MAX_GROUND_SPEED);

    p3d goalBack(0, 0, -rules->arena.depth/2 - rules->arena.goal_depth + rules->arena.bottom_radius);

    p3d runupPos = at + normalize(goalBack - at) * (flyDist + accelDist);
    if (runupPos.z < goalBack.z)
        runupPos = goalBack;
    if (runupPos.z > -rules->arena.depth/2)
    {
        runupPos = goalBack + (at - goalBack) * ((rules->arena.goal_depth - rules->arena.bottom_radius) / (at.z - goalBack.z)); // goalline
    }
    runupPos.y = me->radius;
    double distToRunup = distanceXZ(runupPos, myPos);
    int runupTime = 0;
    while (distToRunup > 0.02 && !eq(distToRunup, 0)) {
        p3d targetV;
        double upV = std::min(rules->ROBOT_MAX_GROUND_SPEED, length(myV) + rules->ROBOT_ACCELERATION / TICKS);
        double upDeltaPos = length(deltaPos(myV, normalize(myV)* upV, 100));
        bool acc = distToRunup - upDeltaPos > brakeDistance(upV);
        double newVLen;
        if (acc)
        {
            targetV = normalize(runupPos - myPos) * rules->ROBOT_MAX_GROUND_SPEED;
            newVLen = std::min(rules->ROBOT_MAX_GROUND_SPEED, length(myV) + rules->ROBOT_ACCELERATION * (1.0 / TICKS));
        }
        else
        {
            targetV = p3d();
            newVLen = std::max(0.0, length(myV) - rules->ROBOT_ACCELERATION * (1.0 / TICKS));
        }
        m_interceptionPlan[game->current_tick + stopTime + runupTime].targetV = targetV;
        m_interceptionPlan[game->current_tick + stopTime + runupTime].jump = false;
        m_interceptionPlan[game->current_tick + stopTime + runupTime].stage = "Runup";
        p3d myVCur = myV;
        ++runupTime;
        myV = normalize(runupPos - myPos) * newVLen;
        p3d deltaP = deltaPos(myVCur, myV, 100);
        myPos = myPos + deltaP;
        //myPos = myPos + myV * (1.0 / TICKS);
        distToRunup -= length(deltaP);//newVLen * (1.0 / TICKS); //
        m_interceptionPlan[game->current_tick + stopTime + runupTime].curPos = myPos;
        m_interceptionPlan[game->current_tick + stopTime + runupTime].curV = myV;
    }

    // 3. Calculate Pace
    int timeToTakeoff = targetTick - game->current_tick - stopTime - runupTime - elevationTime.second;
    double distToTarget = distanceXZ(at, runupPos);

    double vAlong = 0.0;
    double distanceCovered = 0.0;
    int takesTicks = 0;
    while (distanceCovered + vAlong / TICKS * elevationTime.second < distToTarget)
    {
        ++takesTicks;

        double vCur = vAlong;

        vAlong += rules->ROBOT_ACCELERATION * (1.0 / TICKS);
        vAlong = std::min(vAlong, rules->ROBOT_MAX_GROUND_SPEED);

        double deltaP = length(deltaPos(p3d(vCur, 0, 0), p3d(vAlong, 0, 0), 100));
        distanceCovered += deltaP;//vAlong * (1.0 / TICKS);
    }
    int pace = timeToTakeoff - takesTicks;
    if (pace < 0)
    {
        if (must)
        {
            pace += runupTime;

            auto planIter = m_interceptionPlan.begin();
            auto nextIter = m_interceptionPlan.end();
            for (; planIter != m_interceptionPlan.end(); ++planIter)
                if (planIter->second.stage != "Stop")
                {
                    nextIter = planIter;
                    std::advance(nextIter, 1);
                    break;
                }
            m_interceptionPlan.erase(nextIter, m_interceptionPlan.end());

            //m_interceptionPlan.clear();

            if (planIter != m_interceptionPlan.end())
            {
                myPos = planIter->second.curPos;
                myV = planIter->second.curV;
            }
            else
            {
                myPos = p3d(me->x, me->y, me->z);
                myV = p3d(me->velocity_x, me->velocity_y, me->velocity_z);
            }

            runupTime = 0;

            runupPos = p3d(at.x, me->radius, at.z);
            addSphere(sphere(runupPos, me->radius, p3d(1.0, 1.0, 1.0)));

            distToRunup = distanceXZ(runupPos, myPos);
            while (distToRunup > 0.02 && !eq(distToRunup, 0)) {
                p3d targetV;
                double upV = std::min(rules->ROBOT_MAX_GROUND_SPEED, length(myV) + rules->ROBOT_ACCELERATION / TICKS);
                double upDeltaPos = length(deltaPos(myV, normalize(myV)* upV, 100));
                bool acc = distToRunup - upDeltaPos > brakeDistance(upV);
                double newVLen;
                if (acc)
                {
                    targetV = normalize(runupPos - myPos) * rules->ROBOT_MAX_GROUND_SPEED;
                    newVLen = std::min(rules->ROBOT_MAX_GROUND_SPEED, length(myV) + rules->ROBOT_ACCELERATION * (1.0 / TICKS));
                }
                else
                {
                    targetV = p3d();
                    newVLen = std::max(0.0, length(myV) - rules->ROBOT_ACCELERATION * (1.0 / TICKS));
                }
                m_interceptionPlan[game->current_tick + stopTime + runupTime].targetV = targetV;
                m_interceptionPlan[game->current_tick + stopTime + runupTime].jump = false;
                m_interceptionPlan[game->current_tick + stopTime + runupTime].stage = "Short Runup";
                p3d myVCur = myV;
                ++runupTime;
                myV = normalize(runupPos - myPos) * newVLen;
                p3d deltaP = deltaPos(myVCur, myV, 100);
                myPos = myPos + deltaP;
                distToRunup -= length(deltaP);
                m_interceptionPlan[game->current_tick + stopTime + runupTime].curPos = myPos;
                m_interceptionPlan[game->current_tick + stopTime + runupTime].curV = myV;
            }

            timeToTakeoff = targetTick - game->current_tick - stopTime - runupTime - elevationTime.second;
        }
        else
        {
            m_interceptionPlan.clear();
            return false;
        }
    }

    // 4. Wait
    int waitTime = 0;
    while (waitTime < pace) {
        m_interceptionPlan[game->current_tick + stopTime + runupTime + waitTime].targetV = p3d();
        m_interceptionPlan[game->current_tick + stopTime + runupTime + waitTime].jump = false;
        m_interceptionPlan[game->current_tick + stopTime + runupTime + waitTime].stage = "Wait";
        ++waitTime;
        m_interceptionPlan[game->current_tick + stopTime + runupTime + waitTime].curPos = myPos;
        m_interceptionPlan[game->current_tick + stopTime + runupTime + waitTime].curV = myV;
    }

    // 5. Accelerate + run
    int timeToRun = timeToTakeoff - std::max(0, pace);
    int runTime = 0;
    p3d runNormal = normalize(p3d(at.x, me->radius, at.z) - runupPos);
    while (runTime <= timeToRun) {
        p3d targetV = runNormal * rules->ROBOT_MAX_GROUND_SPEED;
        m_interceptionPlan[game->current_tick + stopTime + runupTime + waitTime + runTime].targetV = targetV;
        m_interceptionPlan[game->current_tick + stopTime + runupTime + waitTime + runTime].jump = false;
        m_interceptionPlan[game->current_tick + stopTime + runupTime + waitTime + runTime].stage = "Run";
        p3d curV = myV;
        ++runTime;
        double newVLen = std::min(rules->ROBOT_MAX_GROUND_SPEED, length(myV) + rules->ROBOT_ACCELERATION * (1.0 / TICKS));
        myV = runNormal * newVLen;
        myPos = myPos + deltaPos(curV, myV, 100);//myV * (1.0 / TICKS);
        m_interceptionPlan[game->current_tick + stopTime + runupTime + waitTime + runTime].curPos = myPos;
        m_interceptionPlan[game->current_tick + stopTime + runupTime + waitTime + runTime].curV = myV;
    }

    // 6. Jump
    int jumpTime = game->current_tick + stopTime + runupTime + waitTime + runTime;

    int ttt = timeToElevate(at.y + rules->BALL_RADIUS/2).second;
    m_interceptionPlan[pace < 0 ? (jumpTime - ttt) : (jumpTime - 1)].jump = true; //hack

    while (jumpTime < targetTick) {
        p3d targetV = myV;
        m_interceptionPlan[jumpTime].targetV = targetV;
        m_interceptionPlan[jumpTime].jump = true;
        m_interceptionPlan[jumpTime].stage = "Jump";
        ++jumpTime;
        myPos = myPos + myV * (1.0 / TICKS);
        m_interceptionPlan[jumpTime].curPos = myPos;
        m_interceptionPlan[jumpTime].curV = myV;
    }

    return true;
}

int MyStrategy::canReachInTime(futurePoint at, int& sprintT, int& elevationTime)
{
    if (at.t == 0)
        return -2;

    sprintT = sprintTime(at.pos, me);
    if (sprintT > at.t)
        return -2; // too far

    elevationTime = timeToElevate(at.pos.y).second;
    if (elevationTime == -1)
    {
        return -3; // too high
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

p3d &p3d::operator=(const p3d &other)
{
    x = other.x;
    y = other.y;
    z = other.z;
    return *this;
}
p3d p3d::to2d()
{
    return p3d(x, 0.0, z);
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

MyStrategy::PlannedShot::PlannedShot() : ball(), me(), tick(0), elevationTime(-1) {}
MyStrategy::PlannedShot::PlannedShot(p3d b, p3d m, int t, int et) : ball(b), me(m), tick(t), elevationTime(et) {}
bool MyStrategy::PlannedShot::isValid()
{
    return tick > 0;
}
void MyStrategy::PlannedShot::invalidate()
{
    *this = PlannedShot();
}
