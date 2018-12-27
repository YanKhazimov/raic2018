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
    : m_role(Role::Unassigned)
{

}

const int TICKS = 60;
double brakeDistance(const Rules& rules)
{
    double result = 0.0;
    double v = rules.ROBOT_MAX_GROUND_SPEED;
    while (v > 0.0) {
        result += v / TICKS;
        v -= rules.ROBOT_ACCELERATION / TICKS;
    }
    return result;
}

bool eq(double a, double b)
{
    return fabs(a - b) <= std::numeric_limits<double>::epsilon();
}

double distanceXZ(p3d a, p3d b)
{
    return sqrt((a.x - b.x)*(a.x - b.x) + (a.z - b.z)*(a.z - b.z));
}

p3d MyStrategy::getGoalieDefaultPosition(const Rules& rules, p3d ballPosition)
{
    const Arena& arena = rules.arena;

    // by bissectrisa
    double d1 = distanceXZ(ballPosition, {-arena.goal_width/2, 0, -arena.depth/2});
    double d2 = distanceXZ(ballPosition, {arena.goal_width/2, 0, -arena.depth/2});

    double x = -arena.goal_width/2 + arena.goal_width * d1 / (d1 + d2);

    p3d result {x, 0.0, -arena.depth/2};

    return result;
}

double sign(double x)
{
    return static_cast<double>((0.0 < x) - (x < 0.0));
}

void setSpeed(double value, p3d normal, Action& action)
{
    if (eq(0.0, normal.x))
        action.target_velocity_z = sign(normal.z) * value;
    else if (eq(0.0, normal.z))
        action.target_velocity_x = sign(normal.x) * value;
    else
    {
        action.target_velocity_z = sign(normal.z) * sqrt(value * value / (1 + normal.x/normal.z * normal.x/normal.z));
        action.target_velocity_x = sign(normal.x) * fabs(action.target_velocity_z * normal.x/normal.z);
    }
}

void runTo(p3d to, const Robot& me, const Rules& rules, Action& action)
{
    if (distanceXZ(to, p3d(me.x, me.y, me.z)) > brakeDistance(rules))
    {
        setSpeed(rules.ROBOT_MAX_GROUND_SPEED, p3d(to.x - me.x, 0.0, to.z - me.z), action);
    //action.target_velocity_x = rules.ROBOT_MAX_GROUND_SPEED * (to.x - me.x);
    //action.target_velocity_z = rules.ROBOT_MAX_GROUND_SPEED * (to.z - me.z);
    }
    else
    {
        action.target_velocity_x = 0.0;
        action.target_velocity_z = 0.0;
    }
}

void sprintTo(p3d to, const Robot& me, const Rules& rules, Action& action)
{
    setSpeed(rules.ROBOT_MAX_GROUND_SPEED, p3d(to.x - me.x, 0.0, to.z - me.z), action);
}

void MyStrategy::C_bullyGoalie(const Robot& me, const Rules& rules, const Game& game, Action& action)
{
    p3d goaliePos { 0, 0, -rules.arena.depth };

    for (Robot r: game.robots)
    {
        if (r.player_id == me.player_id)
            continue;

        if (r.z > goaliePos.z)
        {
            goaliePos = { r.x, r.y, r.z };
        }
    }

    sprintTo(goaliePos, me, rules, action);
}

void MyStrategy::C_bullyAttacker(const Robot& me, const Rules& rules, const Game& game, Action& action)
{
    p3d goaliePos { 0, 0, rules.arena.depth };

    for (Robot r: game.robots)
    {
        if (r.player_id == me.player_id)
            continue;

        if (r.z < goaliePos.z)
        {
            goaliePos = { r.x, r.y, r.z };
        }
    }

    sprintTo(goaliePos, me, rules, action);
}

void MyStrategy::getRole(const Robot& me, const Game& game)
{
    for (Robot r: game.robots)
    {
        if (r.player_id != me.player_id || r.id == me.id)
            continue;

        if (r.z < me.z ||
                eq(r.z, me.z) && r.velocity_z < me.velocity_z ||
                    eq(r.z, me.z) && eq(r.velocity_z, me.velocity_z) && r.id < me.id)

        {
            m_role = Role::Attacker;
        }
        else
        {
            m_role = Role::Goalie;
        }
    }
}

void MyStrategy::getBehindBall(const Robot& me, const Rules& rules, const Game& game, Action& action)
{
    p3d destination(game.ball.x, game.ball.y, game.ball.z - game.ball.radius);

    if (me.x < game.ball.x)
        destination.x -= game.ball.radius + me.radius;
    else
        destination.x += game.ball.radius + me.radius;

    m_spheres.push_back(sphere(destination.x, destination.y, destination.z,
                               me.radius, p3d(0.0, 1.0, 0.0)));

    sprintTo(destination, me, rules, action);
}

bool isRolling(const model::Ball& ball)
{
    return fabs(ball.velocity_y) < 7.0 && ball.y < 2 * ball.radius;
}

bool isCentering(const Rules& rules, const Game& game, MyStrategy::futurePoint& point)
{
    if (!isRolling(game.ball))
        return false;

    if (game.ball.velocity_z < 0.2)//rules.ROBOT_MAX_GROUND_SPEED / 2)
        return false;

    p3d ball(game.ball.x,game.ball.y, game.ball.z);

    bool z = true, x = true;
    while ((x = fabs(ball.x) < rules.arena.width/2 - rules.arena.bottom_radius) &&
           (z = ball.z < rules.arena.depth/2 - rules.arena.bottom_radius))
    {
        ball.x += game.ball.velocity_x / TICKS;
        ball.z += game.ball.velocity_z / TICKS;
    }

    point = {ball, 0};

    return ball.z > rules.arena.depth/2 - rules.arena.corner_radius &&
            fabs(ball.x) > rules.arena.goal_width/2 &&
            (!x || (!z && sign(ball.x) == sign(game.ball.x) && fabs(game.ball.x) > rules.arena.goal_width/2));
}

void MyStrategy::shoot(const Robot& me, const Rules& rules, const Game& game, Action& action)
{
    p3d target;
    futurePoint p;
    if (isCentering(rules, game, p))
    {
        target = p3d(p.first.x > 0.0 ? -rules.arena.goal_width/2 : rules.arena.goal_width/2, 0.0, 30.0);
        m_text = "centering";
        m_spheres.push_back(sphere(target.x, target.y, target.z, me.radius, p3d(1.0, 0.0, 0.0)));
        runTo(target, me, rules, action);
    }
    else if (game.ball.z > rules.arena.depth/2 - rules.arena.corner_radius)
    {
        target = p3d(game.ball.x, game.ball.y, game.ball.z);
        m_text = "";
        runTo(target, me, rules, action);
    }
    else
    {
        target = p3d(game.ball.x, game.ball.y, game.ball.z);
        sprintTo(target, me, rules, action);
    }
}

void MyStrategy::act(const Robot& me, const Rules& rules, const Game& game, Action& action)
{
    if (m_tick_spheres != game.current_tick)
    {
        m_spheres.clear();
        m_tick_spheres = game.current_tick;
    }

    getRole(me, game);

    if (m_role == Role::Attacker)
    {
        p3d ballpos(game.ball.x,game.ball.y, game.ball.z);
        p3d ballv(game.ball.velocity_x, game.ball.velocity_y, game.ball.velocity_z);
        for (int t = 0; t < 10; ++t)
        {

        }


        return;
        if (me.z + me.radius > game.ball.z)
            getBehindBall(me, rules, game, action);
        else
        {

            shoot(me, rules, game, action);
                //m_text = "";

//                p3d ballPos(game.ball.x, game.ball.y, game.ball.z);
//                sprintTo(ballPos, me, rules, action);
//                if (distanceXZ(ballPos, p3d(me.x, me.y, me.z)) < game.ball.radius + 2 * me.radius &&
//                        game.ball.y < 1.5 * game.ball.radius)
//                    action.jump_speed = rules.ROBOT_MAX_JUMP_SPEED;
        }
        return;
    }

    C_defend(me, rules, game, action);
}

std::string MyStrategy::addSphere(double x, double y, double z, double r, p3d rgb)
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

std::string MyStrategy::addText(std::string text)
{
    return "{\"Text\":\"" + text + "\"}";
}

std::string MyStrategy::custom_rendering()
{
    m_json = "[";

    for (sphere sph: m_spheres)
        m_json += addSphere(sph.x, sph.y, sph.z, sph.r, sph.rgb) + ",";

    m_json += addText(m_text);

    m_json += "]";

    return m_json;
}

int timeToElevate(double height, const Robot &me, const Rules &rules)
{
    int ticksToGoUp = 0;
    double myY = me.y;
    double myVY = me.velocity_y;
    while (myY < height)
    {
        myY += myVY / TICKS;
        myVY = std::min(myVY + rules.ROBOT_ACCELERATION / TICKS, rules.ROBOT_MAX_JUMP_SPEED) - rules.GRAVITY / TICKS;
        if (myVY < 0.0)
        {
            return -1;
        }
        ++ticksToGoUp;
    }

    return ticksToGoUp;
}

bool shouldJump(const Robot &me, const MyStrategy::futurePoint& iPoint, const Rules &rules)
{
    return timeToElevate(iPoint.first.y, me, rules) >= iPoint.second;
}

p3d hitPoint(const p3d& iPoint, const Rules &rules)
{
    return p3d(iPoint.x, iPoint.y - rules.BALL_RADIUS/2, iPoint.z - rules.BALL_RADIUS/2);
}

MyStrategy::futurePoint hitPoint(const MyStrategy::futurePoint& iPoint, const Rules &rules)
{
    return { hitPoint(iPoint.first, rules), iPoint.second };
}

std::pair<int, int> MyStrategy::pickInterceptionPoint(const std::vector<futurePoint>& interceptionPoints,
                                          const Robot &me, const Rules &rules)
{
    int iTime = 0;
    for (int i = 0; i < static_cast<int>(interceptionPoints.size()); ++i)
    {
        futurePoint iPoint = interceptionPoints[i];
        iTime = interceptionTime(iPoint, me, rules);
        if (iTime <= iPoint.second)
            return { i, iTime };
    }

    return { static_cast<int>(interceptionPoints.size() - 1), iTime };
}

bool MyStrategy::interceptBounceAt(const futurePoint& point, const Robot &me, const Rules &rules, Action &action)
{
    int sprintTime = 0, elevationTime = 0;
    if (canReachInTime(point, me, rules, sprintTime, elevationTime))
    {
        double desiredSpeed = rules.ROBOT_MAX_GROUND_SPEED * sprintTime / point.second;

        setSpeed(desiredSpeed, p3d(point.first.x - me.x, 0.0, point.first.z - me.z), action);

        if (elevationTime >= point.second)
            action.jump_speed = rules.ROBOT_MAX_JUMP_SPEED;

        return true;
    }

    return false;
}

void MyStrategy::C_defend(const Robot &me, const Rules &rules, const Game &game, Action &action)
{
    p3d pos = getGoalieDefaultPosition(rules, { game.ball.x, game.ball.y, game.ball.z });
    runTo(pos, me, rules, action);

    std::vector<futurePoint> interceptionPoints;
    bool onTarget = ballGoesToGoal(rules, game.ball, interceptionPoints);
    if (onTarget)
    {
        int interceptionTime = 0, iPointIndex = -1;
        std::tie(iPointIndex, interceptionTime) = pickInterceptionPoint(interceptionPoints, me, rules);

        futurePoint iPoint = interceptionPoints[static_cast<unsigned int>(iPointIndex)];
        if (iPoint.first.z > 0.0)
            return;

        //m_text = (interceptionTime <= iPoint.second) ? "can intercept" : "CAN'T INTERCEPT";

        m_spheres.push_back(sphere(iPoint.first.x, iPoint.first.y, iPoint.first.z,
                                   game.ball.radius*1.2, p3d(1.0, 0.0, 0.0)));

        if (isRolling(game.ball))
        {
            if (iPointIndex == 1)
                iPoint = interceptionPoints[0];
            sprintTo(iPoint.first, me, rules, action);

            double distance = distanceXZ(/*iPoint.first*/p3d(game.ball.x, game.ball.y, game.ball.z), p3d(me.x, me.y, me.z));
            if (distance < 2 * me.radius + game.ball.radius)
                action.jump_speed = rules.ROBOT_MAX_JUMP_SPEED;
        }
        else
        {
            // ball is bouncing
            futurePoint pointToHit = hitPoint(iPoint, rules);
            interceptBounceAt(pointToHit, me, rules, action);
        }
    }
    else
    {
        // if I'm fastest to the rolling ball and it's close - charge it
        if (!isRolling(game.ball))
            return;

        p3d ballPos(game.ball.x, game.ball.y, game.ball.z);
        double myDistance = distanceXZ(ballPos, p3d(me.x, me.y, me.z));

        if (game.ball.z < -rules.arena.depth / 2 + rules.arena.depth / 8 && fabs(game.ball.x) > rules.arena.goal_width / 2 * 1.2)
            return;

        for (const Robot& r: game.robots)
        {
            if (r.id == me.id && r.player_id == me.player_id)
                continue;

            if (myDistance > distanceXZ(ballPos, p3d(r.x, r.y, r.z)))
                return;
        }

        sprintTo(ballPos, me, rules, action);
    }
}

void MyStrategy::getInterceptionPoints(const Rules& rules, const Ball &ball, double secondsForward, std::vector<futurePoint>& points)
{
    double GRAVITY = rules.GRAVITY;

    p3d ballPos(ball.x, ball.y, ball.z);

    double ballVy = ball.velocity_y;

    bool rolling = isRolling(ball);

    int ticks = static_cast<int>(secondsForward * TICKS);

    bool newInterval = true; // участок монотонности
    for (int t = 0; t < ticks; ++t)
    {
        ballPos.x += ball.velocity_x / TICKS;
        ballPos.y += ballVy / TICKS - GRAVITY / TICKS / TICKS / 2;
        ballPos.z += ball.velocity_z / TICKS;

        if (ballVy * (ballVy - GRAVITY / TICKS) < 0.0)
        {
            // going down
            newInterval = true;
        }

        ballVy -= GRAVITY / TICKS;

        if (newInterval &&
                ((ballVy < 0.0 && ballPos.y < 2 * ball.radius) || (ballVy > 0.0 && ballPos.y < 2 * ball.radius)))
        {
            if (!rolling)
                points.push_back({p3d(ballPos.x, ballPos.y, ballPos.z), t});
            newInterval = false;
        }

        if (ballPos.y < ball.radius)
        {
            // going up
            ballVy = -ballVy * rules.BALL_ARENA_E;
            ballPos.y += (ball.radius - ballPos.y);
            newInterval = true;
        }
    }

    if (rolling)
    {
        int count = 10;
        points.push_back({p3d(ball.x, ball.y, ball.z), 0});
        for (int i = count - 1; i > 0; --i)
        {
            points.push_back({p3d(ballPos.x + (ball.x - ballPos.x) / count * i,
                                  ballPos.y + (ball.y - ballPos.y) / count * i,
                                  ballPos.z + (ball.z - ballPos.z) / count * i),
                              static_cast<int>(secondsForward * TICKS / count * (count - i))});
        }
    }

    points.push_back({p3d(ballPos.x, ballPos.y, ballPos.z), ticks});

    for (auto ip: points)
    {
        m_spheres.push_back(sphere(ip.first.x, ip.first.y, ip.first.z, ball.radius, p3d(0.0, 0.0, 1.0)));
    }
}

bool MyStrategy::ballGoesToGoal(const Rules& rules, const Ball &ball, std::vector<futurePoint>& interceptionPoints)
{
    interceptionPoints.clear();

    if (ball.velocity_z >= 0.0) // от ворот
        return false;

    if (ball.velocity_x < 0 && ball.x < -rules.arena.goal_width/2) // прокатился налево
        return false;

    if (ball.velocity_x > 0 && ball.x > rules.arena.goal_width/2) // прокатился направо
        return false;

    if (fabs(ball.x) >= rules.arena.width/2 - rules.arena.bottom_radius) // над радиусом - посчитаем позже
        return false;

    double secondsToGoalLine = (ball.z + rules.arena.depth/2) / -ball.velocity_z;

    if (secondsToGoalLine > 30.0)
        return false;

    getInterceptionPoints(rules, ball, secondsToGoalLine, interceptionPoints);

    bool result = fabs(interceptionPoints.back().first.x) <= rules.arena.goal_width/2 - ball.radius &&
            fabs(interceptionPoints.back().first.y) <= rules.arena.goal_height - ball.radius;

    return result;
}

double dot(const p3d& a, const p3d& b)
{
    std::vector<double> av = {a.x, a.y, a.z};
    std::vector<double> bv = {b.x, b.y, b.z};
    return std::inner_product(av.begin(), av.end(), bv.begin(), 0.0);
}

int MyStrategy::interceptionTime(futurePoint at, const Robot &robot, const Rules &rules)
{
    double myDistance = distanceXZ(p3d(robot.x, robot.y, robot.z), at.first);
    p3d normal(at.first.x - robot.x, 0.0, at.first.z - robot.z);
    double vAlong = dot(normal, p3d(robot.velocity_x, 0, robot.velocity_z));
    double distanceCovered = 0.0;
    int takesTicks = 0;
    while (distanceCovered < myDistance)
    {
        distanceCovered += vAlong * (1.0 / TICKS);
        vAlong += rules.ROBOT_ACCELERATION * (1.0 / TICKS);
        vAlong = std::min(vAlong, rules.ROBOT_MAX_GROUND_SPEED);
        ++takesTicks;
    }

    return takesTicks;
}

bool MyStrategy::canReachInTime(futurePoint at, const Robot &me, const Rules &rules, int& sprintTime, int& elevationTime)
{
    sprintTime = interceptionTime(at, me, rules);
    if (sprintTime > at.second)
        return false; // too far

    elevationTime = timeToElevate(at.first.y, me, rules);
    if (elevationTime == -1)
        return false; // too high

    if (elevationTime > at.second)
        return false; // jumping too slow

    return true;
}

p3d::p3d(double _x, double _y, double _z) : x(_x), y(_y), z(_z) {}
p3d::p3d() : x(0.0), y(0.0), z(0.0) {}
sphere::sphere(double _x, double _y, double _z, double _r, p3d _rgb) : x(_x), y(_y), z(_z), r(_r ), rgb(_rgb){}
sphere::sphere() : x(0.0), y(0.0), z(0.0), r(0.0), rgb() {}
