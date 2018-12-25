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

void sprintTo(p3d to, const Robot& me, Action& action)
{
    action.target_velocity_x = 30 * (to.x - me.x);
    action.target_velocity_y = 30 * (to.y - me.y);
    action.target_velocity_z = 30 * (to.z - me.z);
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

    sprintTo(goaliePos, me, action);
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

    sprintTo(goaliePos, me, action);
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

void getBehindBall(const Robot& me, const Game& game, Action& action)
{
    p3d destination(game.ball.x, game.ball.y, game.ball.z - game.ball.radius);

    if (me.x < game.ball.x)
        destination.x -= game.ball.radius + me.radius;
    else
        destination.x += game.ball.radius + me.radius;

    sprintTo(destination, me, action);
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
        //C_bullyGoalie(me, rules, game, action);
        if (me.z + me.radius > game.ball.z)
            getBehindBall(me, game, action);
        else
        {
            p3d ballPos(game.ball.x, game.ball.y, game.ball.z);
            sprintTo(ballPos, me, action);
            if (distanceXZ(ballPos, p3d(me.x, me.y, me.z)) < game.ball.radius + 2 * me.radius &&
                    game.ball.y < 1.5 * game.ball.radius)
                action.jump_speed = rules.ROBOT_MAX_JUMP_SPEED;
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

bool isRolling(const model::Ball& ball)
{
    return fabs(ball.velocity_y) < 7.0 && ball.y < 2 * ball.radius;
}

bool timeToJump(const Robot &me, const std::pair<p3d, int>& iPoint, const Rules &rules)
{
    int ticksToGoUp = 0;
    double myY = me.y;
    double myVY = me.velocity_y;
    while (myY < iPoint.first.y)
    {
        myY += myVY / TICKS;
        myVY = std::min(myVY + rules.ROBOT_ACCELERATION / TICKS, rules.ROBOT_MAX_JUMP_SPEED) - rules.GRAVITY / TICKS;
        if (myVY < 0.0)
        {
            return false;
        }
        ++ticksToGoUp;
    }

    return ticksToGoUp >= iPoint.second;
}

p3d hitPoint(const p3d& iPoint, const Rules &rules)
{
    return p3d(iPoint.x, iPoint.y - rules.BALL_RADIUS/2, iPoint.z - rules.BALL_RADIUS/2);
}

std::pair<p3d, int> hitPoint(const std::pair<p3d, int>& iPoint, const Rules &rules)
{
    return { hitPoint(iPoint.first, rules), iPoint.second };
}

std::pair<int, int> MyStrategy::pickInterceptionPoint(const std::vector<std::pair<p3d, int>>& interceptionPoints,
                                          const Robot &me, const Rules &rules, const Game &game)
{
    int iTime = 0;
    for (int i = 0; i < static_cast<int>(interceptionPoints.size()); ++i)
    {
        std::pair<p3d, int> iPoint = interceptionPoints[i];
        iTime = interceptionTime(iPoint, me, rules, game);
        if (iTime <= iPoint.second)
            return { i, iTime };
    }

    return { static_cast<int>(interceptionPoints.size() - 1), iTime };
}

void MyStrategy::C_defend(const Robot &me, const Rules &rules, const Game &game, Action &action)
{
    p3d pos = getGoalieDefaultPosition(rules, { game.ball.x, game.ball.y, game.ball.z });
    sprintTo(pos, me, action);

    std::vector<std::pair<p3d, int>> interceptionPoints;
    bool onTarget = ballGoesToGoal(rules, game.ball, interceptionPoints);
    if (onTarget)
    {
        int interceptionTime = 0, iPointIndex = -1;
        std::tie(iPointIndex, interceptionTime) = pickInterceptionPoint(interceptionPoints, me, rules, game);

        std::pair<p3d, int> iPoint = interceptionPoints[static_cast<unsigned int>(iPointIndex)];

        m_text = (interceptionTime <= iPoint.second) ? "can intercept" : "CAN'T INTERCEPT";

        m_spheres.push_back(sphere(iPoint.first.x, iPoint.first.y, iPoint.first.z,
                                   game.ball.radius*1.2, p3d(1.0, 0.0, 0.0)));

        if (isRolling(game.ball))
        {
            if (iPointIndex == 1)
                iPoint = interceptionPoints[0];
            sprintTo(iPoint.first, me, action);

            double distance = distanceXZ(iPoint.first, p3d(me.x, me.y, me.z));
            if (distance < 2 * me.radius + game.ball.radius)
                action.jump_speed = rules.ROBOT_MAX_JUMP_SPEED;
        }
        else
        {
            // ball is bouncing
            std::pair<p3d, int> pointToHit = hitPoint(iPoint, rules);

            if (iPoint.second == interceptionPoints.back().second)
            {
                // goal line interception logic
                //if (interceptionTime + 1 == iPoint.second || interceptionTime == iPoint.second)
                    sprintTo(pointToHit.first, me, action);

                if (timeToJump(me, pointToHit, rules))
                {
                    action.jump_speed = rules.ROBOT_MAX_JUMP_SPEED;
                }
            }
            else
            {
                // field interception logic
                sprintTo(pointToHit.first, me, action);

                if (timeToJump(me, pointToHit, rules))
                {
                    action.jump_speed = rules.ROBOT_MAX_JUMP_SPEED;
                }
            }
        }
    }

}

void MyStrategy::getInterceptionPoints(const Rules& rules, const Ball &ball, double secondsForward, std::vector<std::pair<p3d, int>>& points)
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

bool MyStrategy::ballGoesToGoal(const Rules& rules, const Ball &ball, std::vector<std::pair<p3d, int>>& interceptionPoints)
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

int MyStrategy::interceptionTime(std::pair<p3d, int> at, const Robot &me, const Rules &rules, const Game &game)
{
    double myDistance = distanceXZ(p3d(me.x, me.y, me.z), at.first);
    p3d normal(at.first.x - me.x, 0.0, at.first.z - me.z);
    double vAlong = dot(normal, p3d(me.velocity_x, 0, me.velocity_z));
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

p3d::p3d(double _x, double _y, double _z) : x(_x), y(_y), z(_z) {}
p3d::p3d() : x(0.0), y(0.0), z(0.0) {}
sphere::sphere(double _x, double _y, double _z, double _r, p3d _rgb) : x(_x), y(_y), z(_z), r(_r ), rgb(_rgb){}
sphere::sphere() : x(0.0), y(0.0), z(0.0), r(0.0), rgb() {}
