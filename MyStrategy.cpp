#include "MyStrategy.h"
#include <vector>
#include <iostream>
#include <fstream>
#include <math.h>
#include <string>
#include <numeric>

using namespace model;

MyStrategy::MyStrategy()
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

void MyStrategy::act(const Robot& me, const Rules& rules, const Game& game, Action& action)
{
    if (m_tick_spheres != game.current_tick)
    {
        m_spheres.clear();
        m_tick_spheres = game.current_tick;
    }

    if (me.id % 2 == 1)
    {
        C_bullyGoalie(me, rules, game, action);
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

//    if (!m_spheres.empty())
//        m_json.pop_back();

    m_json += addText(m_text);

    m_json += "]";

    return m_json;
}

bool isRolling(double vy, double y, double r)
{
    return fabs(vy) < 7.0 && y < 2 * r;
}

void MyStrategy::C_defend(const Robot &me, const Rules &rules, const Game &game, Action &action)
{
    p3d pos = getGoalieDefaultPosition(rules, { game.ball.x, game.ball.y, game.ball.z });
    sprintTo(pos, me, action);

    std::vector<std::pair<p3d, int>> interceptionPoints;
    std::cout << "tick = " << game.current_tick << std::endl;
    bool onTarget = ballGoesToGoal(rules, game.ball, interceptionPoints);
    if (onTarget)
    {
        // pick point to intercept
        for (auto iPoint: interceptionPoints)
        {
            if (canIntercept(iPoint, me, rules, game))
            {
                m_spheres.push_back(sphere(iPoint.first.x, iPoint.first.y, iPoint.first.z,
                                           game.ball.radius*1.5, p3d(1.0, 0.0, 0.0)));
                sprintTo(iPoint.first, me, action);
                break;
            }
        }

        if (!eq(game.ball.y, game.ball.radius) && // on the ground
                distanceXZ({ me.x, 0, me.z }, { game.ball.x, 0, game.ball.z }) < game.ball.radius * 2) // high time
        {
            action.jump_speed = rules.ROBOT_MAX_JUMP_SPEED;
        }
    }
    m_text = std::to_string(fabs(game.ball.velocity_y)) + (isRolling(game.ball.velocity_y, game.ball.y, game.ball.radius) ? "Rolling" : "");
    m_text = std::to_string(rules.TICKS_PER_SECOND);
}

void MyStrategy::getInterceptionPoints(const Rules& rules, const Ball &ball, double secondsForward, std::vector<std::pair<p3d, int>>& points)
{
    double GRAVITY = rules.GRAVITY;

    p3d ballPos(ball.x, ball.y, ball.z);

    double ballVy = ball.velocity_y;

    bool rolling = isRolling(ballVy, ball.y, ball.radius);

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
        int count = 5;
        points.push_back({p3d(ball.x, ball.y, ball.z), 0});
        for (int i = count; i > 0; --i)
        {
            points.push_back({p3d(ballPos.x + (ball.x - ballPos.x) / count * i,
                                  ballPos.y + (ball.y - ballPos.y) / count * i,
                                  ballPos.z + (ball.z - ballPos.z) / count * i),
                              static_cast<int>(secondsForward * TICKS / 5)});
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

    bool result = fabs(interceptionPoints.back().first.x) <= rules.arena.goal_width/2 /*- ball.radius*/ &&
            fabs(interceptionPoints.back().first.y) <= rules.arena.goal_height - ball.radius;

    return result;
}

double dot(const p3d& a, const p3d& b)
{
    std::vector<double> av = {a.x, a.y, a.z};
    std::vector<double> bv = {b.x, b.y, b.z};
    return std::inner_product(av.begin(), av.end(), bv.begin(), 0.0);
}

bool MyStrategy::canIntercept(std::pair<p3d, int> at, const Robot &me, const Rules &rules, const Game &game)
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
        if (game.current_tick == 4003)
            std::cout << distanceCovered << std::endl;
    }

    return takesTicks < at.second;
}

p3d::p3d(double _x, double _y, double _z) : x(_x), y(_y), z(_z) {}
p3d::p3d() : x(0.0), y(0.0), z(0.0) {}
sphere::sphere(double _x, double _y, double _z, double _r, p3d _rgb) : x(_x), y(_y), z(_z), r(_r ), rgb(_rgb){}
sphere::sphere() : x(0.0), y(0.0), z(0.0), r(0.0), rgb() {}
