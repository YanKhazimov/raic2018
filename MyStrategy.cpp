#include "MyStrategy.h"
#include <vector>
#include <iostream>
#include <fstream>
#include <math.h>

using namespace model;

MyStrategy::MyStrategy() { }

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

void runFast(p3d to, const Robot& me, Action& action)
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

    runFast(goaliePos, me, action);
}

void MyStrategy::act(const Robot& me, const Rules& rules, const Game& game, Action& action)
{
    if (me.id % 2 == 1)
    {
        C_bullyGoalie(me, rules, game, action);
        return;
    }

    C_defend(me, rules, game, action);
}

void MyStrategy::C_defend(const Robot &me, const Rules &rules, const Game &game, Action &action)
{
    p3d pos = getGoalieDefaultPosition(rules, { game.ball.x, game.ball.y, game.ball.z });

    runFast(pos, me, action);

    std::vector<p3d> interceptionPoints;
    if (ballGoesToGoal(rules, game.ball, interceptionPoints)) // мяч попадает в ворота
    {
        // pick point to intercept

        if (!eq(game.ball.y, game.ball.radius) && // не катится
                distanceXZ({ me.x, 0, me.z }, { game.ball.x, 0, game.ball.z }) < game.ball.radius * 2) // пора
        {
            action.jump_speed = 15.0; // rules.ROBOT_MAX_JUMP_SPEED;
        }
    }
}

std::vector<p3d> MyStrategy::getInterceptionPoints(const Rules& rules, const Ball &ball, double secondsForward, int ticksForward)
{
    double GRAVITY = 30; // rules.GRAVITY
    std::vector<p3d> result;

    p3d ballPos(ball.x, ball.y, ball.z);

    double ballVy = ball.velocity_y;

    bool rolling = false;
    if (fabs(ballVy) < 0.1 && ball.y < 2 * ball.radius) // rolling
    {
        rolling = true;
    }

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
            result.push_back(p3d(ballPos.x, ballPos.y, ballPos.z));
            newInterval = false;
        }

        if (ballPos.y < ball.radius)
        {
            // going up
            ballVy = -ballVy;
            ballPos.y += (ball.radius - ballPos.y);
            newInterval = true;
        }
    }

    result.push_back(p3d(ballPos.x, ballPos.y, ballPos.z));

    return result;
}

bool MyStrategy::ballGoesToGoal(const Rules& rules, const Ball &ball, std::vector<p3d>& interceptionPoints)
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

    double secondsToGoalLineD = (ball.z + rules.arena.depth/2) / -ball.velocity_z;
    int secodnsToGoalLine = static_cast<int>(secondsToGoalLineD);
    int ticksToGoalLine = static_cast<int>((secondsToGoalLineD - secodnsToGoalLine) * 60);

    interceptionPoints = getInterceptionPoints(rules, ball, secondsToGoalLineD, ticksToGoalLine);

    bool result = fabs(interceptionPoints.back().x) <= rules.arena.goal_width/2 /*- ball.radius*/ &&
            fabs(interceptionPoints.back().y) <= rules.arena.goal_height - ball.radius;

    std::ofstream ofs("log.txt", std::ios_base::app);
    for (auto p: interceptionPoints)
        ofs << p.x << '\t' << p.y << '\t' << p.z << std::endl;
    ofs << std::endl;
    ofs.close();

    return result;
}

p3d::p3d(double _x, double _y, double _z)
    : x(_x), y(_y), z(_z)
{
}

p3d::p3d()
    : x(0.0), y(0.0), z(0.0)
{
}
