#include "MyStrategy.h"
#include <vector>
#include <iostream>
#include <math.h>

using namespace model;

MyStrategy::MyStrategy() { }

const int TICKS = 60;

double distance(std::vector<double> a, std::vector<double> b)
{
    return sqrt((a[0] - b[0])*(a[0] - b[0]) + (a[2] - b[2])*(a[2] - b[2]));
}

std::vector<double> MyStrategy::getGoalieDefaultPosition(const Rules& rules, std::vector<double> ballPosition)
{
    const Arena& arena = rules.arena;

    // by bissectrisa
    double d1 = distance(ballPosition, {-arena.goal_width/2, 0, -arena.depth/2});
    double d2 = distance(ballPosition, {arena.goal_width/2, 0, -arena.depth/2});

    double x = -arena.goal_width/2 + arena.goal_width * d1 / (d1 + d2);

    std::vector<double> result {x, 0.0, -arena.depth/2};

    return result;
}

void runFast(std::vector<double> to, const Robot& me, Action& action)
{
    action.target_velocity_x = 30 * (to[0] - me.x);
    action.target_velocity_y = 30 * (to[1] - me.y);
    action.target_velocity_z = 30 * (to[2] - me.z);
}

void bullyGoalie(const Robot& me, const Rules& rules, const Game& game, Action& action)
{
    std::vector<double> goaliePos { 0, 0, -rules.arena.depth };

    for (Robot r: game.robots)
    {
        if (r.player_id == me.player_id)
            continue;

        if (r.z > goaliePos[2])
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
        bullyGoalie(me, rules, game, action);
        return;
    }

    std::vector<double> pos = getGoalieDefaultPosition(rules, { game.ball.x, game.ball.y, game.ball.z });


    runFast(pos, me, action);

    std::vector<double> foo;
    if (ballGoesToGoal(rules, game.ball, foo) && // мяч попадает в ворота &&
            //  && не могу достать
            // пора

            distance({ me.x, 0, me.z }, { game.ball.x, 0, game.ball.z }) < game.ball.radius * 2)
        action.jump_speed = 15.0;//rules.ROBOT_MAX_JUMP_SPEED;

    ballGoesToGoal(rules, game.ball, foo);
}

std::vector<double> MyStrategy::predictBallState(const Rules& rules, const Ball &ball, double secondsForward, int ticksForward)
{
    std::vector<double> result(3);

    result[0] = ball.x + secondsForward * ball.velocity_x;

    result[1] = ball.y;

    double ballVy = ball.velocity_y;

    int ticks = static_cast<int>(secondsForward * TICKS);
    for (int t = 0; t < ticks; ++t)
    {
        result[1] += ballVy / TICKS - rules.GRAVITY / TICKS / TICKS / 2;
        ballVy -= rules.GRAVITY / TICKS;

        if (result[1] < ball.radius)
        {
            ballVy = -ballVy;
            result[1] += (ball.radius - result[1]);
        }
    }

    return result;
}

bool MyStrategy::ballGoesToGoal(const Rules& rules, const Ball &ball, std::vector<double>& where)
{
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

    where = predictBallState(rules, ball, secondsToGoalLineD, ticksToGoalLine);
    where[2] = -rules.arena.depth/2;
    //std::cout << where[0] << '\t' << where[1] << std::endl;

    bool result = fabs(where[0]) <= rules.arena.goal_width/2 /*- ball.radius*/ &&
            fabs(where[1]) <= rules.arena.goal_height - ball.radius;

    //std::cout << result << std::endl;

    return result;
}
