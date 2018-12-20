#include "MyStrategy.h"
#include <vector>
#include <iostream>
#include <math.h>

using namespace model;

MyStrategy::MyStrategy() { }

double distance(std::vector<double> a, std::vector<double> b)
{
    return sqrt((a[0] - b[0])*(a[0] - b[0]) + (a[2] - b[2])*(a[2] - b[2]));
}

std::vector<double> getGoalieDefaultPosition(const Rules& rules, std::vector<double> ballPosition)
{
    const Arena& arena = rules.arena;

    // by bissectrisa
    double d1 = distance(ballPosition, {-arena.goal_width/2, 0, -arena.depth/2});
    double d2 = distance(ballPosition, {arena.goal_width/2, 0, -arena.depth/2});

    double x = -arena.goal_width/2 + arena.goal_width * d1 / (d1 + d2);

    std::vector<double> result {x, 0.0, -arena.depth/2};


    return result;
}

void MyStrategy::act(const Robot& me, const Rules& rules, const Game& game, Action& action)
{
    std::vector<double> pos = getGoalieDefaultPosition(rules, { game.ball.x, game.ball.y, game.ball.z });

    if (me.id % 2 == 1)
    {
        pos[2] += game.ball.radius * 4;
        pos[0] = -pos[0];
    }

    action.target_velocity_x = 30 * (pos[0] - me.x);
    action.target_velocity_y = 30 * (pos[1] - me.y);
    action.target_velocity_z = 30 * (pos[2] - me.z);

    if (distance({ me.x, 0, me.z }, { game.ball.x, 0, game.ball.z }) < game.ball.radius * 4)
        action.jump_speed = 15.0;
}
