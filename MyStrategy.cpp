#include "MyStrategy.h"

using namespace model;

MyStrategy::MyStrategy() { }

void MyStrategy::act(const Robot& me, const Rules& rules, const Game& game, Action& action)
{
    action.target_velocity_x = 5;
    action.target_velocity_y = 4;

}
