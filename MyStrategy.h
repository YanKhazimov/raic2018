#if defined(_MSC_VER) && (_MSC_VER >= 1200)
#pragma once
#endif

#ifndef _MY_STRATEGY_H_
#define _MY_STRATEGY_H_

#include "Strategy.h"

class MyStrategy : public Strategy {
public:
    MyStrategy();

    void act(const model::Robot& me, const model::Rules& rules, const model::Game& world, model::Action& action) override;

private:
    std::vector<double> getGoalieDefaultPosition(const model::Rules& rules,
                                                 std::vector<double> ballPosition);
    std::vector<double> predictBallState(const model::Rules& rules, const model::Ball &ball,
                                         double secondsForward, int ticksForward);
    bool ballGoesToGoal(const model::Rules& rules, const model::Ball &ball,
                        std::vector<double>& where);
};

#endif
