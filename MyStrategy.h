#if defined(_MSC_VER) && (_MSC_VER >= 1200)
#pragma once
#endif

#ifndef _MY_STRATEGY_H_
#define _MY_STRATEGY_H_

#include "Strategy.h"

struct p3d {
    double x;
    double y;
    double z;

    p3d(double _x, double _y, double _z);
    p3d();
};

class MyStrategy : public Strategy {
public:
    MyStrategy();

    void act(const model::Robot& me, const model::Rules& rules, const model::Game& world, model::Action& action) override;

private:
    // COMMANDS
    void C_defend(const model::Robot& me, const model::Rules& rules, const model::Game& game, model::Action& action);
    void C_bullyGoalie(const model::Robot& me, const model::Rules& rules, const model::Game& game, model::Action& action);

    p3d getGoalieDefaultPosition(const model::Rules& rules,
                                 p3d ballPosition);
    std::vector<p3d> getInterceptionPoints(const model::Rules& rules, const model::Ball &ball,
                                         double secondsForward, int ticksForward);
    bool ballGoesToGoal(const model::Rules& rules, const model::Ball &ball,
                        std::vector<p3d> &interceptionPoints);
};

#endif
