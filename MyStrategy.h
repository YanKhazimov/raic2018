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

struct sphere {
    double x;
    double y;
    double z;
    double r;
    p3d rgb;

    sphere(double _x, double _y, double _z, double _r, p3d _rgb);
    sphere();
};

using namespace model;

class MyStrategy : public Strategy {
public:
    MyStrategy();

    void act(const model::Robot& me, const model::Rules& rules, const model::Game& world, model::Action& action) override;

    std::string custom_rendering() override;

    typedef std::pair<p3d, int> futurePoint;


private:
    std::string m_json;
    std::vector<sphere> m_spheres;
    int m_tick_spheres = -1;
    std::string m_text;

    enum Role {
        Unassigned = 0,
        Goalie,
        Attacker
    };

    Role m_role;

    void getRole(const model::Robot& me, const model::Game& game);

    // COMMANDS
    void C_defend(const model::Robot& me, const model::Rules& rules, const model::Game& game, model::Action& action);
    void getInterceptionPoints(const model::Rules& rules, const model::Ball &ball,
                               double secondsForward, std::vector<futurePoint>& points);
    bool ballGoesToGoal(const model::Rules& rules, const model::Ball &ball,
                        std::vector<futurePoint > &interceptionPoints);
    std::pair<int, int> pickInterceptionPoint(const std::vector<futurePoint>& interceptionPoints,
                                              const model::Robot &me, const model::Rules &rules);
    int interceptionTime(futurePoint at, const model::Robot& robot, const model::Rules& rules);
    bool canReachInTime(futurePoint at, const model::Robot& me, const model::Rules& rules, int &sprintTime, int &elevationTime);
    bool interceptBounceAt(const futurePoint& point, const model::Robot &me, const model::Rules &rules, model::Action &action);
    p3d getGoalieDefaultPosition(const model::Rules& rules,
                                 p3d ballPosition);

    void C_bullyGoalie(const model::Robot& me, const model::Rules& rules, const model::Game& game, model::Action& action);
    void C_bullyAttacker(const model::Robot& me, const model::Rules& rules, const model::Game& game, model::Action& action);

    void getBehindBall(const model::Robot& me, const model::Rules& rules, const model::Game& game, model::Action& action);
    void shoot(const Robot& me, const Rules& rules, const Game& game, Action& action);

    std::string addSphere(double x, double y, double z, double r, p3d rgb);
    std::string addText(std::string text);
};

#endif
