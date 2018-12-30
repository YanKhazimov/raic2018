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
    p3d operator- (const p3d& other) const;
    p3d operator+ (const p3d& other) const;
    p3d operator* (const double& mult) const;
};

struct sphere {
    double x;
    double y;
    double z;
    double r;
    p3d rgb;

    sphere(p3d pos, double _r, p3d _rgb);
    sphere(double _x, double _y, double _z, double _r, p3d _rgb);
    sphere();
};

using namespace model;
class MyStrategy : public Strategy {
public:
    MyStrategy();

    void act(const model::Robot& me, const model::Rules& rules, const model::Game& game, model::Action& action) override;

    std::string custom_rendering() override;

    typedef std::pair<p3d, int> futurePoint;

private:
    const Robot* me; const Rules* rules; const Game* game; Action* action;

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

    void getRole();

    // COMMANDS
    void C_defend();
    void getInterceptionPoints(const model::Ball &ball,
                               double secondsForward, std::vector<futurePoint>& points);
    bool ballGoesToGoal(const model::Ball &ball,
                        std::vector<futurePoint > &interceptionPoints);
    std::pair<int, int> pickInterceptionPoint(const std::vector<futurePoint>& interceptionPoints);
    int interceptionTime(futurePoint at, const Robot *robot);
    bool canReachInTime(futurePoint at, int &sprintTime, int &elevationTime);
    bool interceptBounceAt(const futurePoint& point);
    bool shoot(futurePoint point);
    p3d getGoalieDefaultPosition(p3d ballPosition);

    void C_bullyGoalie();
    void C_bullyAttacker();

    void getBehindBall();
    bool simulate(int ticks, futurePoint &shootAt);

    double brakeDistance(double initialSpeed);
    p3d hitPoint(const p3d& iPoint);
    MyStrategy::futurePoint hitPoint(const MyStrategy::futurePoint& iPoint);
    void setSpeed(double value, p3d normal);
    void runTo(p3d to);
    void sprintTo(p3d to);
    void simulateRoll(p3d& ballpos, p3d& ballv, const p3d& normal);
    void simulateBounce(p3d& ballPos, p3d& ballv);
    int timeToElevate(double height);
    bool isScorable(p3d ballPos);



    std::string addSphere(double x, double y, double z, double r, p3d rgb);
    std::string addText(std::string text);
};

#endif
