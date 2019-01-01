#if defined(_MSC_VER) && (_MSC_VER >= 1200)
#pragma once
#endif

#ifndef _MY_STRATEGY_H_
#define _MY_STRATEGY_H_

#include "Strategy.h"

//#define debugging_spheres_yan true

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

    struct futurePoint
    {
        p3d pos;
        p3d v;
        int t;

        futurePoint();
        futurePoint(p3d _pos, p3d _v, int _t);
    };

private:
    const Robot* me; const Rules* rules; const Game* game; Action* action;

    std::string m_json;
    std::vector<sphere> m_spheres;
    int m_tick_spheres = -1;
    std::string m_text;
    const int criticalPaceDiff = 6;

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
    std::pair<int, int> measureShot(futurePoint point);
    p3d getGoalieDefaultPosition(const model::Ball &ball);

    void C_bullyGoalie();
    void C_bullyAttacker();

    void getBehindBall();

    bool pickShootingPoint(int ticks, futurePoint& bestTarget, int &shootingPace, int &elevationTime);
    void intercept(const std::vector<futurePoint>& interceptionPoints, bool homeOnly);

    double brakeDistance(double initialSpeed);
    futurePoint hitPoint(const MyStrategy::futurePoint& center);
    void setSpeed(double value, p3d normal);
    void runTo(p3d to);
    void sprintTo(p3d to, bool jump);
    void simulateRoll(p3d& ballpos, p3d& ballv, const p3d& normal);
    void simulateBounce(p3d& ballPos, p3d& ballv);
    int timeToElevate(double height);
    bool inGoalSector(p3d ballPos, int &xshift);

    void addSphere(sphere s);
    std::string logSphere(double x, double y, double z, double r, p3d rgb);
    std::string logText(std::string text);
};

#endif
