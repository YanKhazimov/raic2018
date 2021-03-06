#ifndef _MY_STRATEGY_H_
#define _MY_STRATEGY_H_

#include "Strategy.h"
#include <map>

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
    bool operator==(const p3d& other);
    bool operator!=(const p3d& other);
    p3d& operator=(const p3d& other);
    p3d to2d();
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

    struct futurePoint
    {
        p3d pos;
        p3d v;
        int t;

        futurePoint();
        futurePoint(p3d _pos, p3d _v, int _t);
        bool operator==(const futurePoint& other);
        bool operator!=(const futurePoint& other);
    };

    void alignShot();
    bool hasRunUp = false;
    p3d getRunupPosition(const MyStrategy::futurePoint& shotPos, p3d shotTarget);
    futurePoint getMoveTarget();
    p3d getShotTarget();
    int timeToRun(p3d to, p3d pos, p3d v);
    int timeToChangeSpeed(p3d targetSpeed, p3d initialSpeed, double &dist);

    void act(const model::Robot& me, const model::Rules& rules, const model::Game& game, model::Action& action) override;

    std::string custom_rendering() override;


private:
    const Robot* me; const Rules* rules; const Game* game; Action* action;

    std::string m_json;
    std::vector<sphere> m_spheres;
    int m_tick_spheres = -1;
    std::string m_text;
    const int criticalPaceDiff = 2;
    int m_clearerId = -1;

    struct PlannedShot {
        p3d ball;
        p3d me;
        int tick;
        int elevationTime;
        PlannedShot();
        PlannedShot(p3d b, p3d m, int t, int et);
        bool isValid();
        void invalidate();
    };

    struct InterceptionStep {
        p3d curV, targetV, curPos;
        bool jump;
        std::string stage;
    };

    struct InterceptionPlan {
        std::map<int, InterceptionStep> steps;

        p3d stopPos, runupPos, jumpPos;
        p3d ball;
        p3d me;

        int pace;
        int tick;
        int elevationTime;

        InterceptionPlan();
        InterceptionPlan(p3d _stopPos, p3d _runupPos, p3d _jumpPos, p3d _ball, p3d _me, int _pace, int _tick, int _elevationTime);
        bool isValid();
        void invalidate();
    };

    PlannedShot m_plannedAttackerTarget;
    InterceptionPlan m_goaliePlan;

    p3d deltaPos(p3d fromV, p3d toV, int mt);

    enum Role {
        Unassigned = 0,
        Goalie,
        Attacker,
        Bully
    };

    std::map<int, Role> m_roles; // id -> role

    void getRoles();
    std::pair<int, int> getTeammate();
    double maxElevation();
    bool isRolling(const p3d& ballPos);

    // COMMANDS
    void C_defend();
    void C_attack();
    void getInterceptionPoints(const model::Ball &ball,
                               double secondsForward, std::vector<futurePoint>& points);
    bool ballGoesToGoal(const model::Ball &ball,
                        std::vector<futurePoint > &interceptionPoints);
    std::pair<int, int> pickInterceptionPoint(const std::vector<futurePoint>& interceptionPoints);
    bool setInterceptionPoint(bool &goalLine);
    int sprintTime(p3d at, const Robot *robot);
    int interceptionTime(p3d at, const Robot *robot, int elevationTime);
    bool makeInterceptionPlan(p3d at, int targetTick, bool must);
    int canReachInTime(futurePoint at, int &sprintT, int &elevationTime);
    bool interceptBounceAt(const futurePoint& point);
    std::pair<int, int> measureShot(futurePoint point);
    p3d getGoalieDefaultPosition(const model::Ball &ball);
    bool isConsistent(const InterceptionPlan &m_goaliePlan);
    bool fasterOpponent(p3d ballpos, int t);
    void executePlan();

    void C_bullyGoalie();
    void C_bullyAttacker();
    void C_bullyClosestToBall();

    void getBehindNextLanding();

    bool pickShootingPoint(int ticks, futurePoint& bestTarget, futurePoint &bestBall, int &shootingPace, int &elevationTime);
    p3d getBestGoalTarget(p3d ballpos);
    void intercept(const std::vector<futurePoint>& interceptionPoints, bool homeOnly);

    double brakeDistance(double initialSpeed);
    futurePoint hitPoint(const MyStrategy::futurePoint& center);
    p3d alignHitTo(p3d target, p3d ball, bool under45 = true);
    void setSpeed(double value, p3d normal);
    void runTo(p3d to);
    void sprintTo(p3d to, bool jump);
    void simulateRoll(p3d& ballpos, p3d& ballv, const p3d& normal);
    void simulateBounce(p3d& ballPos, p3d& ballv);
    void simulateTick(p3d& ballpos, p3d& ballv);
    std::pair<bool, int> timeToElevate(double height);
    //int timeToElevate(double height);
    bool inGoalSector(p3d ballPos);

    void addSphere(sphere s);
    std::string logSphere(double x, double y, double z, double r, p3d rgb);
    std::string logText(std::string text);
};

#endif
