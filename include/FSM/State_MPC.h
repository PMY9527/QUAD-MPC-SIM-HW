#ifndef MPC_H
#define MPC_H

#include "FSM/FSMState.h"
#include "Gait/GaitGenerator.h"
#include "control/BalanceCtrl.h"
#include "thirdParty/quadProgpp/QuadProg++.hh"
#include "thirdParty/quadProgpp/Array.hh"
#include <chrono>
#include <vector>
#include <iostream>
#include <iomanip>
#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>

// ************************** Important parameters ************************** // 
static const int mpc_N = 8;  // Prediction horizon
static const int nx = 13;    // Dimension of state vector
static const int nu = 12;    // Dimension of control input
static const double speed_limx = 0.5; // Speed limit in x direction
static const double NEGATIVE_NUMBER = -1000000.0;
static const double POSITIVE_NUMBER = 1000000.0;
static const double fz_max = 180.0; // Max force in z axis
static const double g = 9.8;         // Gravity
static const double miu = 0.3;       // Friction coef.
static const double d_time = 0.01;  // Discretization time step, while the solver solves at 2.5-3.7 ms for horizon of 10, a larger time step is good for margins. 

class State_MPC : public FSMState
{
public:
    State_MPC(CtrlComponents *ctrlComp);
    ~State_MPC();
    void enter();
    void run();
    void exit();
    virtual FSMStateName checkChange();
    void setHighCmd(double vx, double vy, double wz);

private:
    // ************************** Important parameters ************************** // 
    void setWeight();
    virtual void getUserCmd();
    void calcCmd();

    void calcTau(); 
    void calcQQd();

    void SetMatrices();
    void ConstraintsSetup();
    void solveQP();
    void calcFe();

    GaitGenerator *_gait;
    Estimator *_est;
    QuadrupedRobot *_robModel;
    BalanceCtrl *_balCtrl;

    // Rob State
    Vec3 _posBody, _velBody, _rpy;
    double _yaw, _dYaw;
    Vec34 _posFeetGlobal, _velFeetGlobal;
    Vec34 _posFeet2BGlobal;
    RotMat _B2G_RotMat, _G2B_RotMat;
    Vec12 _q;
    
    // Robot command
    Vec3 _pcd;
    Vec3 _vCmdGlobal, _vCmdBody;
    double _yawCmd, _dYawCmd;
    double _dYawCmdPast;
    Vec3 _wCmdGlobal,current_euler;
    Vec34 _posFeetGlobalGoal, _velFeetGlobalGoal;
    Vec34 _posFeet2BGoal, _velFeet2BGoal;
    RotMat _Rd;
    Vec34 _forceFeetGlobal, _forceFeetBody;
    Vec34 _qGoal, _qdGoal;
    Vec12 _tau;
    // Control Parameters
    double _gaitHeight;
    Vec3 _posError, _velError;
    Mat3 _Kpp, _Kdp, _Kdw;
    double _kpw;
    Mat3 _KpSwing, _KdSwing;
    Vec2 _vxLim, _vyLim, _wyawLim;
    Vec4 *_phase;
    VecInt4 *_contact;
    double dt_actual;
    
    // MPC用到的变量
    double _mass;       
    Eigen::Matrix<double, 5, 3> miuMat;
    Eigen::Matrix<double, 3, 3> Ic;
    Eigen::Matrix<double, nx, 1> currentStates;
    Eigen::Matrix<double, nx * mpc_N, 1> Xd;
    Eigen::Matrix<double, 3, 3> R_curz;
    Eigen::Matrix<double, nx, nx> Ac;
    Eigen::Matrix<double, nx, nu> Bc;
    Eigen::Matrix<double, nx, nx> Ad;
    Eigen::Matrix<double, nx, nu> Bd;
    Eigen::Matrix<double, nx * mpc_N, nx> Aqp;
    Eigen::Matrix<double, nx * mpc_N, nu> Bd_list;
    Eigen::MatrixXd Bqp, x_vec, prediction_X;
    Eigen::MatrixXd dense_hessian;
    Eigen::Matrix<double, nu * mpc_N, 1> gradient; // q
    Eigen::MatrixXd Q_diag,R_diag, Q_diag_N, R_diag_N, Q, R, CI_, CE_;
    Eigen::VectorXd ci0_, ce0_;
    Vec12 F_, Fprev;
    quadprogpp::Matrix<double> G, CE, CI;
    quadprogpp::Vector<double> g0, ce0, ci0, x;

    Eigen::Matrix<double, 3, 3> CrossProduct_A(Eigen::Matrix<double, 3, 1> A);
    Eigen::Matrix<double, 3, 3> Rz3(double theta);
};

#endif // MPC_H