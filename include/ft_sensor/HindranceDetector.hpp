#pragma once

#include <thread>
#include <vector>
#include <mutex>
#include <memory>

#include <Eigen/Dense>
#include <Eigen/Core>

// Implementation of algo proposed in:
// A dynamical system approach for detection and reaction to human
// guidance in physical human–robot interaction
// https://www.researchgate.net/publication/343008526_A_dynamical_system_approach_for_detection_and_reaction_to_human_guidance_in_physical_human-robot_interaction

// Code credit : jcyang

class OnRobotForceTorqueSensor;

class IHindranceObserver {
    public:
      virtual bool onHindranceDetected(){return true;}
      virtual bool onHindranceCleared() {return true;}
};


class HindranceDetector
{

public:

    // force or torque below min_activate threshold are ignored
    // those that pass are then multiplied by sensitivity
    HindranceDetector(
        std::shared_ptr<OnRobotForceTorqueSensor> dataSrc,         
        int refreshHz=100, double sensitivity=1.0,
        double min_activate_force_N = 3.0,
        double min_activate_torque_Nm = 0.05
    );
    ~HindranceDetector();

    bool startMonitor(bool verbose=false);
    bool stopMonitor();

    //  0.0 = never detects
    //  1.0 = default
    // >1.0 = more sensitive then default
    void setSensitivity(double s){std::lock_guard<std::mutex> lock(update_mutex_); sensitivity = s;};

    // simplify intepretation of Admittance value to has / has no Hindrance for simple usecase
    bool hasHinderance() {return hasHinderance_;}

    // h as in Eq8 of paper. range of h: 0-1
    // h=0 -> nothing oppose robot movement
    // h=1 -> persistent external force exist (eg. obstacle hit / human trying to hold and guide the robot)
    double getAdmittanceRatio() {std::lock_guard<std::mutex> lock(update_mutex_); return h;}

    // dx_a as in Eq1 of paper
    // i.e. the estimated intent of the detected external force, expressed as velocity desired
    // for use in a robot (real space) velocity controller
    Eigen::VectorXd getAdmittanceOutput() {std::lock_guard<std::mutex> lock(update_mutex_); return dx_a;}

    void addObserver(IHindranceObserver* observer);
    void delObserver(IHindranceObserver* observer);

private:    

    double          computeAdmittanceRatio();
    Eigen::VectorXd computeAdmittanceOutput(Eigen::VectorXd F_ext);

    std::shared_ptr<OnRobotForceTorqueSensor> dataSrc;
    int refreshHz;
    double sensitivity = 1.0;
    double min_activate_force_N = 3.0;
    double min_activate_torque_Nm = 0.01;

    bool hasHinderance_ = false;

    // AdmitanceControl Predefined params
    Eigen::MatrixXd D_a; // damping matrix for admittance control
    Eigen::MatrixXd M_a; // mass matrix for admittance control
    Eigen::MatrixXd M_a_inv;
    double dt;

    double Pd_tilde; // power dissipation rate
    double E_thres;
    double E_max;

    // AdmitanceControl Working params
    double h;
    double E_m;

    Eigen::VectorXd dx_a;
    Eigen::VectorXd F_h;

    bool verbose_ = false;
    
    // thread control related
    bool isRunning_ = false;
    bool pendingStop_ = false;
    std::vector<IHindranceObserver*> hookedObservers_;
    std::mutex hookedObservers_mutex_;
    std::mutex update_mutex_;
    std::thread loopThread_;
};


