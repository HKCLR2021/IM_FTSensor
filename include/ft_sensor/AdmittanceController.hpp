#pragma once
#include <Eigen/Dense>
#include <Eigen/Core>
#include <algorithm>
#include <iostream>

// Implementation of algo proposed in: 
// A dynamical system approach for detection and reaction to human
// guidance in physical human–robot interaction
// https://www.researchgate.net/publication/343008526_A_dynamical_system_approach_for_detection_and_reaction_to_human_guidance_in_physical_human-robot_interaction

class AdmittanceController
{
public:
    AdmittanceController();
    ~AdmittanceController() {};

    double computeAdmittanceRatio();
    Eigen::VectorXd computeAdmtOutput(Eigen::VectorXd F_ext); // actually wrench here


    double getAdmittanceRatio() { return h; }
    double getDt() { return dt; }

private:
    // Predefined params
    Eigen::MatrixXd D_a; // damping matrix for admittance control
    Eigen::MatrixXd M_a; // mass matrix for admittance control
    Eigen::MatrixXd M_a_inv; 
    double dt;

    double Pd_tilde; // power dissipation rate
    double E_thres;
    double E_max;

    // Working params
    double h;
    double E_m;
    
    Eigen::VectorXd dx_a;
    Eigen::VectorXd F_h;

};