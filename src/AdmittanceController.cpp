#include "ft_sensor/AdmittanceController.hpp"
#define debug if(1) std::cout

AdmittanceController::AdmittanceController()
{

    Eigen::VectorXd D_a_diag(6);
    D_a_diag << 20, 20, 20, 10, 10, 10;
    D_a = D_a_diag.asDiagonal();

    Eigen::VectorXd M_a_diag(6);
    M_a_diag << 15, 15, 15, 7.5, 7.5, 7.5;
    M_a = M_a_diag.asDiagonal();
    M_a_inv = M_a.inverse();

    dt = 1.0 / 100;
    Pd_tilde = 0.25; 
    E_thres = 10;
    E_max = 50;

    dx_a = Eigen::VectorXd::Zero(6);
    F_h = Eigen::VectorXd::Zero(6);

    debug << "\n||D_a||: \n" << D_a << std::endl;
    debug << "\n||M_a||: \n" << M_a << std::endl;

}

// h as in Eq8 of paper. range of h: 0-1
// h=0 -> robot free to move
// h=1 -> persistent external force exist (eg. obstacle hit / human trying to hold and guide the robot)
double AdmittanceController::computeAdmittanceRatio()
{
    if (E_m > E_thres)
        h = (E_m - E_thres) / (E_max - E_thres);
    else 
        h = 0; // dead zone
    return h;
}

// dx_a as in Eq1 of paper
// i.e. the estimated intent of the detected external force, expressed as velocity desired
// for use in a robot (real space) velocity controller
// expect to be called every dt (i.e. dafault 10ms)
Eigen::VectorXd AdmittanceController::computeAdmtOutput(Eigen::VectorXd F_ext)
{
    // Control Loop

    double force_thres_N = 3.0;
    double torque_thres_Nm = 0.05;

    for (int i=0;i<3;i++){
        if (abs(F_ext[i]) < force_thres_N) F_ext[i]=0.0;
    }

    for (int i=3;i<6;i++){
        if (abs(F_ext[i]) < torque_thres_Nm) F_ext[i]=0.0;
    }

    Eigen::VectorXd ddx_a = M_a_inv * (-D_a * dx_a + F_ext);

    double Pi_tilde = dx_a.dot(F_ext);
    double Po_tilde = dx_a.dot(F_h);
    double Pt_tilde = (1 - h) * Pd_tilde;

    double dE = Pi_tilde - Po_tilde - Pt_tilde;
    E_m += dE;

    // Saturated in [0, Em]
    E_m = std::max(E_m, 0.);
    E_m = std::min(E_m, E_max);

    h = computeAdmittanceRatio();
    F_h = h * F_ext; 

    dx_a = dx_a + ddx_a * dt;
    return dx_a;
}

