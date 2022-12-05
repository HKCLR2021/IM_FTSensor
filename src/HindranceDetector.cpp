#include <iostream>

#include "ft_sensor/HindranceDetector.hpp"
#include "ft_sensor/OnRobotForceTorqueSensor.hpp"

#define debug if(1) std::cout

HindranceDetector::HindranceDetector(
    std::shared_ptr<OnRobotForceTorqueSensor> dataSrc,    
    int refreshHz, double sensitivity
    )
{
    this->dataSrc = dataSrc;    
    this->refreshHz = refreshHz;
    this->sensitivity = sensitivity;

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

    dx_a = Eigen::VectorXd(6);
    F_h = Eigen::VectorXd(6);

    debug << "\n||D_a||: \n" << D_a << std::endl;
    debug << "\n||M_a||: \n" << M_a << std::endl;

}

HindranceDetector::~HindranceDetector() {
    stopMonitor();
}

// h as in Eq8 of paper. range of h: 0-1
// h=0 -> robot free to move
// h=1 -> persistent external force exist (eg. obstacle hit / human trying to hold and guide the robot)
double HindranceDetector::computeAdmittanceRatio()
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
Eigen::VectorXd HindranceDetector::computeAdmittanceOutput(Eigen::VectorXd F_ext)
{
    std::lock_guard<std::mutex> lock(update_mutex_); 

    // Control Loop
    Eigen::VectorXd ddx_a = M_a_inv * (-M_a_inv * dx_a + F_ext);

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

bool HindranceDetector::startMonitor(){

    if (pendingStop_) return false;
    if (isRunning_) return true;
    
    // redundant safety, ensure the old loopThread is joined before overwritten with new one
    if (loopThread_.joinable()) loopThread_.join();

    loopThread_ = std::thread([&](){
        isRunning_ = true;
        int dt_ms = 1000 / refreshHz;
        std::array<double, 6> data_darr;
        
        while(true){
            if (pendingStop_) break;

            auto start = std::chrono::system_clock::now();

            // main update logic
            dataSrc->getLatestDataDArray(data_darr);
            Eigen::VectorXd F_ext = Eigen::VectorXd::Map(data_darr.data(), data_darr.size() );
            computeAdmittanceOutput(F_ext);

            // trigger callback on Hinderance detected/cleared
            if (h>0.7 && !hasHinderance_){
                hasHinderance_ = true;
                for (auto& observer : hookedObservers_){
                    observer->onHindranceDetected();
                }                
            }
            else if (h<0.3 && hasHinderance_){
                hasHinderance_ = false;
                for (auto& observer : hookedObservers_){
                    observer->onHindranceCleared();
                }
            }

            auto end = std::chrono::system_clock::now();
            std::chrono::duration<double, std::milli> duration = end-start;

            int wait_ms = dt_ms - duration.count() -1 ; 
            if (wait_ms > 0){
                std::this_thread::sleep_for(std::chrono::milliseconds(wait_ms));                
            }
            else{
                debug << "EEHindranceDetector loop cannot keep up with update Hz : " << refreshHz << "\n";
            }
            
        }

        isRunning_ = false;

    });

    return true;
}

bool HindranceDetector::stopMonitor(){
    pendingStop_ = true;
    if (loopThread_.joinable()) loopThread_.join();
    pendingStop_ = false;

    return true;
}

void HindranceDetector::addObserver(IHindranceObserver* obs){
    std::lock_guard<std::mutex> lock(hookedObservers_mutex_);

    for (const auto& p : hookedObservers_){
        if (p==obs) return;
    }
    
    hookedObservers_.push_back(obs);
}

void HindranceDetector::delObserver(IHindranceObserver* obs){
    std::lock_guard<std::mutex> lock(hookedObservers_mutex_);
    hookedObservers_.erase(std::remove(hookedObservers_.begin(), hookedObservers_.end(), obs), hookedObservers_.end());
}
