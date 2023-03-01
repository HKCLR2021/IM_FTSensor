#include <iostream>

#include "ft_sensor/HindranceDetector.hpp"
#include "ft_sensor/OnRobotForceTorqueSensor.hpp"

#define debug if(1) std::cerr

HindranceDetector::HindranceDetector(
    std::shared_ptr<OnRobotForceTorqueSensor> dataSrc,    
    int refreshHz, double sensitivity,
    double min_activate_force_N,
    double min_activate_torque_Nm,
    double force_thres_high
    )
{
    this->dataSrc = dataSrc;    
    this->refreshHz = refreshHz;
    this->sensitivity = sensitivity;
    this->min_activate_force_N   = min_activate_force_N;
    this->min_activate_torque_Nm = min_activate_torque_Nm;
    this->force_thres_high = force_thres_high;

    Eigen::VectorXd D_a_diag(6);
    D_a_diag << 20, 20, 20, 10, 10, 10;
    D_a = D_a_diag.asDiagonal();

    Eigen::VectorXd M_a_diag(6);
    M_a_diag << 15, 15, 15, 7.5, 7.5, 7.5;
    M_a = M_a_diag.asDiagonal();
    M_a_inv = M_a.inverse();

    dt = 1.0 / refreshHz;
    Pd_tilde = 0.25; // 1.0; // 0.25; 
    E_thres = 10;
    E_max = 50;

    reset();

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
    for (int i=0;i<3;++i){
        F_ext[i] *= sensitivity;
        if ( abs(F_ext[i])<min_activate_force_N ) F_ext[i]=0; 
    }

    for (int i=3;i<6;++i){
        F_ext[i] *= sensitivity;
        if ( abs(F_ext[i])<min_activate_torque_Nm ) F_ext[i]=0; 
    }

    std::lock_guard<std::mutex> lock(update_mutex_); 

    // Control Loop
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

bool HindranceDetector::startMonitor(bool verbose){

    verbose_ = verbose;
    if (pendingStop_) return false;
    if (isRunning_) return true;
    
    // redundant safety, ensure the old loopThread is joined before overwritten with new one
    if (loopThread_.joinable()) loopThread_.join();

    loopThread_ = std::thread([&](){
        isRunning_ = true;
        int dt_ms = 1000 / refreshHz;
        std::array<double, 6> data_darr;
        std::vector<double> data_vec;

        unsigned int loopIdx = 0;

        
        while(true){
            if (pendingStop_) break;

            auto start = std::chrono::system_clock::now();

            // main update logic
            // dataSrc->getLatestDataDArray(data_darr);
            // Eigen::VectorXd F_ext = Eigen::VectorXd::Map(data_darr.data(), data_darr.size() );
            auto prev_data_vec = data_vec;
            dataSrc->getLatestDataVec(data_vec);

            // detect datasource connection loss (i.e. data not changing)
            if (prev_data_vec == data_vec){
                staleDataCount += 1;
            }
            else{
                staleDataCount = 0;
            }
            if (!dataSrc->isFake() && staleDataCount == refreshHz*1){ // 1sec no change
                debug << "Stale unchanging force reading Detected" << std::endl;
                for (auto& observer : hookedObservers_){
                    observer->onDatasourceDead();
                }  
            }

            Eigen::VectorXd F_ext = Eigen::VectorXd::Map(data_vec.data(), data_vec.size() );

            computeAdmittanceOutput(F_ext);

            // if (verbose_ && loopIdx%20==0){
            //     debug << "sensor data : " << data_vec[0] << std::endl;
            //     debug << "sensor data : " << data_vec[1] << std::endl;
            //     debug << "sensor data : " << data_vec[2] << std::endl;
            //     debug << "sensor data : " << data_vec[3] << std::endl;
            //     debug << "sensor data : " << data_vec[4] << std::endl;
            //     debug << "sensor data : " << data_vec[5] << std::endl;
            //     debug << "H : " << h << " Admitance : " << std::endl;
            //     debug << dx_a << std::endl;
            //     loopIdx = 0;
            // }
            // loopIdx+=1;

            // ====================================================
            // trigger callback on Hinderance detected/cleared
            // ====================================================
            // if (h>0.7 && !hasHinderance_){
            //     hasHinderance_ = true;
            //     if (verbose) {
            //         debug << "Hindrance Detected" << std::endl;
            //         debug << "sensor data : " << data_vec[0] << std::endl;
            //         debug << "sensor data : " << data_vec[1] << std::endl;
            //         debug << "sensor data : " << data_vec[2] << std::endl;
            //         debug << "sensor data : " << data_vec[3] << std::endl;
            //         debug << "sensor data : " << data_vec[4] << std::endl;
            //         debug << "sensor data : " << data_vec[5] << std::endl;
            //         debug << "H : " << h << " Admitance : " << std::endl;
            //         debug << dx_a << std::endl;
            //     }
            //     for (auto& observer : hookedObservers_){
            //         observer->onHindranceDetected();
            //     }                
            // }
            // else if (h<0.3 && hasHinderance_){
            //     hasHinderance_ = false;
            //     if (verbose){
            //         debug << "Hindrance Cleared" << std::endl;
            //         debug << "H : " << h << " Admitance : " << std::endl;
            //         debug << dx_a << std::endl;
            //     } 
            //     for (auto& observer : hookedObservers_){
            //         observer->onHindranceCleared();
            //     }
            // }

            // ====================================================
            // QUICK HACK: trigger callback on raw force sensor x,y,z value
            // ====================================================
            double max_force_N = 0.0;
            for (int i=0;i<3;i++){
                max_force_N = std::max(abs(data_vec[i]) , max_force_N);
            }

            // double force_thres_high = 30.0;
            // double force_thres_high = 60.0;
            double force_thres_low = 15.0;

            if (max_force_N>(this->force_thres_high) && !hasHinderance_){
                hasHinderance_ = true;
                if (verbose) {
                    debug << "Hindrance Detected" << std::endl;
                    debug << "sensor data : " << data_vec[0] << std::endl;
                    debug << "sensor data : " << data_vec[1] << std::endl;
                    debug << "sensor data : " << data_vec[2] << std::endl;
                    debug << "sensor data : " << data_vec[3] << std::endl;
                    debug << "sensor data : " << data_vec[4] << std::endl;
                    debug << "sensor data : " << data_vec[5] << std::endl;
                    debug << "max_force_N : " << max_force_N << std::endl;
                }
                for (auto& observer : hookedObservers_){
                    observer->onHindranceDetected();
                }                
            }
            else if (max_force_N<force_thres_low && hasHinderance_){
                hasHinderance_ = false;
                if (verbose){
                    debug << "Hindrance Cleared" << std::endl;
                    debug << "max_force_N : " << max_force_N << std::endl;
                } 
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

    reset();

    return true;
}

void HindranceDetector::reset(){
    hasHinderance_ = false;
    h = 0.0;
    E_m = 0.0;
    dx_a = Eigen::VectorXd::Zero(6);
    F_h = Eigen::VectorXd::Zero(6);
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
