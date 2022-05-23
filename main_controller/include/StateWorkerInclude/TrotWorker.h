//
// Created by demphi on 2021/10/1.
//

#ifndef _TROTWORKER_
#define _TROTWORKER_

class TrotWorker:public StateWorker{
private:
    const double foot2ground_ = 0.1;
    int count_;
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_cur;
    Eigen::Matrix<double, 3, NUM_LEG> foot_vel_cur;
    Eigen::Matrix<float, 1, NUM_LEG> spline_time;
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_target;
    Eigen::Matrix<double, 3, NUM_LEG> foot_vel_target;
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_error;
    Eigen::Matrix<double, 3, NUM_LEG> foot_vel_error;

public:
    ros::NodeHandle nh_;

    GaitScheduler* model_GS = new GaitScheduler(quad::TROT, 0.01);
    FootSwingTrajectory* model_FST[4];
    BezierUtils bezierUtils[NUM_LEG];

    virtual void run(STATE_INTERIOR *cur_state);
    virtual bool is_finished();
    TrotWorker(ros::NodeHandle &nh);
    ~TrotWorker();
};

TrotWorker::TrotWorker(ros::NodeHandle &nh) {
    this->nh_ = nh;
    this->foot_pos_cur.setZero();
    this->foot_vel_cur.setZero();
    this->spline_time.setZero();
    this->foot_pos_target.setZero();
    this->foot_vel_target.setZero();
    this->foot_pos_error.setZero();
    this->foot_vel_error.setZero();

    for (int i = 0; i < 4; i++) {
        this->model_FST[i] = new FootSwingTrajectory();
    }
}

TrotWorker::~TrotWorker() {
    delete this->model_GS;
    for (int i = 0; i < 4; i++) {
        delete this->model_FST[i];
    }
}

void TrotWorker::run(STATE_INTERIOR *cur_state) {
    if(this->count_++ >= 100){
        ROS_INFO("Trotting");
        this->count_ = 0;
    }
    //run gait table






}

bool TrotWorker::is_finished() {
    return false;
}

#endif 
