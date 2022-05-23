//
// Created by demphi on 2021/10/1.
//

#ifndef _STANDWORKER_
#define _STANDWORKER_

#include "StateWorker.h"
#include "geometry_msgs/Vector3.h"
#include "std_msgs/Float64MultiArray.h"
#include "unitree_legged_msgs/LowCmd.h"
#include "unitree_legged_msgs/MotorCmd.h"

/**
 * @brief send angle msg and stand
 * @param none
 */
class StandWorker: public StateWorker{
private:
    /*! max = pi/6*/


public:
    ros::NodeHandle nh_;
    
    virtual void run(STATE_INTERIOR *cur_state);
    virtual bool is_finished();

    StandWorker(ros::NodeHandle &nh);
    ~StandWorker();

};

StandWorker::StandWorker(ros::NodeHandle &nh) {
    this->nh_ = nh;

    /*! Init some param*/
    this->init_time_ = ros::Time::now().toSec();
    this->end_time_ = 0.0;

}

StandWorker::~StandWorker() {

}

void StandWorker::run(STATE_INTERIOR *cur_state) {
    if(this->count_++ > 100){
        ROS_INFO("Standing");
        this->count_ = 0;
    }
    //dt





    cur_state->gait_type = quad::TROT;
    cur_state->gait_type = quad::END;
    /*! Change Gait Type */
    if(cur_state->gait_type != quad::STAND){
        this->flag_ = 2;
    }
}

bool StandWorker::is_finished() {
    if(this->flag_ == 2){
        ROS_INFO("Finish Stand State");
        return true;
    }else{
        return false;
    }

}

#endif
