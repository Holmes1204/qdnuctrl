//
// Created by demphi on 2021/10/1.
//

#ifndef _FINITESTATEMACHINE_
#define _FINITESTATEMACHINE_

#include "stdarg.h"
#include "Workers_common_include.h"
#include "StateWorkerInclude/common_defination.h"
#include "FSM_data.h"
#include "FSM_tpcl.h"


class FSM{
private:

public:
    std::vector<StateWorker*> Workers;
    int flow = 0;
    //New ADD
    FSM_data global_data;
    FSM_topic_control topic_contrl;
    FSM(ros::NodeHandle &nh);
    ~FSM();
    void loop();
    void build_ScheduleTable(int Schedule, ...);

};

FSM::FSM(ros::NodeHandle &nh) :global_data(),topic_contrl(nh,global_data)
{
}

FSM::~FSM() {
    // release
    for(auto each:this->Workers){
        delete each;
    }
}

void FSM::loop() {
    /*! running the schedule table */
    if(this->Workers[this->flow]->is_finished()){
        this->flow++;
        if(this->flow == this->Workers.size()){
            ROS_INFO("Finish ScheduleTable");
            exit(0);
        }
    }
    else{
        /*! Update Phases and Bezier Curve in different gait */
        this->Workers[this->flow]->run();
        //CONTROL
        //send msg 应该封装入子State中不应该放在运行的FSM中
        this->topic_contrl.em_cmd_send();
    }
}

void FSM::build_ScheduleTable(int Schedule, ...) {
    va_list arg_ptr;
    va_start(arg_ptr, Schedule);
    while(Schedule != quad::END){
        switch (Schedule) {
            case quad::STEADY:{
                SteadyWorker* tmp_Worker = new SteadyWorker(this->global_data);
                this->Workers.push_back((StateWorker *)tmp_Worker);
                break;
            }
            case quad::STAND:{
                StandWorker* tmp_Worker = new StandWorker(this->global_data);
                this->Workers.push_back((StateWorker *)tmp_Worker);
                break;
            }
            case quad::STOP:{
                StopWorker* tmp_Worker = new StopWorker(this->global_data);
                this->Workers.push_back((StateWorker *)tmp_Worker);
                break;
            }
            default:
                ROS_ERROR("Wrong type of Schedule Table");
                exit(0);
                break;
        }
        Schedule = va_arg(arg_ptr, int);
    }
    return;
}

#endif
