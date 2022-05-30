#ifndef _STANDWORKER_
#define _STANDWORKER_

#include "StateWorker.h"
#include "FSM_data.h"


class StandWorker: public StateWorker{
private:

public:
    uint32_t iter_run;
    float iter_time_ms;
    FSM_data * data_;
    virtual void run();
    virtual bool is_finished();

    StandWorker(FSM_data &data_);
    ~StandWorker();
};

StandWorker::StandWorker(FSM_data &data) {
    this->iter_run=0;
    this->iter_time_ms=0.0f;
    this->data_ = &data;    
}

StandWorker::~StandWorker() {
}

//不会一直卡在一个run中运行
void StandWorker::run() {
    
    
    
    //leg_angel 增减
    this-> iter_run++;
    std::cout<<"iter_run"<<iter_run<<std::endl;
    return;
}


bool StandWorker::is_finished() {
    if(iter_run>20000)
        {return true;}
    else
        {return false;}
     
}

#endif
