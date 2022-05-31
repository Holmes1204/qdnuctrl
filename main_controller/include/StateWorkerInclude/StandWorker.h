#ifndef _STANDWORKER_
#define _STANDWORKER_
#include "StateWorker.h"

class StandWorker: public StateWorker{
private:

public:
    uint32_t iter_run;
    float iter_time_ms;
    FSM_data &data_;
    virtual void run();
    virtual bool is_finished();

    StandWorker(FSM_data &data_);
    ~StandWorker();
};

StandWorker::StandWorker(FSM_data &data):data_(data){
    this->iter_run=0;
    this->iter_time_ms=0.0f;
}

StandWorker::~StandWorker() {

}

//不会一直卡在一个run中运行
void StandWorker::run() {
    for (int i = 0; i < 4; i++)
    {
        data_.leg[i].p_ref.x()=data_.leg[i].base_p.x();
        data_.leg[i].p_ref.y()=data_.leg[i].base_p.y();
        data_.leg[i].p_ref.z()=data_.leg[i].base_p.z()-iter_run/1000.0f*0.02;
    }
    //leg_angel 增减
    this-> iter_run++;
    //std::cout<<"iter_run"<<iter_run<<std::endl;
    return;
}


bool StandWorker::is_finished() {
    if(0)
        {return true;}
    else
        {return false;}
}
#endif
