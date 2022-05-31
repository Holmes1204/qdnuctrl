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
        data_.leg[i].p_ref.z()=data_.leg[i].base_p.z()-0.005-iter_run/2000.0f*0.03;
    }

    if (iter_run%50==0){
        // std::cout<<"------------"<<std::endl;
        // std::cout<<0<<data_.leg[0].p_ref.transpose()<<std::endl;
        // std::cout<<0<<data_.leg[0].p.transpose()<<std::endl;
        // std::cout<<0<<data_.leg[0].force.transpose()<<std::endl;
    }

    //leg_angel 增减
    this-> iter_run++;
    //std::cout<<"iter_run"<<iter_run<<std::endl;
    return;
}


bool StandWorker::is_finished() {
    if(iter_run>2000)
        {return true;}
    else
        {return false;}
}
#endif
