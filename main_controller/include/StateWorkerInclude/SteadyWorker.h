#ifndef _STEADYWORKER_
#define _STEADYWORKER_

#include "StateWorker.h"
class SteadyWorker: public StateWorker{
private:

public:
    uint32_t iter_run;
    float iter_time_ms;
    FSM_data &data_;
    virtual void run();
    virtual bool is_finished();

    SteadyWorker(FSM_data &data_);
    ~SteadyWorker();
};

SteadyWorker::SteadyWorker(FSM_data &data):data_(data){
    this->iter_run=0;
    this->iter_time_ms=0.0f;
}

SteadyWorker::~SteadyWorker() {

}

//不会一直卡在一个run中运行
void SteadyWorker::run() {
    if (iter_run%10==0)
        for (int i = 0; i < 4; i++)
        {
            std::cout<<i<<" "<<data_.leg[i].q.transpose()<<std::endl;
        }
    this-> iter_run++;
    return;
}


bool SteadyWorker::is_finished() {
    if(0)
        {
            //data_.On();
            for (int i = 0; i < 4; i++)
            {
                std::cout<<i<<" "<<data_.leg[i].q.transpose()<<std::endl;
            }
            std::cout<<"Steady State Over!\n"<<std::endl;
            return true;
        }
    else
        {return false;}
}
#endif
