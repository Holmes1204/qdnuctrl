#ifndef _STOPWORKER_
#define _STOPWORKER_

#include "StateWorker.h"
#include "FSM_data.h"


class StopWorker: public StateWorker{
private:

public:
    uint32_t iter_run;
    float iter_time_ms;
    FSM_data &data_;
    virtual void run();
    virtual bool is_finished();

    StopWorker(FSM_data &data_);
    ~StopWorker();
};

StopWorker::StopWorker(FSM_data &data):data_(data){
    this->iter_run=0;
    this->iter_time_ms=0.0f;
}

StopWorker::~StopWorker() {

}

//不会一直卡在一个run中运行
void StopWorker::run() {
    data_.Off();
    this-> iter_run++;
    //std::cout<<"iter_run"<<iter_run<<std::endl;
    return;
}


bool StopWorker::is_finished() {
    if(iter_run>400)
        {return true;}
    else
        {return false;}
}
#endif
