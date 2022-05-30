#ifndef _STATEWORKER_
#define _STATEWORKER_

class StateWorker{
public:
    virtual void run() = 0;
    virtual bool is_finished() = 0;
    bool is_working = 0;
    StateWorker(){};
    ~StateWorker(){};
};

#endif
