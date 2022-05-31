#ifndef _FSM_DATA_
#define _FSM_DATA_
#include "wtr_serial_msg/em_ev.h"
#include "wtr_serial_msg/em_fb_raw.h"
#include "Controllers/LegCtrl_R.h"

class FSM_data
{
private:
    /* data */
public:
    wtr_serial_msg::em_ev mt_cmd;
    wtr_serial_msg::em_fb_raw mt_fdb;
    Leg_Data leg[4];
    uint32_t motor_state=0;
    void UpdateData();
    void UpdateCmd();
    FSM_data();
    ~FSM_data();
    void On(){
        this->motor_state=1;
    }
    void Off(){
        this->motor_state=0;
    }
};

FSM_data::FSM_data()
{
    
}

FSM_data::~FSM_data()
{

}

void FSM_data::UpdateData() {
    //std::cout<<"--------------------\n";
    for (int i = 0; i < 4; i++)
    {
        leg[i].q(1) = mt_fdb.em_pos_fb_raw[i*2];
        leg[i].qd(1) = mt_fdb.em_vel_fb_raw[i*2];
        leg[i].q(2) = mt_fdb.em_pos_fb_raw[i*2+1];
        leg[i].qd(2) = mt_fdb.em_vel_fb_raw[i*2+1];
        leg[i].compute_Jacobian(i);
        leg[i].get_foot_pos(i);
        leg[i].get_foot_vel();
        //debug
        //std::cout<<"leg"<<i<<std::endl;
        //std::cout<<leg[i].J<<std::endl;
        //std::cout<<"q "<<leg[i].q.transpose()<<"\n"<<"d "<<leg[i].qd.transpose()<<std::endl;
        //std::cout<<"p "<<leg[i].p.transpose()<<"\n"<<"v "<<leg[i].v.transpose()<<std::endl;
    }
}


void FSM_data:: UpdateCmd(){
    //std::cout<<"--------------------\n";
    for (int i = 0; i < 4; i++)
    {
        leg[i].get_motor_angle(i);
        //std::cout<<i<<" "<<leg[i].p_ref.transpose()<<std::endl;
        //std::cout<<i<<" "<<leg[i].q_ref.transpose()<<std::endl;
        mt_cmd.em_ev_kp[2*i]=40;
        mt_cmd.em_ev_kd[2*i]=1;
        mt_cmd.em_ev_pos[2*i]=leg[i].q_ref(1);
        mt_cmd.em_ev_vel[2*i]=0;
        mt_cmd.em_ev_trq[2*i]=0;
        mt_cmd.em_ev_kp[2*i+1]=40;
        mt_cmd.em_ev_kd[2*i+1]=1;
        mt_cmd.em_ev_pos[2*i+1]=leg[i].q_ref(2);
        mt_cmd.em_ev_vel[2*i+1]=0;
        mt_cmd.em_ev_trq[2*i+1]=0;
        mt_cmd.em_state[0]=this->motor_state;
        mt_cmd.em_state[1]=0;
        //std::cout<<i<<" "<<torque_.transpose()<<std::endl;
        //std::cout<<leg[i].J<<std::endl;
    }
}


#endif //_FSM_DATA_
