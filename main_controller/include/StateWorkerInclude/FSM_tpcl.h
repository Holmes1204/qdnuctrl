#ifndef _FSM_TPCL_
#define _FSM_TPCL_
//all the methods regarding ros  topics
#include "ros/ros.h"
#include "FSM_data.h"

class FSM_topic_control
{
private:
public:
    ros::NodeHandle& nh_;
    ros::Publisher em_cmd_pub_;
    ros::Subscriber em_fdb_sub_;

    FSM_data& data_;

    FSM_topic_control(ros::NodeHandle &nh,FSM_data &data);
    ~FSM_topic_control();

    void em_fdb_callback(const wtr_serial_msg::em_fb_raw &joint_state);
    void em_cmd_send();
};

FSM_topic_control::FSM_topic_control(ros::NodeHandle &nh,FSM_data &data):nh_(nh),data_(data)
{
    this->em_cmd_pub_ = this->nh_.advertise<wtr_serial_msg::em_ev>("/em_ev", 1);
    this->em_fdb_sub_ = this->nh_.subscribe("/em_fb_raw", 2, &FSM_topic_control::em_fdb_callback ,this);
}

FSM_topic_control::~FSM_topic_control()
{
}

void FSM_topic_control::em_fdb_callback(const wtr_serial_msg::em_fb_raw &msg){
    data_.mt_fdb = msg;
    data_.UpdateData();
    return;
}

void FSM_topic_control::em_cmd_send(){
    data_.UpdateCmd();
    em_cmd_pub_.publish(data_.mt_cmd);
    return;
}
#endif //_FSM_TPCL_
