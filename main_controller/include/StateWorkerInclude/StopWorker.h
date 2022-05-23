#ifndef _STOPWORKER_
#define _STOPWORKER_

#include "StateWorker.h"
#include "geometry_msgs/Vector3.h"
#include "std_msgs/Float64MultiArray.h"
#include "unitree_legged_msgs/LowCmd.h"
#include "unitree_legged_msgs/MotorCmd.h"

/**
 * @brief stop and down to the floor
 * @param none
 */
class StopWorker: public StateWorker{
private:
    int flag_ = 0;
    int count_;
    double command_hip_ = 0;
    double command_knee_ = 0;
    double init_time_, cur_time_, end_time_;
    double init_hip_ = 90.0, init_knee_ = -175.0;
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_cur;
    Eigen::Matrix<double, 3, NUM_LEG> foot_vel_cur;
    Eigen::Matrix<float, 1, NUM_LEG> spline_time;
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_target;
    Eigen::Matrix<double, 3, NUM_LEG> foot_vel_target;
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_error;
    Eigen::Matrix<double, 3, NUM_LEG> foot_vel_error;

public:
    ros::NodeHandle nh_;
    ros::Publisher pub_joint_cmd[12];
    BezierUtils bezierUtils[NUM_LEG];

    virtual void run(STATE_INTERIOR *cur_state);
    virtual bool is_finished();


    StopWorker(ros::NodeHandle &nh);
    ~StopWorker();

};




#endif

