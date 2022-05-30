#include <FiniteStateMachine.h>
#include <iostream>
#include <ros/ros.h>


int main(int argc, char** argv){
    ros::init(argc, argv, "demo");
    ros::NodeHandle nh;
    FSM Finite_State_Machine(nh);
    sleep(2);
    Finite_State_Machine.build_ScheduleTable(
            //加入steady state
            quad::STAND,
            //quad::WALK,
//            quad::TROT,
//           quad::PACE,
//           quad::GALLOP,
            quad::END
            );
    while (ros::ok()) {
        std::cout<<"runing"<<std::endl;
        ros::Duration(0.0025).sleep();
        Finite_State_Machine.loop();
    }
    return 0;
}