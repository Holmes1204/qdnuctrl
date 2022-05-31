#include <FiniteStateMachine.h>
#include <iostream>
#include <ros/ros.h>
int main(int argc, char** argv){
    ros::init(argc, argv, "demo");
    ros::NodeHandle nh;
    ros::Rate rate(400);//Hz
    FSM Finite_State_Machine(nh);
    printf("Start!\n");
    Finite_State_Machine.build_ScheduleTable(
            quad::STEADY,
            quad::STAND,
            quad::STOP,
            quad::END
            );
    sleep(2);
    printf("Motion Begin!\n");
    getchar();
    while (ros::ok()) {
        Finite_State_Machine.loop();
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}