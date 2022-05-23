//
// Created by demphi on 2021/10/2.
//

#ifndef _GAITTABLE_
#define _GAITTABLE_
#include "reference/common_include.h"

/**
 * @brief Hopf oscillator calculated from matlab and form a time table to search
 */
class GaitTable{
public:
    int count = 0;
    bool is_init = 0;
    std_msgs::Float64MultiArray angle;
    //rad table
    std::vector<float> walk_table_thigh = {
        -0.027801,-0.036719,-0.045236,-0.053316,-0.061291,-0.06911,-0.076684,-0.083997,
        -0.09096,-0.092094,-0.097539,-0.10365,-0.1093,-0.11436,-0.11888,-0.12272,
        -0.12595,-0.12845,-0.12916,-0.13031,-0.13113,-0.12972,-0.12417,-0.11543,
        -0.10344,-0.088866,-0.072024,-0.062984,-0.053595,-0.033997,-0.013839,0.0064049,
        0.0263,0.04534,0.063264,0.079476,0.089284,0.093909,0.10609,0.11607,
        0.12344,0.12834,0.13054,0.13096,0.13059,0.13009,0.12998,0.1292,
        0.12831,0.12726,0.12606,0.12466,0.12305,0.12119,0.11906,0.11663,
        0.11389,0.11082,0.1074,0.10361,0.099424,0.094817,0.090052,0.08925,
        0.085026,0.079634,0.073857,0.067714,0.061204,0.054363,0.047173,0.039668,
        0.037103,0.031867,0.023787,0.015497,0.0070157,-0.0015881,-0.010297,-0.019058
    };
    std::vector<float> walk_table_leg = {
        0,0,0,0,0,0,0,0,
        0,0,0,0,0,0,0,0,
        0,0,0,0,0,-0.014101,-0.030087,-0.044538,
        -0.057522,-0.068866,-0.07815,-0.082042,-0.085393,-0.090297,-0.093002,-0.093351,
        -0.091606,-0.087679,-0.081902,-0.074293,-0.068377,-0.065236,-0.054799,-0.043386,
        -0.031187,-0.018709,-0.0071379,0,0,0,0,0,
        0,0,0,0,0,0,0,0,
        0,0,0,0,0,0,0,0,
        0,0,0,0,0,0,0,0,
        0,0,0,0,0,0,0,0,
    };

    GaitTable();
    ~GaitTable();
    std_msgs::Float64MultiArray Get_Angle_From_Table();
};

GaitTable::GaitTable() {
    this->is_init = true;
    this->angle.data.resize(12);
    this->angle.data[0] = quad::DEFAULT_SHOULD_ANGLE;
    this->angle.data[3] = quad::DEFAULT_SHOULD_ANGLE;
    this->angle.data[6] = quad::DEFAULT_SHOULD_ANGLE;
    this->angle.data[9] = quad::DEFAULT_SHOULD_ANGLE;
}

GaitTable::~GaitTable() {

}

std_msgs::Float64MultiArray GaitTable::Get_Angle_From_Table() {
    if(!this->is_init){
        ROS_INFO("Init failed!");
    }
    // table 0->79
    if (this->count == 80){
        count = 0;
    }
    //LF
    this->angle.data[1] = this->walk_table_thigh[count];
    this->angle.data[2] = this->walk_table_leg[count];

    //RF
    this->angle.data[4] = this->walk_table_thigh[(count+40)%80];
    this->angle.data[5] = this->walk_table_leg[(count+40)%80];

    //RH
    this->angle.data[7] = this->walk_table_thigh[(count+60)%80];
    this->angle.data[8] = this->walk_table_leg[(count+60)%80];

    //LH
    this->angle.data[10] = this->walk_table_thigh[(count+20)%80];
    this->angle.data[11] = this->walk_table_leg[(count+20)%80];
    
    this->count++;

    return this->angle;
}




#endif
