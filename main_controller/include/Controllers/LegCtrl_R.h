#ifndef _LEGCTRL_R_
#define _LEGCTRL_R_
#include "common_parameter.h"

/*!
* Data returned from the legs to the control code.
 */
struct Leg_Data{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Vec3 base_p;
    Vec3 kp,kd;
    Vec3 p, v;
    Vec3 p_ref,v_ref;
    Vec3 force;
    Vec3 q,qd;
    Vec3 q_ref,qd_ref;
    Vec3 trq;
    Mat33 J;
    const float l1 =0.02f,l2=0.02f;
    Leg_Data(Vec3 &base);
    Leg_Data();
    Vec3 get_force();
    Vec3 get_torque();
    Vec3 get_foot_pos(int _leg);
    void compute_Jacobian(int _leg);
    Vec3 get_foot_vel();
};

Leg_Data::Leg_Data(Vec3 &base):base_p(base)
{
    base_p =Vec3::Zero();
    p = Vec3::Zero();
    v = Vec3::Zero();
    p_ref = Vec3::Zero();
    v_ref = Vec3::Zero();
    J = Mat33::Zero();
    q = Vec3::Zero();
    qd = Vec3::Zero();
    q_ref = Vec3::Zero();
    qd_ref = Vec3::Zero();
    trq= Vec3::Zero();
    force = Vec3::Zero();
    kp=Vec3::Zero();
    kd=Vec3::Zero();
    //
    kp.x() = 1;
    kp.y() = 1;
    kp.z() = 1;
    //N/m/s
    kd.x() = 0.01;
    kd.y()= 0.01;
    kd.z() = 0.01;
}
Leg_Data::Leg_Data()
{
    base_p =Vec3::Zero();
    p = Vec3::Zero();
    v = Vec3::Zero();
    p_ref = Vec3::Zero();
    v_ref = Vec3::Zero();
    J = Mat33::Zero();
    q = Vec3::Zero();
    qd = Vec3::Zero();
    q_ref = Vec3::Zero();
    qd_ref = Vec3::Zero();
    trq= Vec3::Zero();
    force = Vec3::Zero();
    kp=Vec3::Zero();
    kd=Vec3::Zero();
    //N/m
    kp.x() = 1;
    kp.y() = 1;
    kp.z() = 1;
    //N/m/s
    kd.x() = 0.01;
    kd.y()= 0.01;
    kd.z() = 0.01;

}

//计算jacobian
void Leg_Data::compute_Jacobian(int _leg) {
        double s1 = std::sin(q(1));
        double s2 = std::sin(q(2));
        double c1 = std::cos(q(1));
        double c2 = std::cos(q(2));
        double s12 = std::sin(q(1)+q(2));
        double c12 = std::cos(q(1)+q(2));
        J(0, 0) = 0;
        J(1, 1) = 0;
        J(1, 2) = 0;
        J(2, 0) = 0;
        J(1, 0) = 0;
        switch (_leg)
        {
        case LF:
        case LB:
            J(0, 1) = (l1 * c1 + l2 * c12);
            J(0, 2) = (l2 * c12);
            J(2, 1) = l1 * s1 + l2 * s12;
            J(2, 2) = l2 * s12;
            break;
        case RF:
        case RB:
            J(0, 1) = -(l1 * c1 + l2 * c12);
            J(0, 2) = -(l2 * c12);
            J(2, 1) = l1 * s1 + l2 * s12;
            J(2, 2) = l2 * s12;
            break;
        default:
            break;
        }
    }
//forward kinematic
    Vec3 Leg_Data::get_foot_pos(int _leg){
        switch (_leg)
        {
        case LF:
        case LB:
            this->p.x() = this->base_p.x() + this->l1*std::sin(q(1))+ this->l2*std::sin(q(1)+q(2));
            this->p.y() = this->base_p.y();
            this->p.z() = this->base_p.z() - this->l1*std::cos(q(1))-this->l2*std::cos(q(1)+q(2));
            break;
        case RF:
        case RB:
            this->p.x() = this->base_p.x() - this->l1*std::sin(q(1))- this->l2*std::sin(q(1)+q(2));
            this->p.y() = this->base_p.y();
            this->p.z() = this->base_p.z() - this->l1*std::cos(q(1))-this->l2*std::cos(q(1)+q(2));
        default:
            break;
        }
        return this->p;
    }
    Vec3 Leg_Data::get_foot_vel(){
        this->v = this->J*this->qd;
        return this->v;
    }
//calcualte desire force
    Vec3 Leg_Data::get_force(){
        this->force.x() = this->kp.x()*(this->p_ref.x()-this->p.x())+ this->kd.x()*(this->v_ref.x()-this->v.x());
        this->force.y() = this->kp.y()*(this->p_ref.y()-this->p.y())+ this->kd.y()*(this->v_ref.y()-this->v.y());
        this->force.z() = this->kp.z()*(this->p_ref.z()-this->p.z())+ this->kd.z()*(this->v_ref.z()-this->v.z());
        return this->force;
    }
//calculate desire torque
    Vec3 Leg_Data::get_torque(){
        get_force();
        this->trq = this->J.transpose()*this->force;
        return this->trq;
    }

#endif 
