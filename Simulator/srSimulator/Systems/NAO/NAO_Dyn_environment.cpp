#include "nao.h"
#include "NAO_Dyn_environment.h"
#include <iostream>
#include "srDyn/srSpace.h"
#include <stdio.h>
#include "common/utils.h"

#include <srConfiguration.h>
#include <Configuration.h>
#include <srTerrain/Ground.h>
#include <srTerrain/Config_Space_Definition.h>
//#include <NAO_Controller/StateProvider.h>
//#include <NAO_Controller/interface.h>

NAO_Dyn_environment::NAO_Dyn_environment():
    indicated_contact_pt_list_(4),
    commanded_contact_force_list_(4),
    ang_vel_(3)
{

    /********** Space Setup **********/
    m_Space = new srSpace();
    m_ground = new Ground();
    m_Space->AddSystem(m_ground->BuildGround());

    robot_ = new srNao();
    robot_->BuildRobot(Vec3 (0., 0., 0.), srSystem::FIXED, 
            srJoint::TORQUE, ModelPath"NAO_Model/urdf/nao_simple.urdf");
    m_Space->AddSystem((srSystem*)robot_);

    //interface_ = new Interface();
    //contact_pt_list_.clear();
    contact_force_list_.clear();

    m_Space->DYN_MODE_PRESTEP();
    m_Space->SET_USER_CONTROL_FUNCTION_2(ContolFunction);
    m_Space->SetTimestep(0.001);
    m_Space->SetGravity(0.0,0.0,-9.81);
    m_Space->SetNumberofSubstepForRendering(20);

    printf("[Nao Dynamic Environment] Build Dynamic Environment\n");
}

void NAO_Dyn_environment::ContolFunction( void* _data ) {
    //static int count(0);
    //++count;

    //NAO_Dyn_environment* pDyn_env = (NAO_Dyn_environment*)_data;
    //srNao* robot = (srNao*)(pDyn_env->robot_);
    //double alternate_time = SIM_SERVO_RATE * count;

    //std::vector<double> jpos(robot->num_act_joint_);
    //std::vector<double> jvel(robot->num_act_joint_);
    //std::vector<double> jtorque(robot->num_act_joint_);
    //dynacore::Vect3 pos;
    //dynacore::Quaternion rot;
    //dynacore::Vect3 body_vel;
    //dynacore::Vect3 ang_vel;
    //std::vector<double> torque_command(robot->num_act_joint_);

    //for(int i(0); i<robot->num_act_joint_; ++i){
        //jpos[i] = robot->r_joint_[i]->m_State.m_rValue[0];
        //jvel[i] = robot->r_joint_[i]->m_State.m_rValue[1];
        //jtorque[i] = robot->r_joint_[i]->m_State.m_rValue[3];
    //}

    //for (int i(0); i<3; ++i){
        //pos[i] = robot->vp_joint_[i]->m_State.m_rValue[0];
        //body_vel[i] = robot->vp_joint_[i]->m_State.m_rValue[1];
        //ang_vel[i] = robot->link_[robot->link_idx_map_.find("torso")->second]->GetVel()[i];
    //}

    //pDyn_env->_Get_Orientation(rot);
    //pDyn_env->interface_->GetCommand(alternate_time, jpos, jvel, jtorque, pos, rot, body_vel, ang_vel, torque_command);

    //for(int i(0); i<3; ++i){
        //robot->vp_joint_[i]->m_State.m_rCommand = 0.0;
        //robot->vr_joint_[i]->m_State.m_rCommand = 0.0;
    //}

    //pDyn_env->_ListCommandedReactionForce(StateProvider::GetStateProvider()->Fr_);

    //for(int i(0); i<robot->num_act_joint_; ++i){
        //robot->r_joint_[i]->m_State.m_rCommand = torque_command[i];
    //}

}

void NAO_Dyn_environment::Rendering_Fnc()
{
    //_Draw_CoM3DAcc_Point();
    //_Draw_CoMAcc();

    //_Draw_Contact_Point();
    // _Draw_Contact_Force();
    //_Draw_Commanded_Force();
    // _Draw_Path();
    // _Draw_FootPlacement();
}

void NAO_Dyn_environment::_Draw_Contact_Point(){
    double radi(0.02);

    double theta(0.0);
    dynacore::Vect3 contact_loc;
    for (int j(0); j<contact_pt_list_.size(); ++j){
        contact_loc = contact_pt_list_[j];
        glBegin(GL_LINE_LOOP);
        for (int i(0); i<3600; ++i){
            theta += DEG2RAD(i*0.1);
            glColor4f(0.5f, 0.1f, 0.5f, 0.1f);

            glVertex3f(contact_loc[0] + cos(theta)*radi, contact_loc[1] + sin(theta)*radi, contact_loc[2]);
        }
        glEnd();
    }
}
void NAO_Dyn_environment::_ListCommandedReactionForce(const dynacore::Vector & Fr){
    dynacore::Vect3 vec3_fr;
    dynacore::Vect3 vec3_cp;
    Vec3 contact_point;

    for(int i(0); i<4; ++i){
        contact_point = robot_->link_[robot_->link_idx_map_.find("FootOutFront")->second+ i]->GetPosition();
        for (int j(0); j<3; ++j){
            vec3_cp[j] = contact_point[j];
            vec3_fr[j] = Fr[3*i + j];
        }
        indicated_contact_pt_list_[i] = vec3_cp;
        commanded_contact_force_list_[i] = vec3_fr;
    }
}

void NAO_Dyn_environment::_Draw_Commanded_Force(){
    dynacore::Vect3 contact_loc;
    dynacore::Vect3 contact_force;
    double reduce_ratio(0.001);

    for (int j(0); j<indicated_contact_pt_list_.size(); ++j){
        glLineWidth(2.5);
        glColor4f(0.9f, 0.0f, 0.0f, 0.8f);
        glBegin(GL_LINES);

        contact_loc = indicated_contact_pt_list_[j];
        contact_force = reduce_ratio * commanded_contact_force_list_[j];
        contact_force += contact_loc;

        glVertex3f(contact_loc[0],  contact_loc[1],  contact_loc[2]);
        glVertex3f(contact_force[0], contact_force[1], contact_force[2]);
        glEnd();
    }
}

void NAO_Dyn_environment::_Get_Orientation(dynacore::Quaternion & rot){
    SO3 so3_body =  robot_->link_[robot_->link_idx_map_.find("torso")->second]->GetOrientation();

    Eigen::Matrix3d ori_mtx;
    for (int i(0); i<3; ++i){
        ori_mtx(i, 0) = so3_body[0+i];
        ori_mtx(i, 1) = so3_body[3+i];
        ori_mtx(i, 2) = so3_body[6+i];
    }
    dynacore::Quaternion ori_quat(ori_mtx);
    rot = ori_quat;
}

NAO_Dyn_environment::~NAO_Dyn_environment()
{
    SR_SAFE_DELETE(interface_);
    SR_SAFE_DELETE(robot_);
    SR_SAFE_DELETE(m_Space);
    SR_SAFE_DELETE(m_ground);
}


