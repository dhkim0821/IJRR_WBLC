#include "SystemGenerator.h"
#include <Utils/utilities.hpp>
#include <sstream>

void SystemGenerator::_SetJointParam(int idx){
    URDFJointMap::iterator Jointidxiter = joint_map_.find(joint_names_[idx]);

    URDFLinkMap::iterator p_Linkidxiter = 
        link_map_.find(Jointidxiter->second->parent_link_name);
    URDFLinkMap::iterator c_Linkidxiter = 
        link_map_.find(Jointidxiter->second->child_link_name);

    Vec3 p_Link_offset;
    Vec3 c_Link_offset;

    if(p_Linkidxiter->second->inertial){
        p_Link_offset=
            Vec3( p_Linkidxiter->second->inertial->origin.position.x,
                  p_Linkidxiter->second->inertial->origin.position.y,
                  p_Linkidxiter->second->inertial->origin.position.z);
    }
    if(c_Linkidxiter->second->inertial){
        c_Link_offset = Vec3(c_Linkidxiter->second->inertial->origin.position.x,
                             c_Linkidxiter->second->inertial->origin.position.y,
                             c_Linkidxiter->second->inertial->origin.position.z);
    }
    Vec3 joint_rpy;
    Jointidxiter->second->parent_to_joint_origin_transform.rotation.getRPY (joint_rpy[0], joint_rpy[1], joint_rpy[2]);

    Vec3 JointOff_pos(Jointidxiter->second->parent_to_joint_origin_transform.position.x,
            Jointidxiter->second->parent_to_joint_origin_transform.position.y,
            Jointidxiter->second->parent_to_joint_origin_transform.position.z);


    Vec3 axis010=Vec3(0,0,-SR_PI_HALF);
    Vec3 neg_axis010=Vec3(0,0,-SR_PI_HALF);
    Vec3 axis100=Vec3(0,SR_PI_HALF,0);
    Vec3 neg_axis100=Vec3(0,-SR_PI_HALF,0);

    // Set Parent (link CoM) to joint positon
    for(int i(0);i<3;i++)
        JointOff_pos[i]= (JointOff_pos[i] - p_Link_offset[i]);

    Vec3 JointOff_ori(joint_rpy[2],joint_rpy[1],joint_rpy[0]);
    SE3 JointOffFrame(EulerZYX(Vec3(joint_rpy[2], joint_rpy[1], joint_rpy[0]), 
                JointOff_pos ));
    SE3 Axis010Frame(EulerZYX(Vec3(0, 0, -SR_PI_HALF), Vec3(0., 0., 0.)));
    SE3 negAxis010Frame(EulerZYX(Vec3(0, 0, SR_PI_HALF), Vec3(0., 0., 0.)));
    
    SE3 Axis100Frame(EulerZYX(Vec3(0, SR_PI_HALF, 0), Vec3(0,0,0)));
    SE3 negAxis100Frame(EulerZYX(Vec3(0, -SR_PI_HALF, 0), Vec3(0,0,0)));

    if(Jointidxiter->second->type == dynacore::urdf::Joint::FIXED){
        fixed_joint_[WJidx_]->GetGeomInfo().SetShape(srGeometryInfo::SPHERE);
        map<string,int>::iterator Linkidxiter=link_idx_map_.find(p_Linkidxiter->first);
        fixed_joint_[WJidx_]->SetParentLink(link_[Linkidxiter->second]);
        Linkidxiter=link_idx_map_.find(c_Linkidxiter->second->name);
        fixed_joint_[WJidx_]->SetChildLink(link_[Linkidxiter->second]);
        fixed_joint_[WJidx_]->SetParentLinkFrame(EulerZYX(JointOff_ori,JointOff_pos));
        fixed_joint_[WJidx_]->SetChildLinkFrame(EulerZYX(Vec3(),-c_Link_offset));
        WJidx_++;
    }

    else if(Jointidxiter->second->type == dynacore::urdf::Joint::REVOLUTE || Jointidxiter->second->type == dynacore::urdf::Joint::CONTINUOUS){
        double axis_x(Jointidxiter->second->axis.x);
        double axis_y(Jointidxiter->second->axis.y);
        double axis_z(Jointidxiter->second->axis.z);
        if(fabs(axis_x) == 1){ // Rotation Axis: X
            r_joint_[RJidx_]->GetGeomInfo().SetShape(srGeometryInfo::CYLINDER);
            map<string,int>::iterator Linkidxiter=
                link_idx_map_.find(p_Linkidxiter->first);
            r_joint_[RJidx_]->SetParentLink(link_[Linkidxiter->second]);
            Linkidxiter=link_idx_map_.find(c_Linkidxiter->second->name);
            r_joint_[RJidx_]->SetChildLink(link_[Linkidxiter->second]);
            SE3 ParentLinkFrame;
            if(axis_x>0){
                ParentLinkFrame = JointOffFrame * Axis100Frame;
                r_joint_[RJidx_]->SetParentLinkFrame(ParentLinkFrame);
                r_joint_[RJidx_]->SetChildLinkFrame(EulerZYX(axis100,-c_Link_offset));
            } else{
                ParentLinkFrame = JointOffFrame * 
                    SE3(EulerZYX(Vec3(0, -SR_PI_HALF, 0), Vec3(0,0,0)));
                r_joint_[RJidx_]->SetParentLinkFrame(ParentLinkFrame);
                r_joint_[RJidx_]->SetChildLinkFrame(EulerZYX(neg_axis100,-c_Link_offset));
            }
            RJidx_++;
        }

        else if(fabs(axis_y) == 1){
            r_joint_[RJidx_]->GetGeomInfo().SetShape(srGeometryInfo::CYLINDER);
            map<string,int>::iterator Linkidxiter=link_idx_map_.find(p_Linkidxiter->first);
            r_joint_[RJidx_]->SetParentLink(link_[Linkidxiter->second]);
            Linkidxiter=link_idx_map_.find(c_Linkidxiter->second->name);
            r_joint_[RJidx_]->SetChildLink(link_[Linkidxiter->second]);

            if(axis_y>0){
                SE3 ParentLinkFrame= JointOffFrame * Axis010Frame;
                r_joint_[RJidx_]->SetParentLinkFrame(ParentLinkFrame);
                r_joint_[RJidx_]->SetChildLinkFrame(EulerZYX(axis010,-c_Link_offset));
            }else{
                SE3 ParentLinkFrame= JointOffFrame * negAxis010Frame;
                //SE3 ParentLinkFrame= JointOffFrame * Axis010Frame;
                r_joint_[RJidx_]->SetParentLinkFrame(ParentLinkFrame);
                r_joint_[RJidx_]->SetChildLinkFrame(EulerZYX(neg_axis010,-c_Link_offset));
              std::cout << "Doublecheck :srSysGenerator/SystemGenerator"
                  <<"(joint param(" <<joint_names_[idx]<< "): rotation axis_y)" << std::endl;
             }
            RJidx_++;
        }

        else if(fabs(axis_z) == 1){
            r_joint_[RJidx_]->GetGeomInfo().SetShape(srGeometryInfo::CYLINDER);
            map<string,int>::iterator Linkidxiter=link_idx_map_.find(p_Linkidxiter->first);
            r_joint_[RJidx_]->SetParentLink(link_[Linkidxiter->second]);
            Linkidxiter=link_idx_map_.find(c_Linkidxiter->second->name);
            r_joint_[RJidx_]->SetChildLink(link_[Linkidxiter->second]);

            if(axis_z>0){
                r_joint_[RJidx_]->SetParentLinkFrame( EulerZYX(JointOff_ori, JointOff_pos) );
                r_joint_[RJidx_]->SetChildLinkFrame(EulerZYX(Vec3(),-c_Link_offset));
            }else{
                printf("[System Generator] TODO: negative z axis\n");
                r_joint_[RJidx_]->SetParentLinkFrame( EulerZYX(JointOff_ori, JointOff_pos) );
                r_joint_[RJidx_]->SetChildLinkFrame(EulerZYX(Vec3(),-c_Link_offset));
            }
            RJidx_++;
        }
        else if(axis_x*axis_y + axis_y*axis_z + axis_z*axis_x != 0){
            if (axis_x == 0) {
                double atan_zy = atan2( axis_z, axis_y);
                Vec3 axis0yz(0., 0., atan_zy - SR_PI_HALF);
                printf("(%f, %f) atan: %f\n", axis_y, axis_z, atan_zy);
                r_joint_[RJidx_]->GetGeomInfo().SetShape(srGeometryInfo::CYLINDER);
                map<string,int>::iterator Linkidxiter=
                    link_idx_map_.find(p_Linkidxiter->first);
                r_joint_[RJidx_]->SetParentLink(link_[Linkidxiter->second]);
                Linkidxiter=link_idx_map_.find(c_Linkidxiter->second->name);
                r_joint_[RJidx_]->SetChildLink(link_[Linkidxiter->second]);
                r_joint_[RJidx_]->SetParentLinkFrame(EulerZYX(axis0yz, JointOff_pos) );
                r_joint_[RJidx_]->SetChildLinkFrame(EulerZYX(axis0yz, -c_Link_offset));
                RJidx_++;
            }
            else if (axis_y == 0) {
                Vec3 axisx0z(0., atan2(axis_z, axis_x), 0.);
                r_joint_[RJidx_]->GetGeomInfo().SetShape(srGeometryInfo::CYLINDER);
                map<string,int>::iterator Linkidxiter=link_idx_map_.find(p_Linkidxiter->first);
                r_joint_[RJidx_]->SetParentLink(link_[Linkidxiter->second]);
                Linkidxiter=link_idx_map_.find(c_Linkidxiter->second->name);
                r_joint_[RJidx_]->SetChildLink(link_[Linkidxiter->second]);
                r_joint_[RJidx_]->SetParentLinkFrame(EulerZYX(axisx0z, JointOff_pos) );
                r_joint_[RJidx_]->SetChildLinkFrame(EulerZYX(axisx0z, -c_Link_offset));
                RJidx_++;
              std::cout << "Doublecheck :srSysGenerator/SystemGenerator"
                  <<"(joint param(" <<joint_names_[idx]<< "): rotation axis_y)" << std::endl;
             }
            else if (axis_z == 0) {
              Vec3 axisxy0(acos(axis_x), SR_PI_HALF, 0);
              r_joint_[RJidx_]->GetGeomInfo().SetShape(srGeometryInfo::CYLINDER);
              map<string,int>::iterator Linkidxiter=link_idx_map_.find(p_Linkidxiter->first);
              r_joint_[RJidx_]->SetParentLink(link_[Linkidxiter->second]);
              Linkidxiter=link_idx_map_.find(c_Linkidxiter->second->name);
              r_joint_[RJidx_]->SetChildLink(link_[Linkidxiter->second]);
              r_joint_[RJidx_]->SetParentLinkFrame(EulerZYX(axisxy0, JointOff_pos) );
              r_joint_[RJidx_]->SetChildLinkFrame(EulerZYX(axisxy0, -c_Link_offset));
              RJidx_++;
              std::cout << "Doublecheck :srSysGenerator/SystemGenerator"
                  <<"(joint param(" <<joint_names_[idx]<< "): rotation axis_z)" << std::endl;
            }
            else{
                std::cout << "todo : srSysGenerator/SystemGenerator(joint param - rotation axis part)" << std::endl;
                exit(0);
            }
        }
    }

    else if(Jointidxiter->second->type == dynacore::urdf::Joint::PRISMATIC){
        std::cout << "todo : srSysGenerator/SystemGenerator.cpp(joint param - prismatic part)" << std::endl;
        exit(0);
    }

    else{
        std::cout << "todo : srSysGenerator/SystemGenerator.cpp(joint param - other joint)" << std::endl;
        exit(0);
    }
}


void SystemGenerator::_SetLinkParam(int idx){
    URDFLinkMap::iterator Linkidxiter = link_map_.find(link_names_[idx]);

    string ds3D=".3ds";
    string tok1="/";
    string tok2=".";
    string* first_str = new string[32];
    string* second_str= new string[32];
    Vec3 Inertiaoffset_;
    Vec3 link_visual_rpy;
    Vec3 link_visual_xyz;

    if(Linkidxiter->second->inertial){
        Inertiaoffset_=Vec3(Linkidxiter->second->inertial->origin.position.x,
                            Linkidxiter->second->inertial->origin.position.y,
                            Linkidxiter->second->inertial->origin.position.z);
    }
    
    if(Linkidxiter->second->visual!=0){
        link_visual_xyz[0]=Linkidxiter->second->visual->origin.position.x;
        link_visual_xyz[1]=Linkidxiter->second->visual->origin.position.y;
        link_visual_xyz[2]=Linkidxiter->second->visual->origin.position.z;
        Linkidxiter->second->visual->
            origin.rotation.getRPY (link_visual_rpy[2], link_visual_rpy[1], 
                    link_visual_rpy[0]);

        // Mesh
        if((Linkidxiter->second->visual_array[0]->geometry->type)
                == dynacore::urdf::Geometry::MESH){

            boost::shared_ptr<dynacore::urdf::Mesh> mesh
                = boost::dynamic_pointer_cast<dynacore::urdf::Mesh>(
                        Linkidxiter->second->visual_array[0]->geometry );

            string ModelFileName_ = file_path_ + mesh->filename;
             //std::cout << ModelFileName_ << std::endl;
            const char* modelnamepath = ModelFileName_.c_str();
            link_[idx]->GetGeomInfo().SetMeshScale(mesh->scale.x, 
                    mesh->scale.y, 
                    mesh->scale.z);
            link_[idx]->GetGeomInfo().SetShape(srGeometryInfo::MESH);

            std::stringstream ss(mesh->filename);
            string extension;
            getline(ss, extension, '.');
            getline(ss, extension, '.');
            if(extension == "3ds")
                link_[idx]->GetGeomInfo().SetShape(srGeometryInfo::TDS);
            link_[idx]->GetGeomInfo().SetFileName(modelnamepath);
        }
        //Box
        else if((Linkidxiter->second->visual_array[0]->geometry->type)==
                dynacore::urdf::Geometry::BOX){
            boost::shared_ptr<dynacore::urdf::Box>box=boost::dynamic_pointer_cast<dynacore::urdf::Box>(Linkidxiter->second->visual_array[0]->geometry);
            link_[idx]->GetGeomInfo().SetShape(srGeometryInfo::BOX);
            link_[idx]->GetGeomInfo().SetDimension(box->dim.x,box->dim.y,box->dim.z);
        }
        else if((Linkidxiter->second->visual_array[0]->geometry->type)==
                dynacore::urdf::Geometry::CYLINDER){
            boost::shared_ptr<dynacore::urdf::Cylinder>cylinder=boost::dynamic_pointer_cast<dynacore::urdf::Cylinder>(Linkidxiter->second->visual_array[0]->geometry);
            link_[idx]->GetGeomInfo().SetShape(srGeometryInfo::CYLINDER);
            link_[idx]->GetGeomInfo().SetDimension(cylinder->radius,cylinder->length,0);
        }
        else if((Linkidxiter->second->visual_array[0]->geometry->type)==
                dynacore::urdf::Geometry::SPHERE){
            boost::shared_ptr<dynacore::urdf::Sphere>sphere=boost::dynamic_pointer_cast<dynacore::urdf::Sphere>(Linkidxiter->second->visual_array[0]->geometry);
            link_[idx]->GetGeomInfo().SetShape(srGeometryInfo::SPHERE);
            link_[idx]->GetGeomInfo().SetDimension(sphere->radius,0,0);
        }
        else if((Linkidxiter->second->visual_array[0]->geometry->type)==
                dynacore::urdf::Geometry::CAPSULE){
            boost::shared_ptr<dynacore::urdf::Capsule>capsule=
                boost::dynamic_pointer_cast<dynacore::urdf::Capsule>(
                        Linkidxiter->second->visual_array[0]->geometry);
            link_[idx]->GetGeomInfo().SetShape(srGeometryInfo::CAPSULE);
            link_[idx]->GetGeomInfo().SetDimension(capsule->radius,capsule->length,0);
        }

    }
    else{
        //link_[idx]->GetGeomInfo().SetShape(srGeometryInfo::MESH);
        //link_[idx]->GetGeomInfo().SetFileName(
                //"/Users/donghyunkim/Repository/Humanoid_2018/Simulator/SimulationModel/Mercury_Model/meshes/color_foot_R.dae");
 
        //link_[idx]->GetGeomInfo().SetShape(srGeometryInfo::TDS);
        //link_[idx]->GetGeomInfo().SetFileName(
                //"/Users/donghyunkim/Repository/Humanoid_2018/Simulator/SimulationModel/Mercury_Model/Updated_foot.3ds");
 
        link_[idx]->GetGeomInfo().SetShape(srGeometryInfo::BOX);
        link_[idx]->GetGeomInfo().SetDimension(Vec3(0.01,0.015,0.02));
    }

    if(Linkidxiter->second->inertial!=0){
        link_[idx]->GetGeomInfo().SetLocalFrame(EulerZYX(
                    link_visual_rpy+Vec3(0.0, 0.0, SR_PI_HALF),
                    //link_visual_rpy,
                    link_visual_xyz-Inertiaoffset_ ));
        //std::cout<<"link visual: "<<link_visual_xyz<<std::endl;
    }
    else {
        link_[idx]->GetGeomInfo().SetLocalFrame(EulerZYX(
                    link_visual_rpy+Vec3(0.0, 0.0, SR_PI_HALF),link_visual_xyz));
    }
}


