#include "SystemGenerator.h"
#include <Utils/utilities.hpp>
#include <sstream>

#define FIX_IN_THE_AIR 0

SystemGenerator::SystemGenerator() :
  num_r_joint_(0),
  num_fixed_joint_(0),
  num_p_joint_(0),
  RJidx_(0),
  WJidx_(0){}
SystemGenerator::~SystemGenerator(){}

void SystemGenerator::BuildRobot(Vec3 location, 
        srSystem::BASELINKTYPE base_link_type, srJoint::ACTTYPE joint_type, 
        std::string filename){
    _Parsing(filename);
    _SetLinkIdx();
    _SetControlJointIdx();
    _ResizingLinkJoint();
    _SetJointType();
    _AssembleModel(location, base_link_type, joint_type);
    _SetInertia();
    num_act_joint_ = num_p_joint_ + num_r_joint_;

    _SetInitialConf();
    _SetCollision();
    _SetJointLimit();
}

void SystemGenerator::_AssembleModel(Vec3 location, srSystem::BASELINKTYPE base_link_type, srJoint::ACTTYPE joint_type){
    _SetBase(location, base_link_type);
    _SetPassiveJoint(joint_type);
    _SetLinkJoint();
}

void SystemGenerator::_ResizingLinkJoint(){
    link_.resize(link_map_.size());
    r_joint_.resize(num_r_joint_);
    p_joint_.resize(num_p_joint_);
    vp_joint_.resize(3);
    vr_joint_.resize(3);
    v_link_.resize(6);
    fixed_joint_.resize(num_fixed_joint_);

    for(int i(0); i<link_.size(); ++i){ link_[i] = new srLink(); }

    for(int i(0); i<r_joint_.size(); ++i){ r_joint_[i] = new srRevoluteJoint(); }

    for(int i(0); i<p_joint_.size(); ++i){ p_joint_[i] = new srPrismaticJoint(); }

    for(int i(0); i<vp_joint_.size(); ++i){
        vp_joint_[i] = new srPrismaticJoint();
    }
    for(int i(0); i<vr_joint_.size(); ++i){
        vr_joint_[i] = new srRevoluteJoint();
    }
    for(int i(0); i<v_link_.size(); ++i){
        v_link_[i] = new srLink();
    }
    for(int i(0); i<fixed_joint_.size(); ++i){
        fixed_joint_[i] = new srWeldJoint();
    }
}

void SystemGenerator::_SetInertia(){
    Inertia dummy = Inertia(0.0);
    for(int i(0); i<6; ++i ) v_link_[i]->SetInertia(dummy);

    for (int i(0); i<link_names_.size(); i++){
        URDFLinkMap::iterator iter = link_map_.find(link_names_[i]);
        if(iter->second->inertial){
            Inertia inertia = Inertia(iter->second->inertial->ixx,iter->second->inertial->iyy,iter->second->inertial->izz,iter->second->inertial->ixy,iter->second->inertial->iyz,iter->second->inertial->ixz);
            Vec3  offset =  Vec3(iter->second->inertial->origin.position.x,
                    iter->second->inertial->origin.position.y,
                    iter->second->inertial->origin.position.z);
            inertia.SetMass(iter->second->inertial->mass);
            link_[i]->SetInertia(inertia);
        }
        else{
            Inertia inertia = Inertia(0,0,0,0,0,0);
            link_[i]->SetInertia(inertia);
        }
    }
}

void SystemGenerator::_SetJointType(){
    for(int i(0); i < num_r_joint_; ++i){
        r_joint_[i]->SetActType(srJoint::TORQUE);
    }

    for (int i(0); i < num_p_joint_; ++i) {
        p_joint_[i]->SetActType(srJoint::TORQUE);
    }

#if (FIX_IN_THE_AIR)
    printf("Fixed in The Air\n");
    for(int i(0); i<3; ++i){
      vp_joint_[i]->SetActType(srJoint::HYBRID);
      vr_joint_[i]->SetActType(srJoint::HYBRID);
    }
#else
    printf("Floating\n");
    for(int i(0); i<3; ++i){
      vp_joint_[i]->SetActType(srJoint::TORQUE);
      vr_joint_[i]->SetActType(srJoint::TORQUE);
    }
#endif
}

void SystemGenerator::_SetLinkJoint(){
    for (int i(0); i < joint_map_.size(); ++i) {
        _SetLinkParam(i);
        _SetJointParam(i);
    }
    _SetLinkParam(joint_map_.size());
}
void SystemGenerator::_SetBase(Vec3 location, srSystem::BASELINKTYPE base_link_type){
    v_link_[0]->SetFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), location));

    this->SetBaseLink(v_link_[0]);
    this->SetBaseLinkType(base_link_type);
    this->SetSelfCollision(true);
}

void SystemGenerator::_SetPassiveJoint(srJoint::ACTTYPE joint_type){
    for (int i = 0; i < 6; ++i) {
        v_link_[i]->GetGeomInfo().SetShape(srGeometryInfo::CYLINDER);
        v_link_[i]->GetGeomInfo().SetDimension(0.001, 0.001, 0);
    }

    float Link_R = (rand()%100)*0.008f;
    float Link_G = (rand()%100)*0.011f;
    float Link_B = (rand()%100)*0.012f;
    double passive_radius(0.1001);
    double passive_length(0.1001);

    //Virtual Link
    for (int i(0); i<6; ++i){
        v_link_[i]->GetGeomInfo().SetShape(srGeometryInfo::CYLINDER);
        v_link_[i]->GetGeomInfo().SetDimension(0.001, 0.001, 0);
    }

        //v_link_[5]->GetGeomInfo().SetDimension(10.001, 0.001, 0);
    //Passive Joint (PRISMATIC)
    vp_joint_[0]->SetParentLink(v_link_[0]);
    vp_joint_[0]->SetChildLink(v_link_[1]);
    vp_joint_[0]->GetGeomInfo().SetColor(Link_R, Link_G, 0.0);
    vp_joint_[0]->GetGeomInfo().SetShape(srGeometryInfo::CYLINDER);
    vp_joint_[0]->GetGeomInfo().SetDimension(passive_radius, passive_length, 0.0);
    vp_joint_[0]->SetParentLinkFrame(EulerZYX(Vec3(0.0, SR_PI_HALF, 0.0), Vec3(0.0, 0.0, 0.0)));
    vp_joint_[0]->SetChildLinkFrame(EulerZYX(Vec3(-SR_PI_HALF, 0.0, 0.0), Vec3(0.0, 0.0, 0.0)));

    vp_joint_[1]->SetParentLink(v_link_[1]);
    vp_joint_[1]->SetChildLink(v_link_[2]);
    vp_joint_[1]->GetGeomInfo().SetColor(0.0, Link_G, Link_B);
    vp_joint_[1]->GetGeomInfo().SetShape(srGeometryInfo::CYLINDER);
    vp_joint_[1]->GetGeomInfo().SetDimension(passive_radius, passive_length, 0.0);
    vp_joint_[1]->SetParentLinkFrame(EulerZYX(Vec3(0.0, SR_PI_HALF, 0.0), Vec3(0.0, 0.0, 0.0)));
    vp_joint_[1]->SetChildLinkFrame(EulerZYX(Vec3(-SR_PI_HALF, 0.0, 0.0), Vec3(0.0, 0.0, 0.0)));

    vp_joint_[2]->SetParentLink(v_link_[2]);
    vp_joint_[2]->SetChildLink(v_link_[3]);
    vp_joint_[2]->GetGeomInfo().SetColor(Link_R, 0.0, Link_B);
    vp_joint_[2]->GetGeomInfo().SetShape(srGeometryInfo::CYLINDER);
    vp_joint_[2]->GetGeomInfo().SetDimension(passive_radius, passive_length, 0.0);
    vp_joint_[2]->SetParentLinkFrame(EulerZYX(Vec3(0.0, SR_PI_HALF, 0.0), Vec3(0.0, 0.0, 0.0)));
    vp_joint_[2]->SetChildLinkFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.0)));

    //Passive Joint (Revolute)
    vr_joint_[0]->SetParentLink(v_link_[3]);
    vr_joint_[0]->SetChildLink(v_link_[4]);
    vr_joint_[0]->GetGeomInfo().SetColor(Link_R, 0.0, 0.0);
    vr_joint_[0]->GetGeomInfo().SetShape(srGeometryInfo::CAPSULE);
    vr_joint_[0]->GetGeomInfo().SetDimension(passive_radius, passive_length, 0.0);
    vr_joint_[0]->SetParentLinkFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.0)));
    vr_joint_[0]->SetChildLinkFrame(EulerZYX(Vec3(0.0, SR_PI_HALF, 0.0), Vec3(0.0, 0.0, 0.0)));

    vr_joint_[1]->SetParentLink(v_link_[4]);
    vr_joint_[1]->SetChildLink(v_link_[5]);
    vr_joint_[1]->GetGeomInfo().SetColor(0.0, Link_G, 0.0);
    vr_joint_[1]->GetGeomInfo().SetShape(srGeometryInfo::CAPSULE);
    vr_joint_[1]->GetGeomInfo().SetDimension(passive_radius, passive_length, 0.0);
    vr_joint_[1]->SetParentLinkFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.0)));
    vr_joint_[1]->SetChildLinkFrame(EulerZYX(Vec3(0.0, 0.0, SR_PI_HALF), Vec3(0.0, 0.0, 0.0)));

    //Root
    vr_joint_[2]->SetParentLink(v_link_[5]);
    vr_joint_[2]->SetChildLink(link_[0]);
    vr_joint_[2]->GetGeomInfo().SetColor(0.0, 0.0, Link_B);
    vr_joint_[2]->GetGeomInfo().SetShape(srGeometryInfo::CAPSULE);
    vr_joint_[2]->GetGeomInfo().SetDimension(passive_radius, passive_length, 0.0);
    vr_joint_[2]->SetParentLinkFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0., 0., 0.)));


    URDFLinkMap::iterator Linkidxiter = link_map_.find(link_names_[0]);
    Vec3 c_Link_offset = Vec3(
            Linkidxiter->second->inertial->origin.position.x,
            Linkidxiter->second->inertial->origin.position.y,
            Linkidxiter->second->inertial->origin.position.z);
    vr_joint_[2]->SetChildLinkFrame(EulerZYX(Vec3(0.0,-SR_PI_HALF,SR_PI), 
                -c_Link_offset));
}

//joint_name_
void SystemGenerator::_Parsing(std::string filename){
    std::string full_path(filename);
    std::size_t found = full_path.find_last_of("/");
    file_path_ =  full_path.substr(0,found) +"/";

    std::ifstream model_file( full_path.c_str() );
    if (!model_file) {
        std::cerr << "Error opening file '" << full_path << "'." << std::endl;
        abort();
    }

    // reserve memory for the contents of the file
    std::string model_xml_string;
    model_file.seekg(0, std::ios::end);
    model_xml_string.reserve(model_file.tellg());
    model_file.seekg(0, std::ios::beg);
    model_xml_string.assign((std::istreambuf_iterator<char>(model_file)), 
            std::istreambuf_iterator<char>());
    model_file.close();

    URDFModelPtr urdf_model = dynacore::urdf::parseURDF (model_xml_string);

    link_map_ = urdf_model->links_;
    joint_map_ = urdf_model->joints_;
    URDFLinkPtr urdf_root_ptr;

    std::stack<URDFLinkPtr> link_stack;
    std::stack<int> joint_index_stack;

    link_stack.push(link_map_[(urdf_model->getRoot()->name)]);

    if (link_stack.top()->child_joints.size() > 0) {
        joint_index_stack.push(0);
    }
    while (link_stack.size() > 0) {
        URDFLinkPtr cur_link = link_stack.top();
        unsigned int joint_idx = joint_index_stack.top();
        if (joint_idx < cur_link->child_joints.size()) {
            URDFJointPtr cur_joint = cur_link->child_joints[joint_idx];
            // increment joint index
            joint_index_stack.pop();
            joint_index_stack.push (joint_idx + 1);
            link_stack.push (link_map_[cur_joint->child_link_name]);
            joint_index_stack.push(0);
            joint_names_.push_back(cur_joint->name);
        }
        else {
            link_stack.pop();
            joint_index_stack.pop();
        }
    }
}

//link_idx_map_, link_names_
void SystemGenerator::_SetLinkIdx(){
    URDFJointPtr urdf_joint = joint_map_[joint_names_[0]];
    string rootLink=urdf_joint->parent_link_name;
    link_names_.push_back(urdf_joint->parent_link_name);

    link_idx_map_.insert(make_pair(urdf_joint->parent_link_name,0));

    for(int i(0);i<joint_names_.size();i++){
        link_names_.push_back(joint_map_[joint_names_[i]]->child_link_name);
        link_idx_map_.insert(make_pair(joint_map_[joint_names_[i]]->child_link_name,i+1));

        //printf("joint name: %s\n", joint_names_[i].c_str());
        //printf("child_link name: %s\n", joint_map_[joint_names_[i]]->child_link_name.c_str());
    }

}

//r_joint_idx_map_
void SystemGenerator::_SetControlJointIdx(){
    URDFJointMap::iterator Jointidxiter;

    for(int i(0); i<joint_names_.size(); ++i){
        Jointidxiter = joint_map_.find(joint_names_[i]);

        if(Jointidxiter->second->type == dynacore::urdf::Joint::REVOLUTE || Jointidxiter->second->type == dynacore::urdf::Joint::CONTINUOUS){
            r_joint_idx_map_.insert( make_pair(joint_names_[i], num_r_joint_));
            ++num_r_joint_;
        }
        else if(Jointidxiter->second->type == dynacore::urdf::Joint::PRISMATIC){
            p_joint_idx_map_.insert( make_pair(joint_names_[i], num_p_joint_));
            ++num_p_joint_;
        }
        else if(Jointidxiter->second->type == dynacore::urdf::Joint::FIXED){
            fixed_joint_idx_map_.insert( make_pair(joint_names_[i], num_fixed_joint_));
            ++num_fixed_joint_;
        }
        else{
            std::cout << "todo : srSysGenerator/SystemGenerator.cpp" << std::endl;
            exit(0);
        }
    }
}

void SystemGenerator::_PrintLinkJointNum() {
    std::cout << "link : " << link_idx_map_.size() << std::endl;
    std::cout <<"fixed joint : " << num_fixed_joint_ << std::endl;
    std::cout << "revolute joint : " << num_r_joint_ << std::endl;
    std::cout << "prismatic joint : " << num_p_joint_ << std::endl;
}
void SystemGenerator::_PrintControlJoint(){
    std::map<std::string, int>::iterator iter = r_joint_idx_map_.begin();
    std::cout << "Revolute Joint" << std::endl;
    for (; iter != r_joint_idx_map_.end(); ++iter) {
        std::cout << "Index : " << iter->second << "    Name : " << iter->first << std::endl;
    }
    std::cout<<"============================================"<<std::endl;

    std::map<std::string, int>::iterator iter_ = p_joint_idx_map_.begin();
    std::cout << "Prismatic Joint" << std::endl;
    for (; iter != p_joint_idx_map_.end(); ++iter) {
        std::cout << "Index : " << iter->second << "    Name : " << iter->first << std::endl;
    }
    std::cout<<"============================================"<<std::endl;

    std::map<std::string, int>::iterator iter__ = fixed_joint_idx_map_.begin();
    std::cout << "Fixed Joint" << std::endl;
    for (; iter != fixed_joint_idx_map_.end(); ++iter) {
        std::cout << "Index : " << iter->second << "    Name : " << iter->first << std::endl;
    }
    std::cout<<"============================================"<<std::endl;
}

void SystemGenerator::_PrintLink(){
    URDFLinkMap::iterator iter = link_map_.begin();
    int idx(0);
    for (; iter != link_map_.end(); ++iter) {
        std::cout << idx++ << "th link : " << iter->second->name << std::endl;
    }
}

void SystemGenerator::_PrintJoint(){
    URDFJointMap::iterator iter = joint_map_.begin();
    int idx(0);
    for (; iter != joint_map_.end(); ++iter){
        std::cout << idx++ << "th joint : " << iter->first <<std::endl;
    }

}

void SystemGenerator::_SplitString(std::string* str_array, std::string strTarget, std::string strTok) {
     int nCutPos = 0;
    int nIndex = 0;
    while ((nCutPos = strTarget.find_first_of(strTok)) != strTarget.npos){
      if (nCutPos > 0){
        str_array[nIndex++] = strTarget.substr(0, nCutPos);
      }
      strTarget = strTarget.substr(nCutPos+1);
    }
    if(strTarget.length() > 0){
      str_array[nIndex++] = strTarget.substr(0, nCutPos);
    }
}
