#ifndef SYSTEMGENERATOR_H
#define SYSTEMGENERATOR_H

#include <urdf/model.h>
#include <urdf/urdf_parser.h>
#include <srConfiguration.h>
#include <iostream>
#include <fstream>
#include <stack>
#include "srDyn/srSpace.h"

typedef boost::shared_ptr<dynacore::urdf::Link> URDFLinkPtr;
typedef boost::shared_ptr<dynacore::urdf::Joint> URDFJointPtr;
typedef boost::shared_ptr<dynacore::urdf::ModelInterface> URDFModelPtr;
typedef std::vector<URDFLinkPtr> URDFLinkVector;
typedef std::vector<URDFJointPtr> URDFJointVector;
typedef std::map<std::string, URDFLinkPtr> URDFLinkMap;
typedef std::map<std::string, URDFJointPtr> URDFJointMap;

class SystemGenerator: public srSystem
{
 protected:

  void _Parsing(std::string file);
  void _SetLinkIdx();
  void _SetControlJointIdx();

  void _AssembleModel(Vec3 location, srSystem::BASELINKTYPE base_link_type, srJoint::ACTTYPE joint_type);
  void _SetPassiveJoint(srJoint::ACTTYPE joint_type);
  void _ResizingLinkJoint();
  void _SetBase(Vec3 location, srSystem::BASELINKTYPE base_link_type);
  void _SetLinkJoint();
  void _SetLinkParam(int i);
  void _SetJointParam(int i);
  void _SetInertia();
  void _SetJointType();
  void _SplitString(std::string* str_array, std::string strTarget, std::string strTok);
  void _SplitStringLast(std::string* str_array, std::string strTarget, std::string strTok);

  URDFLinkMap link_map_;
  URDFJointMap joint_map_;
  std::vector<std::string> joint_names_;
  std::vector<std::string> link_names_;
  int num_r_joint_;
  int num_p_joint_;
  int num_fixed_joint_;
  int RJidx_;
  int WJidx_;

  void _PrintLink();
  void _PrintJoint();
  void _PrintControlJoint();
  void _PrintLinkJointNum();

  virtual void _SetCollision() = 0;
  virtual void _SetInitialConf() = 0;
  virtual void _SetJointLimit() = 0;

  std::string file_path_;
 
 public:
  SystemGenerator();
  virtual ~SystemGenerator();

  void BuildRobot(Vec3 location, srSystem::BASELINKTYPE base_link_type, srJoint::ACTTYPE joint_type, std::string filename);

  std::vector<srLink*> link_;
  std::vector<srRevoluteJoint*> r_joint_;
  std::vector<srPrismaticJoint*> p_joint_;
  std::vector<srWeldJoint*> fixed_joint_;
  std::vector<srPrismaticJoint*> vp_joint_;
  std::vector<srRevoluteJoint*> vr_joint_;
  std::vector<srLink*> v_link_;
  std::map<std::string, int> fixed_joint_idx_map_;
  std::map<std::string, int> p_joint_idx_map_;
  std::map<std::string, int> r_joint_idx_map_;
  std::map<std::string, int> link_idx_map_;

	int num_act_joint_;
};

#endif /* SYSTEMGENERATOR_H */

