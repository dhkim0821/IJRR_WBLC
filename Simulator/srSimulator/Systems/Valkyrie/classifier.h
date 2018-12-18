#include <fstream>
#include <string>
#include <vector>
#include <iostream>
#include <LieGroup/LieGroup.h>
#include <math.h>
#include <map>
#include <list>
#include <iterator>
#include <algorithm>

#define NOCHANNEL 5

using namespace std;

struct jointOffset{

	Vec3 ori;
	Vec3 pos;

};



class LinkParam{

	public:
		LinkParam();
		 ~LinkParam();
public:

	Vec3	 m_Inertia_off_ori;				//orientation Intertia offset 
	Vec3	 m_Inertia_off_pos;				//position Intertia offset 
	double	 m_mass;						// mass
	vector<double>  m_Inertia_element;			//6 element for Ixx, Iyy, Izz, Ixz, Ixz, Iyz
	bool	 m_IsVirtual;					//true if it is virtual link
	string   m_Name;					//Link name 
	string   m_Model_Name;				//3ds Moedel name 
	int      m_Lidx;

	void _setmass(double mass_);
	void _setName(string Name_);
	void _setModelName(string ModelName_);
	void _setInertia_pos_idx(double xx_, int idx_);
	void _setInertia_ori_idx(double xx_, int idx_);
	void _setItertia_pos(double x, double y, double z);
	void _setItertia_ori(double r, double p, double y);
	void _setInertia(double Ixx,double Ixy,double Ixz,double Iyy,double Iyz,double Izz);
	void _setInertia_idx(double xx_,int idx_);
	void _setIsVirtual(bool ISvirtual_);
	void _setLidx(const int _idx);


	double _getmass(){return m_mass;};
	string _getName(){return m_Name;};
	bool   _getIsVirtual(){return m_IsVirtual;};
	string _getModelName(){return m_Model_Name;};
	Vec3 _getInertia_ori(){return m_Inertia_off_ori;};
	vector<double> _getInertia(){return m_Inertia_element;};
	Vec3 _getInertia_pos(){return m_Inertia_off_pos;};
	int  _getLidx(){return m_Lidx;};
	void printInfo();

};

class JointParam{

public:
	JointParam();
	~JointParam();
public:

	Vec3	 m_Joff_ori;				//orientation Intertia offset 
	Vec3	 m_Joff_pos;				//position Intertia offset 
	Vec3	 m_axis;					//axis 	
	string   m_JointName;					//Link name 
	string	 m_parentLink_Name;
	string	 m_childLink_Name;
	string   m_JointType;
	vector<double>  m_limits;
	vector<double>  m_gains;
	int      m_Jidx;
	
	void _setJointName(const string Name_);
	void _setParentLinkName(const string Name_);
	void _setChildLinkName(const string Name_);
	void _setoffset_pos_idx(double xx_, int idx_);
	void _setoffset_ori_idx(double xx_, int idx_);
	void _setlimits_idx(double xx_, int idx_);	
	void _setgains_idx(double xx_, int idx_);	
	void _setaxis_idx(double xx_, int idx_);	
	void _setJointType(const string Name_);
	void _setJidx(const int _idx);
	void printInfo();

	string _getJointName(){return m_JointName;};
	string _getParentLinkName(){return m_parentLink_Name;}
	string _getChildLinkName(){return m_childLink_Name;}
	string _getJointType(){return m_JointType;};
	Vec3 _getJointoff_ori(){return m_Joff_ori;};
	Vec3 _getJointoff_pos(){return m_Joff_pos;};
	Vec3 _getaxis(){return m_axis;}
	int  _getJidx(){return m_Jidx;};
};


