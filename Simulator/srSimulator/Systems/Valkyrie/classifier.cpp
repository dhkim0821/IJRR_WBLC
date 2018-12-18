#include "classifier.h"


//ifstream	InputFiless;
//LinkParam test_Link;
//JointParam test_Joint;
//int flag;
//vector<LinkParam> LinkSet;
//vector<JointParam> JointSet;


// void split(const string& text, string& separators, vector<string>& words) {
//     int n = text.length();
//     int start, stop;

//      start = text.find_first_not_of(separators);
// 	    while ((start >= 0) && (start < n)) {
//       stop = text.find_first_of(separators, start);
//       if ((stop < 0) || (stop > n))
//             stop = n;
//        words.push_back(text.substr(start, stop - start));
//        start = text.find_first_not_of(separators, stop+1);
//    }
// 	}





LinkParam::LinkParam():m_IsVirtual(false)
{
	m_Inertia_element.resize(6);
	for(int i(0);i<6;i++)
		m_Inertia_element[i]=0.0;
}

LinkParam::~LinkParam()
{
	m_Inertia_element.clear();
}

void LinkParam::_setmass( double mass_ )
{
	m_mass=mass_;
}

void LinkParam::_setLidx(const int _idx)
{
	m_Lidx=_idx;
}

void LinkParam::_setName( string Name_ )
{
	m_Name=Name_;
}

void LinkParam::_setModelName( string ModelName_ )
{
	m_Model_Name=ModelName_;
}

void LinkParam::_setItertia_pos( double x_, double y_, double z_ )
{
	m_Inertia_off_pos[0]=x_;
	m_Inertia_off_pos[1]=y_;
	m_Inertia_off_pos[2]=z_;

}

void LinkParam::_setItertia_ori( double r_, double p_, double y_ )
{
	m_Inertia_off_ori[0]=r_;
	m_Inertia_off_ori[1]=p_;
	m_Inertia_off_ori[2]=y_;
}

void LinkParam::_setInertia( double Ixx_,double Iyy_,double Izz_,double Ixy_,double Ixz_,double Iyz_)
{

	m_Inertia_element[0]=Ixx_;
	m_Inertia_element[1]=Iyy_;
	m_Inertia_element[2]=Izz_;
	m_Inertia_element[3]=Ixy_;
	m_Inertia_element[4]=Ixz_;
	m_Inertia_element[5]=Iyz_;

}

void LinkParam::_setInertia_pos_idx( double xx_, int idx_ )
{
	m_Inertia_off_pos[idx_]=xx_;
}

void LinkParam::_setInertia_ori_idx( double xx_, int idx_ )
{
	m_Inertia_off_ori[idx_]=xx_;
}



void LinkParam::_setInertia_idx( double xx_,int idx_ )
{
	m_Inertia_element[idx_]=xx_;	
}


void LinkParam::_setIsVirtual(bool ISvirtual_)
{

	m_IsVirtual=ISvirtual_;
}



void LinkParam::printInfo()
{
	std::cout<<"---Link Information print----"<<endl;
	std::cout<<"Index Number : "<<m_Lidx<<endl;
	std::cout<<"Link Name : "<<m_Name<<endl;
	std::cout<<"Virtual? : "<<m_IsVirtual<<endl;
	std::cout<<"Mass : "<<m_mass<<endl;
	
	std::cout<<"Inertia offset orienatation"<<endl;
	for(int i(0);i<3;i++)
	{
		cout<<m_Inertia_off_ori[i]<<" ,";
	}
	std::cout<<std::endl;
	
	std::cout<<"Inertia offset position"<<endl;
	for(int i(0);i<3;i++)
	{
		cout<<m_Inertia_off_pos[i]<<" ,";
	}
	std::cout<<std::endl;

	std::cout<<"Inertia element"<<endl;
	for(int i(0);i<6;i++)
	{
		cout<<m_Inertia_element[i]<<" ,";
	}
	std::cout<<std::endl;
}




JointParam::JointParam()
{
	m_limits.resize(4);	//efforts/upper/lower/velocity
	
	for(int i(0);i<4;i++)
		m_limits[i]=0.0;

	m_gains.resize(2);
	for(int i(0);i<2;i++)
		m_gains[i]=0.0;
	
}

JointParam::~JointParam()
{
	m_limits.clear();
	m_gains.clear();
}

void JointParam::_setJointName( const string Name_ )
{
	m_JointName=Name_;
}

void JointParam::_setJidx(const int _idx)
{
	m_Jidx=_idx;
}

void JointParam::_setParentLinkName( const string Name_ )
{
	m_parentLink_Name=Name_;
}

void JointParam::_setChildLinkName( const string Name_ )
{
	m_childLink_Name=Name_;
}

void JointParam::_setoffset_pos_idx( double xx_, int idx_ )
{
	m_Joff_pos[idx_]=xx_;
}

void JointParam::_setoffset_ori_idx( double xx_, int idx_ )
{
	m_Joff_ori[idx_]=xx_;
}

void JointParam::_setlimits_idx( double xx_, int idx_ )
{
	m_limits[idx_]=xx_;
}

void JointParam::_setgains_idx( double xx_, int idx_ )
{

	m_gains[idx_]=xx_;
	
}


void JointParam::_setJointType(const string Type_)
{

	m_JointType=Type_;

}

void JointParam::_setaxis_idx(double xx_, int idx_)
{

	m_axis[idx_]=xx_;
}


void JointParam::printInfo()
{
	std::cout<<"---Joint Information print----"<<endl;
	std::cout<<"Joint Name : "<<m_JointName<<endl;
	std::cout<<"Joint Type : "<<m_JointType<<endl;
	std::cout<<"axis : "<<endl;
	for(int i(0);i<3;i++)
	{
		cout<<m_axis[i]<<" ,";
	}
	std::cout<<std::endl;


	std::cout<<"ParentLink Name : "<<m_parentLink_Name<<endl;
	std::cout<<"ChildLink Name : "<<m_childLink_Name<<endl;

	std::cout<<"Joint offset orienatation"<<endl;
	for(int i(0);i<3;i++)
	{
		cout<<m_Joff_ori[i]<<" ,";
	}
	std::cout<<std::endl;
	
	std::cout<<"Joint offset position"<<endl;
	for(int i(0);i<3;i++)
	{
		cout<<m_Joff_pos[i]<<" ,";
	}
	std::cout<<std::endl;


	std::cout<<"Limits"<<endl;
	for(int i(0);i<4;i++)
	{
		cout<<m_limits[i]<<" ,";
	}
	std::cout<<std::endl;

std::cout<<"Gains"<<endl;
	for(int i(0);i<2;i++)
	{
		cout<<m_gains[i]<<" ,";
	}
	std::cout<<std::endl;


}