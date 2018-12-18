#include "srDyn/srObject.h"
#include <iostream>
#include <string.h>
#include <stdio.h>

unsigned int srObject::s_CountCreation = 0;
unsigned int srObject::s_CountNumObj = 0;
_array<srObject*> srObject::s_Objects;

srObject::srObject()
{
	s_CountCreation++;
	s_CountNumObj++;

	AddObject(this);

#ifdef SR_UNICODE
	wchar_t buf[16];
	swprintf(buf, "%d", s_CountCreation);
	m_Name = std::wstring("SRObj-") + std::wstring(buf);
#else
//	char buf[16];
//	sprintf(buf, "%d", s_CountCreation);
//	m_Name = string("SRObj-") + string(buf);
#ifdef WIN32
#else
#endif
	sprintf(m_Name, "SRObj-%d", s_CountCreation);
#endif
};

srObject::~srObject()
{
	RemoveObject(this);
	s_CountNumObj--;
};

unsigned int srObject::GetTotalNumObj()
{
	return s_CountNumObj;
}

srObject* srObject::FindObject(char* name)
{
	for(int i = 0 ; i < s_Objects.get_size() ; ++i) {
		if(s_Objects[i] != NULL) {
			if(!strcmp(s_Objects[i]->GetName(), name)) {
				return s_Objects[i];
			}
		}
	}

	return NULL;
}

//srObject* srObject::FindObject(string name)
//{
//	for(int i = 0 ; i < s_Objects.get_size() ; ++i) {
//		if(s_Objects[i] != NULL) {
//			if(s_Objects[i]->GetName() == name) {
//				return s_Objects[i];
//			}
//		}
//	}
//
//	return NULL;
//}

void srObject::AddObject(srObject* obj)
{
	s_Objects.add_tail(obj);
}

void srObject::RemoveObject(srObject* obj)
{
	s_Objects.remove(obj);

//	int idx = s_Objects.find(obj);
//	if(idx != -1)
//		s_Objects[idx] = NULL;
}

#ifdef SR_UNICODE
std::wstring& srObject::GetName()
{
	return m_Name;
}

void srObject::SetName(string name)
{
	m_Name = std::wstring(name.length(), L'');
	copy(name.begin(), name.end(), m_Name.begin());
}

void srObject::SetName(wstring name)
{
	m_Name = name;
}

void srObject::SetName(char* name)
{
	std::string temp(name);
	SetName(temp);
}

void srObject::SetName(wchar_t* name)
{
	std::wstring temp(name);
	SetName(temp);
}
#else
char* srObject::GetName()
{
	return m_Name;
}

void srObject::SetName(char* name)
{
#ifdef DEBUG_
	if(FindObject(name) != NULL) {
		cout << "Warning!: The name you give is already used. Using unique name is recommended." << endl;
	}
#endif
	if(name != NULL)
		strcpy(m_Name, name);
}

void srObject::SetName(string name)
{
	SetName((char*)name.c_str());
}
//string& srObject::GetName()
//{
//	return m_Name;
//}
//
//void srObject::SetName(string name)
//{
//	if(FindObject(name) != NULL) {
//		cout << "Warning!: The name you give is already used. Using unique name is recommended." << endl;
//	}
//	m_Name = name;
//}
//
//void srObject::SetName(char* name)
//{
//	string temp(name);
//	SetName(temp);
//}
#endif

