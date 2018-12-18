#ifndef SRLIB_OBJECT
#define SRLIB_OBJECT

#ifdef WIN32
	#define _CRT_SECURE_NO_DEPRECATE 
	#define _CRT_SECURE_NO_WARNINGS 
	#define _CRT_NONSTDC_NO_DEPRECATE 

	#pragma warning ( disable:4996 )
#endif

#include "LieGroup/_array.h"
#include <stdlib.h>
#include <wchar.h>
#include <string>

// To use unicode, activate below define.
// Unicode src is not tested. Do not use.
// Or you may complete this.
//#define SR_UNICODE

/*!
	\class srObject
	\brief Base class of srLib
*/

using namespace::std;

class srObject
{
private:
	/*!
		Count the total creation of srObject.
	 */
	static unsigned int s_CountCreation;
	/*!
		Count the total number of srObject.
		This value is less than or equal to `s_CountCreate'.
	 */
	static unsigned int s_CountNumObj;
	/*!
		Contain all srObject instances' points.
	 */
	static _array<srObject*> s_Objects;

	/*!
		Every time new srObject(and its derived) created,
		add it to `s_Objects'.
	 */
	static void AddObject(srObject* );
	/*!
		Every time srObject destroyed,
		remove it from `s_Objects'.
	 */
	static void RemoveObject(srObject* );

public:	
			 srObject();
	virtual ~srObject();

	/*1
		Get total number of srObject.
	 */
	static unsigned int GetTotalNumObj();
	/*!
		Find the index of srObject whose name is `name'.
		If find failed, return `-1'.
	 */
//	static srObject* FindObject(string name);
	static srObject* FindObject(char* name);

#ifdef SR_UNICODE
protected:
	wstring m_Name;
public:
	wstring& GetName();
	void SetName(string name);
	void SetName(wstring name);
	void SetName(char* name);
	void SetName(wchar_t* name);
#else
protected:
	//string m_Name;
	char	m_Name[1024];
public:
	//string& GetName();
	char*	GetName();
	void SetName(string name);
	void SetName(char* name);
#endif
};

#endif
