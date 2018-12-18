#include "srDyn/srGeometryInfo.h"
#include <memory.h>

srGeometryInfo::srGeometryInfo()
{
	m_Type = BOX;

	m_Dimension[0] = 0.1f;
	m_Dimension[1] = 0.1f;
	m_Dimension[2] = 0.1f;

	m_Color[0] = 0.7f;
	m_Color[1] = 0.7f;
	m_Color[2] = 0.7f;
	m_Color[3] = 1.0f;

	m_BoundingRadius = 0.086602540378f;

	memset(m_Filename, 0, 2048);

	m_LocalFrame = SE3(0.0);
}


float& srGeometryInfo::GetBoundingRadius()
{
	return m_BoundingRadius;
}

void srGeometryInfo::UpdateBoundingRadius()
{
	switch(m_Type) {
		case SPHERE:
			m_BoundingRadius = 0.5f * (float)GetDimension()[0];
			break;
		case BOX:
			m_BoundingRadius = 0.5f * (float)sqrt( GetDimension()[0] * GetDimension()[0]
			+ GetDimension()[1] * GetDimension()[1]
			+ GetDimension()[2] * GetDimension()[2]);
			break;
		case CAPSULE:
			m_BoundingRadius = 0.5f * (float)(GetDimension()[0] +  GetDimension()[1]);
			break;
		case CYLINDER:
			m_BoundingRadius = 0.5f * (float)sqrt(GetDimension()[0] * GetDimension()[0]
			+ GetDimension()[1] * GetDimension()[1]);
			break;
		case USER:
		default:
			break;
	}
}

void srGeometryInfo::SetShape(SHAPETYPE t)
{
	m_Type = t;
}

srGeometryInfo::SHAPETYPE srGeometryInfo::GetShape()
{
	return m_Type;
}

void srGeometryInfo::SetFileName(const char* name)
{
//	delete m_Filename;

//	int len = int(strlen(name))+3;
//	m_Filename = new char(len);
	strcpy(m_Filename, name);
}

char* srGeometryInfo::GetFileName()
{
	return m_Filename;
}

void srGeometryInfo::SetLocalFrame(SE3 T)
{
	m_LocalFrame = T;
}

SE3& srGeometryInfo::GetLocalFrame()
{
	return m_LocalFrame;
}

void srGeometryInfo::SetDimension(Vec3 v)
{
	m_Dimension[0] = (float)v[0];
	m_Dimension[1] = (float)v[1];
	m_Dimension[2] = (float)v[2];
}

void srGeometryInfo::SetDimension(float* v)
{
	m_Dimension[0] = v[0];
	m_Dimension[1] = v[1];
	m_Dimension[2] = v[2];
}

void srGeometryInfo::SetDimension(double w, double h, double d)
{
	m_Dimension[0] = w;
	m_Dimension[1] = h;
	m_Dimension[2] = d;
}

void srGeometryInfo::SetMeshScale(double x, double y, double z){
    m_Mesh_scale[0] = x;
    m_Mesh_scale[1] = y;
    m_Mesh_scale[2] = z;
}
Vec3 & srGeometryInfo::GetMeshScale(){
    return m_Mesh_scale;
}
//void srGeometryInfo::SetDiameter(float d)
//{
//	m_Dimension[0] = d;
//}
//
//void srGeometryInfo::SetHeight(float h)
//{
//	m_Dimension[1] = h;
//}
//
//void srGeometryInfo::SetWidth(float w)
//{
//	m_Dimension[0] = w;
//}
//
//void srGeometryInfo::SetDepth(float d)
//{
//	m_Dimension[2] = d;
//}

Vec3& srGeometryInfo::GetDimension()
{
	return m_Dimension;
}

void srGeometryInfo::GetDimension(float& a, float& b, float& c)
{
	a = (float)m_Dimension[0];
	b = (float)m_Dimension[1];
	c = (float)m_Dimension[2];
}

void srGeometryInfo::GetDimension(sr_real& a, sr_real& b, sr_real& c)
{
	a = m_Dimension[0];
	b = m_Dimension[1];
	c = m_Dimension[2];
}

//float& srGeometryInfo::GetDiameter()
//{
//	return (float&)m_Dimension[0];
//}
//
//float& srGeometryInfo::GetHeight()
//{
//	return (float&)m_Dimension[1];
//}
//
//float& srGeometryInfo::GetDepth()
//{
//	return (float&)m_Dimension[2];
//}

void srGeometryInfo::SetColor(float* c)
{
	m_Color[0] = c[0];
	m_Color[1] = c[1];
	m_Color[2] = c[2];
	m_Color[3] = c[3];
}

void srGeometryInfo::SetColor(float r, float g, float b, float a)
{
	m_Color[0] = r;
	m_Color[1] = g;
	m_Color[2] = b;
	m_Color[3] = a;
}

float* srGeometryInfo::GetColor()
{
	return m_Color;
}

void srGeometryInfo::GetColor(float& r, float& g, float& b, float& a)
{
	r = m_Color[0];
	g = m_Color[1];
	b = m_Color[2];
	a = m_Color[3];
}
