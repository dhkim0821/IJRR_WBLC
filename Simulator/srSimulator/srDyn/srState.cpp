#include "srDyn/srState.h"

srState::srState()
{
}

//////////////////////////////////////////////////////////////////////////
// Revolute Joint State
srRevoluteState::srRevoluteState()
{
	m_rValue[0] = 0.0;
	m_rValue[1] = 0.0;
	m_rValue[2] = 0.0;
	m_rValue[3] = 0.0;
	m_rCommand  = 0.0;

	m_rPosErrVel= 0.0;
	m_rDelVel	= 0.0;
	m_rImp		= 0.0;

	m_InitValue[0] = 0.0;
	m_InitValue[1] = 0.0;
	m_InitValue[2] = 0.0;
	m_InitValue[3] = 0.0;

	m_History.clear();
}

srRevoluteState::srRevoluteState(srRevoluteState& s)
{
	(*this) = s;

	m_InitValue[0] = 0.0;
	m_InitValue[1] = 0.0;
	m_InitValue[2] = 0.0;
	m_InitValue[3] = 0.0;

	m_History.clear();
}

const srRevoluteState & srRevoluteState::operator = (const srRevoluteState &_rv)
{
	m_rValue[0] = _rv.m_rValue[0];
	m_rValue[1] = _rv.m_rValue[1];
	m_rValue[2] = _rv.m_rValue[2];
	m_rValue[3] = _rv.m_rValue[3];
	m_rCommand = _rv.m_rCommand;

	return *this;
}

void srRevoluteState::UpdatePosition(sr_real& t)
{
	m_rValue[0] += t * m_rValue[1];
}

void srRevoluteState::UpdateVelocity(sr_real& t)
{
	m_rValue[1] += t * m_rValue[2];
}

void srRevoluteState::UpdateVelWithDelVel()
{
	m_rValue[1] += m_rDelVel;
}

void srRevoluteState::UpdateAccWithDelVel(sr_real& fps)	// fps == reciprocal of time step
{
	m_rValue[2] += fps * m_rDelVel;
}

void srRevoluteState::UpdateTorqueWithImp(sr_real& fps)
{
	m_rValue[3] += fps * m_rImp;
}

void srRevoluteState::UpdatePosWithPosErrVel(sr_real& t)
{
	m_rValue[0] += t * m_rPosErrVel;
}

void srRevoluteState::UpdatePosErrVelWithDelVel()
{
	m_rPosErrVel += m_rDelVel;
}

void srRevoluteState::ResetPosErrVel()
{
	m_rPosErrVel = 0.0;
}

void srRevoluteState::UpdatePosWithCorrection(sr_real& t)
{
	m_rValue[0] += t * (m_rValue[1] + m_rPosErrVel);
}

void srRevoluteState::ResetConstraintImpulse()
{
	m_rImp = 0.0;
}

void srRevoluteState::ResetCommand()
{
	m_rCommand = 0.0;
}

void srRevoluteState::ResetVel()
{
	m_rValue[1] = 0.0;
}

void srRevoluteState::ResetAcc()
{
	m_rValue[2] = 0.0;
}

void srRevoluteState::ClearHistory()
{
	m_History.clear();
}

void srRevoluteState::PushState()
{
	float tmp = (float)m_rValue[0];
	m_History.add_tail(tmp);
}

void srRevoluteState::PopState(int index)
{
	if(m_History.get_size() == 0)
		return;

	if(index >= m_History.get_size() || index < 0)
		index = m_History.get_size() - 1;

	m_rValue[0] = m_History[index];
}

void srRevoluteState::BackupInitState()
{
	m_InitValue[0] = (float)m_rValue[0];
	m_InitValue[1] = (float)m_rValue[1];
	m_InitValue[2] = (float)m_rValue[2];
	m_InitValue[3] = (float)m_rValue[3];
}

void srRevoluteState::RestoreInitState()
{
	m_rValue[0] = m_InitValue[0];
	m_rValue[1] = m_InitValue[1];
	m_rValue[2] = m_InitValue[2];
	m_rValue[3] = m_InitValue[3];
}


//////////////////////////////////////////////////////////////////////////
// Universal Joint State
srUniversalState::srUniversalState()
{
	m_rValue[0][0] = 0.0;
	m_rValue[0][1] = 0.0;
	m_rValue[0][2] = 0.0;
	m_rValue[0][3] = 0.0;

	m_rValue[1][0] = 0.0;
	m_rValue[1][1] = 0.0;
	m_rValue[1][2] = 0.0;
	m_rValue[1][3] = 0.0;

	m_rCommand[0] = 0.0;
	m_rCommand[1] = 0.0;

	m_rPosErrVel[0] = 0;
	m_rPosErrVel[1] = 0;
	m_rDelVel[0] = 0;
	m_rDelVel[1] = 0;

	m_rImp[0] = 0.0;
	m_rImp[1] = 0.0;

	//
	m_InitValue[0][0] = 0.0f;
	m_InitValue[0][1] = 0.0f;
	m_InitValue[0][2] = 0.0f;
	m_InitValue[0][3] = 0.0f;

	m_InitValue[1][0] = 0.0f;
	m_InitValue[1][1] = 0.0f;
	m_InitValue[1][2] = 0.0f;
	m_InitValue[1][3] = 0.0f;

	m_History.clear();
}

srUniversalState::srUniversalState(srUniversalState& s)
{
	(*this) = s;

	m_InitValue[0][0] = 0.0f;
	m_InitValue[0][1] = 0.0f;
	m_InitValue[0][2] = 0.0f;
	m_InitValue[0][3] = 0.0f;

	m_InitValue[1][0] = 0.0f;
	m_InitValue[1][1] = 0.0f;
	m_InitValue[1][2] = 0.0f;
	m_InitValue[1][3] = 0.0f;

	m_History.clear();
}

const srUniversalState& srUniversalState::operator = (const srUniversalState &_rv)
{
	m_rValue[0][0] = _rv.m_rValue[0][0];
	m_rValue[0][1] = _rv.m_rValue[0][1];
	m_rValue[0][2] = _rv.m_rValue[0][2];
	m_rValue[0][3] = _rv.m_rValue[0][3];

	m_rValue[1][0] = _rv.m_rValue[1][0];
	m_rValue[1][1] = _rv.m_rValue[1][1];
	m_rValue[1][2] = _rv.m_rValue[1][2];
	m_rValue[1][3] = _rv.m_rValue[1][3];

	m_rCommand[0] = _rv.m_rCommand[0];
	m_rCommand[1] = _rv.m_rCommand[1];

	return *this;
}

void srUniversalState::UpdatePosition(sr_real & t)
{
	m_rValue[0][0] += t * m_rValue[0][1];
	m_rValue[1][0] += t * m_rValue[1][1];
}

void srUniversalState::UpdateVelocity(sr_real & t)
{
	m_rValue[0][1] += t * m_rValue[0][2];
	m_rValue[1][1] += t * m_rValue[1][2];
}

void srUniversalState::UpdateVelWithDelVel()
{
	m_rValue[0][1] += m_rDelVel[0];
	m_rValue[1][1] += m_rDelVel[1];
}

void srUniversalState::UpdateAccWithDelVel(sr_real & fps)
{
	m_rValue[0][2] += fps * m_rDelVel[0];
	m_rValue[1][2] += fps * m_rDelVel[1];
}

void srUniversalState::UpdateTorqueWithImp(sr_real & fps)
{
	m_rValue[0][3] += fps * m_rImp[0];
	m_rValue[1][3] += fps * m_rImp[1];
}

void srUniversalState::UpdatePosWithPosErrVel(sr_real & t)
{
	m_rValue[0][0] += t * m_rPosErrVel[0];
	m_rValue[1][0] += t * m_rPosErrVel[1];
}

void srUniversalState::UpdatePosErrVelWithDelVel()
{
	m_rPosErrVel[0] += m_rDelVel[0];
	m_rPosErrVel[1] += m_rDelVel[1];
}

void srUniversalState::ResetPosErrVel()
{
	m_rPosErrVel[0] = 0.0;
	m_rPosErrVel[1] = 0.0;
}

void srUniversalState::UpdatePosWithCorrection(sr_real & t)
{
	m_rValue[0][0] += t * (m_rValue[0][1] + m_rPosErrVel[0]);
	m_rValue[1][0] += t * (m_rValue[1][1] + m_rPosErrVel[1]);
}

void srUniversalState::ResetConstraintImpulse()
{
	m_rImp[0] = 0.0;
	m_rImp[1] = 0.0;
}

void srUniversalState::ResetCommand()
{
	m_rCommand[0] = 0.0;
	m_rCommand[1] = 0.0;
}

void srUniversalState::ResetVel()
{
	m_rValue[0][1] = 0.0;
	m_rValue[1][1] = 0.0;
}

void srUniversalState::ResetAcc()
{
	m_rValue[0][2] = 0.0;
	m_rValue[1][2] = 0.0;
}

void srUniversalState::ClearHistory()
{
	m_History.clear();
}

void srUniversalState::PushState()
{
	_state stateTemp;
	stateTemp.val[0] = (float)m_rValue[0][0];
	stateTemp.val[1] = (float)m_rValue[1][0];

	m_History.add_tail(stateTemp);
}

void srUniversalState::PopState(int index)
{
	if(m_History.get_size() == 0)
		return;

	if(index >= m_History.get_size() || index < 0)
		index = m_History.get_size() - 1;

	m_rValue[0][0] = m_History[index].val[0];
	m_rValue[1][0] = m_History[index].val[1];
}

void srUniversalState::BackupInitState()
{
	m_InitValue[0][0] = (float)m_rValue[0][0];
	m_InitValue[0][1] = (float)m_rValue[0][1];
	m_InitValue[0][2] = (float)m_rValue[0][2];
	m_InitValue[0][3] = (float)m_rValue[0][3];

	m_InitValue[1][0] = (float)m_rValue[1][0];
	m_InitValue[1][1] = (float)m_rValue[1][1];
	m_InitValue[1][2] = (float)m_rValue[1][2];
	m_InitValue[1][3] = (float)m_rValue[1][3];
}

void srUniversalState::RestoreInitState()
{
	m_rValue[0][0] = m_InitValue[0][0];
	m_rValue[0][1] = m_InitValue[0][1];
	m_rValue[0][2] = m_InitValue[0][2];
	m_rValue[0][3] = m_InitValue[0][3];

	m_rValue[1][0] = m_InitValue[1][0];
	m_rValue[1][1] = m_InitValue[1][1];
	m_rValue[1][2] = m_InitValue[1][2];
	m_rValue[1][3] = m_InitValue[1][3];
}

//////////////////////////////////////////////////////////////////////////
// Ball Joint State
srBallState::srBallState()
{
	m_SO3Pos = SO3();
	m_Vel = Vec3(0.0);
	m_Acc = Vec3(0.0);
	m_Torque = Vec3(0.0);

	m_Command = Vec3(0.0);
	m_EulerAngle = Vec3(0.0);

	m_PosErrVel = Vec3(0.0);
	m_DelVel = Vec3(0.0);
	m_Imp = Vec3(0.0);


	//
	m_InitPos = SO3();
	m_InitVel = Vec3(0.0);
	m_InitAcc = Vec3(0.0);
	m_InitTorque = Vec3(0.0);

	m_History.clear();
}

srBallState::srBallState(srBallState& s)
{
	(*this) = s;

	m_InitPos = SO3();
	m_InitVel = Vec3(0);
	m_InitAcc = Vec3(0);
	m_InitTorque = Vec3(0);

	m_History.clear();
}

const srBallState& srBallState::operator = (const srBallState &_rv)
{
	m_EulerAngle = _rv.m_EulerAngle;
	m_SO3Pos = _rv.m_SO3Pos;
	m_Vel = _rv.m_Vel;
	m_Acc = _rv.m_Acc;
	m_Torque = _rv.m_Torque;
	m_Command = _rv.m_Command;

	return *this;
}

void srBallState::UpdatePosition(sr_real & t)
{
	m_SO3Pos = m_SO3Pos * Exp(t * m_Vel);
}

void srBallState::UpdateVelocity(sr_real & t)
{
	m_Vel += t * m_Acc;
}

void srBallState::UpdateVelWithDelVel()
{
	m_Vel += m_DelVel;
}

void srBallState::UpdateAccWithDelVel(sr_real & fps)
{
	m_Acc += fps * m_DelVel;
}

void srBallState::UpdateTorqueWithImp(sr_real & fps)
{
	m_Torque += fps * m_Imp;
}

void srBallState::UpdatePosWithPosErrVel(sr_real & t)
{
	m_SO3Pos = m_SO3Pos * Exp(t * m_PosErrVel);
}

void srBallState::UpdatePosErrVelWithDelVel()
{
	m_PosErrVel += m_DelVel;
}

void srBallState::ResetPosErrVel()
{
	m_PosErrVel = 0.0;
}

void srBallState::UpdatePosWithCorrection(sr_real & t)
{
	m_SO3Pos = m_SO3Pos*Exp(t * (m_Vel + m_PosErrVel));
}

void srBallState::ResetConstraintImpulse()
{
	m_Imp = 0.0;
}

void srBallState::ResetCommand()
{
	m_Command = 0.0;
}

void srBallState::ResetVel()
{
	m_Vel = 0.0;
}

void srBallState::ResetAcc()
{
	m_Acc = 0.0;
}

void srBallState::ClearHistory()
{
	m_History.clear();
}

void srBallState::PushState()
{
	m_History.add_tail(m_SO3Pos);
}

void srBallState::PopState(int index)
{
	if(m_History.get_size() == 0)
		return;

	if(index >= m_History.get_size() || index < 0)
		index = m_History.get_size() - 1;

	m_SO3Pos = m_History[index];
}

void srBallState::BackupInitState()
{
	m_InitPos = m_SO3Pos;
	m_InitVel = m_Vel;
	m_InitAcc = m_Acc;
	m_InitTorque = m_Torque;
}

void srBallState::RestoreInitState()
{
	m_SO3Pos = m_InitPos;
	m_Vel = m_InitVel;
	m_Acc = m_InitAcc;
	m_Torque = m_InitTorque;
}

