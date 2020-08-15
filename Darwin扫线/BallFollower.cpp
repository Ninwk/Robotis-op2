/*
 *   BallFollower.cpp
 *   扫线改进
 *
 *   Author: wubinghan
 *
 */

#include <stdio.h>
#include "ImgProcess.h"
#include "MX28.h"
#include "Head.h"
#include "Action.h"
#include "Walking.h"
#include "BallFollower.h"
#include "MotionStatus.h"


using namespace Robot;


BallFollower::BallFollower()
{
	m_NoBallMaxCount = 10;
	m_NoBallCount = m_NoBallMaxCount;
	m_KickBallMaxCount = 10;
	m_KickBallCount = 0;//计数延时

	m_KickTopAngle = -5.0;//上下追随角度
	m_KickRightAngle = -30.0;//追随消角度30
	m_KickLeftAngle = 30.0;//追随角度，30 左为正，右为负
	///////////////////////////
	m_FollowMaxFBStep = 30.0;//最大步长
    m_FollowMinFBStep = 5.0;//最小步长
	m_FollowMaxRLTurn = 35.0;//最大左右步长
	m_FitFBStep = 3.0;//
	m_FitMaxRLTurn = 35.0;
	m_UnitFBStep = 0.3;//前后步长增加速率
	m_UnitRLTurn = 1.0;
	m_UnitRLTurn1 = 5;
	//左右步长增加速率
	//////////////////////
	m_GoalFBStep = 0;//目标前后步长
	m_GoalRLTurn = 0;//目标左右步长
	m_FBStep = 0;//初始前后步长
	m_RLTurn = 0;//初始左右步长
	DEBUG_PRINT = false;//是否执行
	KickBall = 0; 
}

BallFollower::~BallFollower()
{
}

void BallFollower::Process(Point2D ball_pos)
{
	if(DEBUG_PRINT == true)
		fprintf(stderr, "\r                                                                               \r");

    if(ball_pos.X == -1.0 || ball_pos.Y == -1.0)
    {
		//没有目标后，保持一定的速度
		KickBall = 0;
		m_GoalFBStep = 5;
		m_GoalRLTurn = 0;
    }
    else//如果有目标
    {
		m_NoBallCount = 0;//初始化noball延时值		

		double pan = MotionStatus::m_CurrentJoints.GetAngle(JointData::ID_HEAD_PAN);//左右头部距离
		double pan_range = Head::GetInstance()->GetLeftLimitAngle();//左右限制角度
		double pan_percent = pan / pan_range;//角度百分比

		double tilt = MotionStatus::m_CurrentJoints.GetAngle(JointData::ID_HEAD_TILT);//头部距离
		double tilt_min = Head::GetInstance()->GetBottomLimitAngle();//下头部限制
		double tilt_range = Head::GetInstance()->GetTopLimitAngle() - tilt_min;//上限制-下限制
		double tilt_percent = (tilt - tilt_min) / tilt_range;//角度限制百分比
		if(tilt_percent < 0)
			tilt_percent = -tilt_percent;

		if(pan > m_KickRightAngle && pan < m_KickLeftAngle)//当角度在左右限制内
		{
			if (tilt <= (tilt_min + MX28::RATIO_VALUE2ANGLE))
			{


				m_GoalFBStep = 5;
				m_GoalRLTurn = m_FitMaxRLTurn * pan_percent;
				if (DEBUG_PRINT == true)
					fprintf(stderr, "[FOLLOW(P:%.2f T:%.2f>%.2f]", pan, tilt, tilt_min);
			}
			else
			{
				

				m_KickBallCount = 0;
				KickBall = 0;
				m_GoalFBStep = 20;
				//if(m_GoalFBStep < m_FollowMinFBStep)
				   // m_GoalFBStep = m_FollowMinFBStep;
				m_GoalRLTurn = m_FollowMaxRLTurn * pan_percent;
			}
		}
		else
		{
			m_KickBallCount = 0;
			KickBall = 0;
			m_GoalFBStep = 0;
			m_GoalRLTurn = m_FollowMaxRLTurn * pan_percent;
			if(DEBUG_PRINT == true)
				fprintf(stderr, "[FOLLOW(P:%.2f T:%.2f>%.2f]", pan, tilt, tilt_min);
		}		
	}

	if(m_GoalFBStep == 0 && m_GoalRLTurn == 0 && m_FBStep == 0 && m_RLTurn == 0)
	{
		if (Walking::GetInstance()->IsRunning() == true)
			;//Walking::GetInstance()->Stop();
		else
		{
			if(m_KickBallCount < m_KickBallMaxCount)
				m_KickBallCount++;
		}

		if(DEBUG_PRINT == true)
			fprintf(stderr, " STOP");
	}
	else
	{
		if(DEBUG_PRINT == true)
			fprintf(stderr, " START");

		if(Walking::GetInstance()->IsRunning() == false)
		{ 
			m_FBStep = 0;
			m_RLTurn = 0;
			m_KickBallCount = 0;
			KickBall = 0;
			Walking::GetInstance()->X_MOVE_AMPLITUDE = m_FBStep;
			Walking::GetInstance()->A_MOVE_AMPLITUDE = m_RLTurn;
			Walking::GetInstance()->Start();
			
		}
		else
		{
			
			if(m_FBStep < m_GoalFBStep)
				m_FBStep += m_UnitFBStep;
			else if(m_FBStep > m_GoalFBStep)
				m_FBStep = m_GoalFBStep;
			Walking::GetInstance()->X_MOVE_AMPLITUDE = m_FBStep;

			if(m_RLTurn < m_GoalRLTurn)
				m_RLTurn += m_UnitRLTurn;
			else if(m_RLTurn > m_GoalRLTurn)
				m_RLTurn -= m_UnitRLTurn1;
			Walking::GetInstance()->A_MOVE_AMPLITUDE = m_RLTurn;

			if(DEBUG_PRINT == true)
				fprintf(stderr, " (FB:%.1f RL:%.1f)", m_FBStep, m_RLTurn);
		}
	}
}
