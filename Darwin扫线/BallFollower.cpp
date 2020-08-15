/*
 *   BallFollower.cpp
 *   ɨ�߸Ľ�
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
	m_KickBallCount = 0;//������ʱ

	m_KickTopAngle = -5.0;//����׷��Ƕ�
	m_KickRightAngle = -30.0;//׷�����Ƕ�30
	m_KickLeftAngle = 30.0;//׷��Ƕȣ�30 ��Ϊ������Ϊ��
	///////////////////////////
	m_FollowMaxFBStep = 30.0;//��󲽳�
    m_FollowMinFBStep = 5.0;//��С����
	m_FollowMaxRLTurn = 35.0;//������Ҳ���
	m_FitFBStep = 3.0;//
	m_FitMaxRLTurn = 35.0;
	m_UnitFBStep = 0.3;//ǰ�󲽳���������
	m_UnitRLTurn = 1.0;
	m_UnitRLTurn1 = 5;
	//���Ҳ�����������
	//////////////////////
	m_GoalFBStep = 0;//Ŀ��ǰ�󲽳�
	m_GoalRLTurn = 0;//Ŀ�����Ҳ���
	m_FBStep = 0;//��ʼǰ�󲽳�
	m_RLTurn = 0;//��ʼ���Ҳ���
	DEBUG_PRINT = false;//�Ƿ�ִ��
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
		//û��Ŀ��󣬱���һ�����ٶ�
		KickBall = 0;
		m_GoalFBStep = 5;
		m_GoalRLTurn = 0;
    }
    else//�����Ŀ��
    {
		m_NoBallCount = 0;//��ʼ��noball��ʱֵ		

		double pan = MotionStatus::m_CurrentJoints.GetAngle(JointData::ID_HEAD_PAN);//����ͷ������
		double pan_range = Head::GetInstance()->GetLeftLimitAngle();//�������ƽǶ�
		double pan_percent = pan / pan_range;//�ǶȰٷֱ�

		double tilt = MotionStatus::m_CurrentJoints.GetAngle(JointData::ID_HEAD_TILT);//ͷ������
		double tilt_min = Head::GetInstance()->GetBottomLimitAngle();//��ͷ������
		double tilt_range = Head::GetInstance()->GetTopLimitAngle() - tilt_min;//������-������
		double tilt_percent = (tilt - tilt_min) / tilt_range;//�Ƕ����ưٷֱ�
		if(tilt_percent < 0)
			tilt_percent = -tilt_percent;

		if(pan > m_KickRightAngle && pan < m_KickLeftAngle)//���Ƕ�������������
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
