/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.							  */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/
#pragma once

#include "RobotBase.h"

class OurSampleRobot : public RobotBase
{
public:
	OurSampleRobot();
	virtual ~OurSampleRobot() {}
	virtual void RobotInit();
	virtual void Disabled();
	virtual void Autonomous();
	virtual void OperatorControl();
	virtual void Test();
	virtual void RobotMain();
	void StartCompetition();

private:
	bool m_robotMainOverridden;
};
