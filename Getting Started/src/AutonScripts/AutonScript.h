/*
 * AutonScript.h
 *
 *  Created on: Mar 6, 2018
 *      Author: PatriotRobotics
 */

#ifndef SRC_AUTONSCRIPTS_AUTONSCRIPT_H_
#define SRC_AUTONSCRIPTS_AUTONSCRIPT_H_
#include <RobotLogic.h>
#include <Timer.h>
#include <cmath>

class AutonScript {
public:
	AutonScript(int, RobotLogic *);
	virtual ~AutonScript();
	virtual void RunScript()=0;
	virtual void CheckFlags()=0;
	void SetBot(RobotLogic *);
	void BuildTrajectory(Waypoint *, int, int);

protected:
	int stage = 0;
	RobotLogic * bot = NULL;
	frc::Timer timer;

	// POMONA SETTINGS
	//double armRaiseMultiplier = 4.0;
	//double armLowerMultiplier = 3.0;
	double armRaiseMultiplier = 8.0;
	double armLowerMultiplier = 6.0;
	double armEndingRangeMultiplier = 1.1;

	Segment** trajectories;
	Segment** leftTrajectories;
	Segment** rightTrajectories;
	int* trajectoriesLength;
};

#endif /* SRC_AUTONSCRIPTS_AUTONSCRIPT_H_ */
