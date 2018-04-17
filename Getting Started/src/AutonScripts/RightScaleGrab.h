/*
 * RightScaleGrab.h
 *
 *  Created on: Apr 16, 2018
 *      Author: PatriotRobotics
 */

#ifndef SRC_AUTONSCRIPTS_RIGHTSCALEGRAB_H_
#define SRC_AUTONSCRIPTS_RIGHTSCALEGRAB_H_

#include <AutonScripts/AutonScript.h>

class RightScaleGrab : public AutonScript{
private:
	double prevHeading = 0.0;
public:
	RightScaleGrab(RobotLogic *);
	virtual ~RightScaleGrab();
	void RunScript();
	void CheckFlags();
};

#endif /* SRC_AUTONSCRIPTS_RIGHTSCALEGRAB_H_ */
