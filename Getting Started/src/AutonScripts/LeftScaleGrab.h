/*
 * LeftScaleGrab.h
 *
 *  Created on: Apr 16, 2018
 *      Author: PatriotRobotics
 */

#ifndef SRC_AUTONSCRIPTS_LEFTSCALEGRAB_H_
#define SRC_AUTONSCRIPTS_LEFTSCALEGRAB_H_

#include "AutonScripts/AutonScript.h"

class LeftScaleGrab : public AutonScript{
private:
	double prevHeading = 0.0;
public:
	LeftScaleGrab(RobotLogic *);
	virtual ~LeftScaleGrab();
	void RunScript();
	void CheckFlags();
};

#endif /* SRC_AUTONSCRIPTS_LEFTSCALEGRAB_H_ */
