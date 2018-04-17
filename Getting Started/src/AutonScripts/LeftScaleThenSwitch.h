/*
 * LeftScaleThenSwitch.h
 *
 *  Created on: Apr 16, 2018
 *      Author: PatriotRobotics
 */

#ifndef SRC_AUTONSCRIPTS_LEFTSCALETHENSWITCH_H_
#define SRC_AUTONSCRIPTS_LEFTSCALETHENSWITCH_H_

#include "AutonScripts/AutonScript.h"

class LeftScaleThenSwitch : public AutonScript{
private:
	double prevHeading = 0.0;
public:
	LeftScaleThenSwitch(RobotLogic *);
	virtual ~LeftScaleThenSwitch();
	void RunScript();
	void CheckFlags();
};

#endif /* SRC_AUTONSCRIPTS_LEFTSCALETHENSWITCH_H_ */
