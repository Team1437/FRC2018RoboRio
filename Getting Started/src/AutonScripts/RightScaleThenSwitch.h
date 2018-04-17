/*
 * RightScaleThenSwitch.h
 *
 *  Created on: Apr 16, 2018
 *      Author: PatriotRobotics
 */

#ifndef SRC_AUTONSCRIPTS_RIGHTSCALETHENSWITCH_H_
#define SRC_AUTONSCRIPTS_RIGHTSCALETHENSWITCH_H_

#include <AutonScripts/AutonScript.h>

class RightScaleThenSwitch : public AutonScript{
private:
	double prevHeading = 0.0;
public:
	RightScaleThenSwitch(RobotLogic *);
	virtual ~RightScaleThenSwitch();
	void RunScript();
	void CheckFlags();
};

#endif /* SRC_AUTONSCRIPTS_RIGHTSCALETHENSWITCH_H_ */
