/*
 * LeftHookSwitch.h
 *
 *  Created on: Mar 11, 2018
 *      Author: PatriotRobotics
 */

#ifndef SRC_AUTONSCRIPTS_LEFTHOOKSWITCH_H_
#define SRC_AUTONSCRIPTS_LEFTHOOKSWITCH_H_

#include <AutonScripts/AutonScript.h>

class LeftHookSwitch : public AutonScript{
private:
	double prevHeading = 0.0;
public:
	LeftHookSwitch(RobotLogic *);
	virtual ~LeftHookSwitch();
	void RunScript();
	void CheckFlags();
};

#endif /* SRC_AUTONSCRIPTS_LEFTHOOKSWITCH_H_ */
