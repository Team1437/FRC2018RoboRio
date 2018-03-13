/*
 * RightHookSwitch.h
 *
 *  Created on: Mar 11, 2018
 *      Author: PatriotRobotics
 */

#ifndef SRC_AUTONSCRIPTS_RIGHTHOOKSWITCH_H_
#define SRC_AUTONSCRIPTS_RIGHTHOOKSWITCH_H_

#include <AutonScripts/AutonScript.h>

class RightHookSwitch : public AutonScript{
private:
	double prevHeading = 0.0;
public:
	RightHookSwitch(RobotLogic *);
	virtual ~RightHookSwitch();
	void RunScript();
	void CheckFlags();
};

#endif /* SRC_AUTONSCRIPTS_RIGHTHOOKSWITCH_H_ */
