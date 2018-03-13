/*
 * CenterSwitchLeft.h
 *
 *  Created on: Mar 7, 2018
 *      Author: patriotrobotics
 */

#ifndef SRC_AUTONSCRIPTS_LEFTSWITCH_H_
#define SRC_AUTONSCRIPTS_LEFTSWITCH_H_

#include <AutonScripts/AutonScript.h>

class LeftSwitch : public AutonScript{
private:
	double prevHeading = 0.0;
public:
	LeftSwitch(RobotLogic *);
	virtual ~LeftSwitch();
	void RunScript();
	void CheckFlags();
};

#endif /* SRC_AUTONSCRIPTS_LEFTSWITCH_H_ */
