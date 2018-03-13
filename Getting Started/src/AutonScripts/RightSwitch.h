/*
 * RightSwitch.h
 *
 *  Created on: Mar 8, 2018
 *      Author: patriotrobotics
 */

#ifndef SRC_AUTONSCRIPTS_RIGHTSWITCH_H_
#define SRC_AUTONSCRIPTS_RIGHTSWITCH_H_

#include <AutonScripts/AutonScript.h>

class RightSwitch : public AutonScript{
private:
	double prevHeading = 0.0;
public:
	RightSwitch(RobotLogic *);
	virtual ~RightSwitch();
	void RunScript();
	void CheckFlags();
};

#endif /* SRC_AUTONSCRIPTS_RIGHTSWITCH_H_ */
