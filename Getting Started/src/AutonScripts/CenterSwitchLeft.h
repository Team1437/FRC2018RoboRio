/*
 * CenterSwitchLeft.h
 *
 *  Created on: Mar 10, 2018
 *      Author: patriotrobotics
 */

#ifndef SRC_AUTONSCRIPTS_CENTERSWITCHLEFT_H_
#define SRC_AUTONSCRIPTS_CENTERSWITCHLEFT_H_

#include <AutonScripts/AutonScript.h>

class CenterSwitchLeft : public AutonScript{
private:
	double prevHeading = 0.0;
public:
	CenterSwitchLeft(RobotLogic *);
	virtual ~CenterSwitchLeft();
	void RunScript();
	void CheckFlags();
};

#endif /* SRC_AUTONSCRIPTS_CENTERSWITCHLEFT_H_ */
