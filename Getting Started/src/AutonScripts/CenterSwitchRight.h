/*
 * CenterSwitchRight.h
 *
 *  Created on: Mar 10, 2018
 *      Author: patriotrobotics
 */

#ifndef SRC_AUTONSCRIPTS_CENTERSWITCHRIGHT_H_
#define SRC_AUTONSCRIPTS_CENTERSWITCHRIGHT_H_

#include <AutonScripts/AutonScript.h>

class CenterSwitchRight : public AutonScript{
private:
	double prevHeading = 0.0;
public:
	CenterSwitchRight(RobotLogic *);
	virtual ~CenterSwitchRight();
	void RunScript();
	void CheckFlags();
};

#endif /* SRC_AUTONSCRIPTS_CENTERSWITCHRIGHT_H_ */
