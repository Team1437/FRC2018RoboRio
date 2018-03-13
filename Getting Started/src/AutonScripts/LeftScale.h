/*
 * LeftScale.h
 *
 *  Created on: Mar 6, 2018
 *      Author: PatriotRobotics
 */

#ifndef SRC_AUTONSCRIPTS_LEFTSCALE_H_
#define SRC_AUTONSCRIPTS_LEFTSCALE_H_

#include <AutonScripts/AutonScript.h>

class LeftScale : public AutonScript{
private:
	double prevHeading = 0.0;
public:
	LeftScale(RobotLogic *);
	virtual ~LeftScale();
	void RunScript();
	void CheckFlags();
};

#endif /* SRC_AUTONSCRIPTS_LEFTSCALE_H_ */
