/*
 * LeftScaleDouble.h
 *
 *  Created on: Apr 14, 2018
 *      Author: PatriotRobotics
 */

#ifndef SRC_AUTONSCRIPTS_LEFTSCALEDOUBLE_H_
#define SRC_AUTONSCRIPTS_LEFTSCALEDOUBLE_H_

#include "AutonScripts/AutonScript.h"

class LeftScaleDouble : public AutonScript{
private:
	double prevHeading = 0.0;
public:
	LeftScaleDouble(RobotLogic *);
	virtual ~LeftScaleDouble();
	void RunScript();
	void CheckFlags();
};

#endif /* SRC_AUTONSCRIPTS_LEFTSCALEDOUBLE_H_ */
