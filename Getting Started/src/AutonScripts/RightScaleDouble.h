/*
 * RightScaleDouble.h
 *
 *  Created on: Apr 16, 2018
 *      Author: PatriotRobotics
 */

#ifndef SRC_AUTONSCRIPTS_RIGHTSCALEDOUBLE_H_
#define SRC_AUTONSCRIPTS_RIGHTSCALEDOUBLE_H_

#include <AutonScripts/AutonScript.h>

class RightScaleDouble : public AutonScript{
private:
	double prevHeading = 0.0;
public:
	RightScaleDouble(RobotLogic *);
	virtual ~RightScaleDouble();
	void RunScript();
	void CheckFlags();
};

#endif /* SRC_AUTONSCRIPTS_RIGHTSCALEDOUBLE_H_ */
