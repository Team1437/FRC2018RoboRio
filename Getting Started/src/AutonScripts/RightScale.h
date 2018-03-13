/*
 * RightScale.h
 *
 *  Created on: Mar 11, 2018
 *      Author: PatriotRobotics
 */

#ifndef SRC_AUTONSCRIPTS_RIGHTSCALE_H_
#define SRC_AUTONSCRIPTS_RIGHTSCALE_H_

#include <AutonScripts/AutonScript.h>

class RightScale : public AutonScript{
private:
	double prevHeading = 0.0;
public:
	RightScale(RobotLogic *);
	virtual ~RightScale();
	void RunScript();
	void CheckFlags();
};

#endif /* SRC_AUTONSCRIPTS_RIGHTSCALE_H_ */
