/*
 * RightSwitch.cpp
 *
 *  Created on: Mar 8, 2018
 *      Author: patriotrobotics
 */

#include <AutonScripts/RightSwitch.h>

RightSwitch::RightSwitch(RobotLogic * bot) : AutonScript(1, bot){
	int numPoints_1 = 3;
	Waypoint * points_1 = (Waypoint*)malloc(numPoints_1 * sizeof(Waypoint));
	Waypoint p11 = { 0, 0, 0 };
	Waypoint p12 = { 2, 0, 0};
	Waypoint p13 = {4, 0, 0};
	points_1[0] = p11;
	points_1[1] = p12;
	points_1[2] = p13;
	this->BuildTrajectory(points_1, numPoints_1, 0);
}

void RightSwitch::RunScript(){
	switch(stage){
	case 0: {
		bot->AutonFollowTrajectory(this->leftTrajectories[0], this->rightTrajectories[0], this->trajectoriesLength[0]);
		bot->ClawNeutralSuck();
		break;
	}
	case 1: {
		bot->AutonTurn();
		break;
	}
	case 2: {
		bot->ClawSpitFast();
		break;
	}
	case 3: {
		break;
	}
	}
}

void RightSwitch::CheckFlags(){
	switch(stage){
	case 0: {
		if(bot->leftEncoder->finished == 1 && bot->rightEncoder->finished == 1){
			stage = 1;
			bot->AutonSetBearing(90);
		}
		break;
	}
	case 1: {
		double heading = bot->pigeon->GetFusedHeading();
		if(abs(bot->angleDifference) < 10 && abs(heading-prevHeading) < 0.2){
			stage = 2;
			bot->AutonFreeEncoders();
			bot->AutonInitEncoders();
			bot->DriveOff();
			timer.Start();
		}
		this->prevHeading = heading;
		break;
	}
	case 2: {
		if(timer.Get() > 1){
			stage = 3;
			bot->ClawNeutralSuck();
		}
		break;
	}
	case 3: {
		break;
	}
	}
}

RightSwitch::~RightSwitch() {
	// TODO Auto-generated destructor stub
}

