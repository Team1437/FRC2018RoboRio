/*
 * CenterSwitchLeft.cpp
 *
 *  Created on: Mar 10, 2018
 *      Author: patriotrobotics
 */

#include <AutonScripts/CenterSwitchLeft.h>

CenterSwitchLeft::CenterSwitchLeft(RobotLogic * bot) : AutonScript(1, bot){
	int numPoints_1 = 3;
	Waypoint * points_1 = (Waypoint*)malloc(numPoints_1 * sizeof(Waypoint));
	Waypoint p11 ={0, 0, 0};
	Waypoint p12 = {1.5, 1.75, d2r(45)};
	Waypoint p13 = {3.4, 3, 0};
	points_1[0] = p11;
	points_1[1] = p12;
	points_1[2] = p13;

	/*int numPoints_2 = 4;
	Waypoint * points_2 = (Waypoint*)malloc(numPoints_2 * sizeof(Waypoint));
	Waypoint p21 ={0, 0, 0};
	Waypoint p22 = {0.3, 0, 0};
	Waypoint p23 = {0.60, 0.75, d2r(-15)};
	Waypoint p24 = {0.7, 1.25, 0};
	points_2[0] = p21;
	points_2[1] = p22;
	points_2[2] = p23;
	points_2[3] = p24;

	int numPoints_3 = 3;
	Waypoint * points_3 = (Waypoint*)malloc(numPoints_3 * sizeof(Waypoint));
	Waypoint p31 ={0, 0, 0};
	Waypoint p32 = {2, 0, 0};
	Waypoint p33 = {8, 0, 0};
	points_3[0] = p31;
	points_3[1] = p32;
	points_3[2] = p33;*/

	this->BuildTrajectory(points_1, numPoints_1, 0);
	/*this->BuildTrajectory(points_2, numPoints_2, 1);
	this->BuildTrajectory(points_3, numPoints_3, 2);*/
}

void CenterSwitchLeft::RunScript(){
	switch(stage){
	case 0: {
		bot->AutonFollowTrajectory(this->leftTrajectories[0], this->rightTrajectories[0], this->trajectoriesLength[0]);
		bot->ClawNeutralSuck();
		break;
	}
	case 1: {
		bot->AutonMoveArm(PID_AUTO_TARGET, armRaiseMultiplier);
		bot->ClawWristExtend();
		break;
	}
	case 2: {
		bot->ClawOpen();
		break;
	}
	/*case 3:{
		bot->AutonFollowReverseTrajectory(this->leftTrajectories[1], this->rightTrajectories[1], this->trajectoriesLength[1]);
		break;
	}
	case 4:{
		bot->AutonTurn();
		break;
	}
	case 5:{
		bot->AutonFollowTrajectory(this->leftTrajectories[2], this->rightTrajectories[2], this->trajectoriesLength[2]);
		break;
	}
	case 6:{
		break;
	}*/
	}
}

void CenterSwitchLeft::CheckFlags(){
	switch(stage){
	case 0: {
		if(bot->leftEncoder->finished == 1 && bot->rightEncoder->finished == 1){
			stage = 1;
			bot->DriveOff();
			timer.Start();
		}
		break;
	}
	case 1:{
		if(timer.Get() > 1.0){
			stage = 2;
			timer.Reset();
			timer.Start();
		}
		break;
	}
	case 2: {
		if(timer.Get() > 3.0){
			//stage = 3;
			//bot->AutonFreeEncoders();
			//bot->AutonInitEncoders();
			bot->ClawClose();
			bot->ClawWristRetract();
			bot->AutonMoveArm(PID_LOW_TARGET, armLowerMultiplier);
			//bot->AutonSetBearing(180);
		}
		break;
	}
	/*case 3: {
		if(bot->leftEncoder->finished == 1 && bot->rightEncoder->finished == 1){
			stage = 4;
			bot->AutonSetBearing(0);
		}
		break;
	}
	case 4: {
		double heading = bot->pigeon->GetFusedHeading();
		if(abs(bot->angleDifference) < 1 && abs(heading-prevHeading) < 0.2){
			stage = 5;
			bot->AutonFreeEncoders();
			bot->AutonInitEncoders();
			bot->DriveOff();
			bot->ClawOpen();
			bot->ClawWristExtend();
		}
		this->prevHeading = heading;
		break;
	}
	case 5:{
		if(bot->leftEncoder->finished == 1 && bot->rightEncoder->finished == 1){
			stage = 6;

		}
		break;
	}
	case 6:{
		bot->ClawClose();
	}*/
	}
}

CenterSwitchLeft::~CenterSwitchLeft() {
	// TODO Auto-generated destructor stub
}



