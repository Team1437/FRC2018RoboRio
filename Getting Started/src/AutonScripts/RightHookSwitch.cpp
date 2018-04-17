/*
 * RightHookSwitch.cpp
 *
 *  Created on: Mar 11, 2018
 *      Author: PatriotRobotics
 */

#include <AutonScripts/RightHookSwitch.h>

RightHookSwitch::RightHookSwitch(RobotLogic * bot) : AutonScript(3, bot){
	int numPoints_1 = 3;
	Waypoint * points_1 = (Waypoint*)malloc(numPoints_1 * sizeof(Waypoint));
	Waypoint p11 = {0, 0, 0};
	Waypoint p12 = {3, 0, 0};
	Waypoint p13 = {5.25, 1, 0};
	points_1[0] = p11;
	points_1[1] = p12;
	points_1[2] = p13;

	int numPoints_2 = 3;
	Waypoint * points_2 = (Waypoint*)malloc(numPoints_2 * sizeof(Waypoint));
	Waypoint p21 = {0, 0, 0};
	Waypoint p22 = {1.5, 0, 0};
	Waypoint p23 = {3.0, 0, 0};
	points_2[0] = p21;
	points_2[1] = p22;
	points_2[2] = p23;

	int numPoints_3 = 3;
	Waypoint * points_3 = (Waypoint*)malloc(numPoints_3 * sizeof(Waypoint));
	Waypoint p31 = {0, 0, 0};
	Waypoint p32 = {0.5, 0, 0};
	Waypoint p33 = {1.25, 0, 0};
	points_3[0] = p31;
	points_3[1] = p32;
	points_3[2] = p33;

	this->BuildTrajectory(points_1, numPoints_1, 0);
	this->BuildTrajectory(points_2, numPoints_2, 1);
	this->BuildTrajectory(points_3, numPoints_3, 2);
}

void RightHookSwitch::RunScript(){
	switch(stage){
	case 0: {
		bot->AutonFollowTrajectory(this->leftTrajectories[0], this->rightTrajectories[0], this->trajectoriesLength[0]);
		break;
	}
	case 1: {
		bot->AutonTurn();
		break;
	}
	case 2: {
		bot->AutonFollowTrajectory(this->leftTrajectories[1], this->rightTrajectories[1], this->trajectoriesLength[1]);
		break;
	}
	case 3: {
		bot->AutonTurn();
		break;
	}
	case 4: {
		bot->AutonMoveArm(PID_AUTO_TARGET, armRaiseMultiplier);
		bot->AutonFollowTrajectory(this->leftTrajectories[2], this->rightTrajectories[2], this->trajectoriesLength[2]);
		break;
	}
	case 5: {
		bot->ClawWristExtend();
		break;
	}
	case 6: {
		bot->ClawOpen();
		break;
	}
	case 7: {
		bot->AutonMoveArm(PID_LOW_TARGET, armLowerMultiplier);
		//bot->ClawWristRetract();
		//bot->ClawClose();
		bot->ClawNeutralSuck();
		//bot->AutonTurn();
	}
	case 8: {
		bot->ClawOpen();
		break;
	}
	case 9: {
		bot->ClawWristExtend();
		break;
	}
	}
}

void RightHookSwitch::CheckFlags(){
	switch(stage){
	case 0: {
		if(bot->leftEncoder->finished == 1 && bot->rightEncoder->finished == 1){
			stage = 1;
			bot->DriveOff();
			bot->AutonSetBearing(90);
		}
		break;
	}
	case 1: {
		double heading = bot->pigeon->GetFusedHeading();
		if(abs(bot->angleDifference) < 10 && abs(heading-prevHeading) < 0.2){
			stage = 2;
			bot->DriveOff();
			bot->AutonFreeEncoders();
			bot->AutonInitEncoders();
		}
		this->prevHeading = heading;
		break;
	}
	case 2: {
		if(bot->leftEncoder->finished == 1 && bot->rightEncoder->finished == 1){
			stage = 3;
			bot->DriveOff();
			bot->AutonSetBearing(150);
		}
		break;
	}
	case 3: {
		double heading = bot->pigeon->GetFusedHeading();
		if(abs(bot->angleDifference) < 10 && abs(heading-prevHeading) < 0.2){
			stage = 4;
			bot->DriveOff();
			bot->AutonFreeEncoders();
			bot->AutonInitEncoders();
		}
		this->prevHeading = heading;
		break;
	}
	case 4: {
		if(bot->leftEncoder->finished == 1 && bot->rightEncoder->finished == 1){
			stage = 5;
			bot->DriveOff();
			timer.Start();
		}
		break;
	}
	case 5: {
		if(timer.Get() > 1){
			stage = 6;
			timer.Reset();
			timer.Start();
		}
		break;
	}
	case 6: {
		if(timer.Get() > 3){
			stage = 7;
			//bot->AutonSetBearing(180);
		}
		break;
	}
	case 7: {
		double heading = bot->pigeon->GetFusedHeading();
		if(abs(bot->angleDifference) < 10 && abs(heading-prevHeading) < 0.2){
			//stage = 8;
			bot->DriveOff();
			bot->AutonFreeEncoders();
			bot->AutonInitEncoders();
			timer.Reset();
			timer.Start();
		}
		this->prevHeading = heading;
		break;
	}
	case 8: {
		if(timer.Get() > 1){
			stage = 9;
		}
		break;
	}
	case 9: {
		break;
	}
	}
}

RightHookSwitch::~RightHookSwitch() {
	// TODO Auto-generated destructor stub
}

