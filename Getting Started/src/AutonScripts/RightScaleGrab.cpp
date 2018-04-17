/*
 * RightScaleGrab.cpp
 *
 *  Created on: Apr 16, 2018
 *      Author: PatriotRobotics
 */

#include <AutonScripts/RightScaleGrab.h>

RightScaleGrab::RightScaleGrab(RobotLogic * bot) : AutonScript(2, bot){
	int numPoints_1 = 4;
	Waypoint * points_1 = (Waypoint*)malloc(numPoints_1 * sizeof(Waypoint));
	Waypoint p11 = { 0, 0, 0 };
	Waypoint p12 = { 2, 0, 0 };
	Waypoint p13 = {4, 0, 0};
	Waypoint p14 = {7.4, 0, 0};
	//Waypoint p15 = {8.65, -0.125, 0};
	points_1[0] = p11;
	points_1[1] = p12;
	points_1[2] = p13;
	points_1[3] = p14;
	//points_1[4] = p15;

	//OLD COMPETITION AND UNUSED
	/*int numPoints_2 = 3;
	Waypoint * points_2 = (Waypoint*)malloc(numPoints_2 * sizeof(Waypoint));
	Waypoint p21 = {0, 0, 0 };
	Waypoint p22 = {2, -1, d2r(-30) };
	Waypoint p23 = {4, -2, 0};
	points_2[0] = p21;
	points_2[1] = p22;
	points_2[2] = p23;*/

	int numPoints_2 = 4;
	Waypoint * points_2 = (Waypoint*)malloc(numPoints_2 * sizeof(Waypoint));
	Waypoint p21 = {0, 0, 0 };
	Waypoint p22 = {1.0, 0.25, 0};
	Waypoint p23 = {2.25, -0.25, d2r(30) };
	Waypoint p24 = {2.75, -1.25, d2r(90)};
	points_2[0] = p21;
	points_2[1] = p22;
	points_2[2] = p23;
	points_2[3] = p24;

	this->BuildTrajectory(points_1, numPoints_1, 0);
	this->BuildTrajectory(points_2, numPoints_2, 1);
}

void RightScaleGrab::RunScript(){
	bot->AutonMoveArm();
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
		break;
	}
	case 3: {
		bot->AutonMoveArm(PID_HIGH_TARGET, armRaiseMultiplier);
		break;
	}
	case 4: {
		bot->AutonMoveArm();
		bot->ClawWristExtend();
		break;
	}
	case 5: {
		bot->AutonMoveArm();
		bot->ClawSpitSpeed(-0.45);
		break;
	}
	case 6: {
		bot->AutonMoveArm();
		bot->ClawNeutralSuck();
		bot->ClawWristRetract();
		break;
	}
	case 7: {
		bot->AutonMoveArm(PID_LOW_TARGET, armLowerMultiplier);
		break;
	}
	case 8: {
		bot->AutonMoveArm();
		bot->AutonTurn();
		bot->ClawWristRetract();
		break;
	}
	case 9: {
		bot->ClawWristRetract();
		bot->ClawClose();
		bot->AutonFollowTrajectory(this->leftTrajectories[1], this->rightTrajectories[1], this->trajectoriesLength[1]);
		break;
	}
	case 10: {
		bot->AutonTurn();
		break;
	}
	case 11: {
		bot->ClawWristExtend();
		bot->ClawSuck();
		bot->AutonVisionFollow(0.45);
		break;
	}
	case 12: {
		bot->ClawNeutralSuck();
		bot->AutonFollow(-0.4);
		break;
	}
	case 13: {
		bot->ClawOpen();
		break;
	}
	}
}

void RightScaleGrab::CheckFlags(){
	switch(stage){
	case 0: {
		if(bot->leftEncoder->finished == 1 && bot->rightEncoder->finished == 1){
			stage = 1;
			bot->DriveOff();
			bot->AutonSetBearing(-100);
		}
		break;
	}
	case 1:{
		double heading = bot->pigeon->GetFusedHeading();
		if(abs(bot->angleDifference) < 10 && abs(heading-prevHeading) < 0.2){
			stage = 2;
			bot->DriveOff();
			timer.Start();
		}
		this->prevHeading = heading;
		break;
	}
	case 2:{
		if(timer.Get() > 1){
			stage = 3;
		}
		break;
	}
	case 3: {
		if(abs(bot->armTarget - PID_HIGH_TARGET) < PID_ARM_MULTIPLIER * armEndingRangeMultiplier * armRaiseMultiplier){
			stage = 4;
			timer.Start();
		}
		break;
	}
	case 4: {
		if(timer.Get() > 3.0){
			stage = 5;
			timer.Reset();
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
		if(timer.Get() > 1){
			stage = 7;
		}
		break;
	}
	case 7: {
		if(abs(bot->armTarget - PID_LOW_TARGET) < PID_ARM_MULTIPLIER * armEndingRangeMultiplier * armLowerMultiplier){
			stage = 8;
			bot->DriveOff();
			bot->AutonSetBearing(180);
		}
		break;
	}
	case 8: {
		double heading = bot->pigeon->GetFusedHeading();
		if(abs(bot->angleDifference) < 10 && abs(heading-prevHeading) < 0.2){
			stage = 9;
			bot->DriveOff();
			bot->AutonFreeEncoders();
			bot->AutonInitEncoders();
		}
		this->prevHeading = heading;
		break;
	}
	case 9: {
		if(bot->leftEncoder->finished == 1 && bot->rightEncoder->finished == 1){
			stage = 10;
			bot->DriveOff();
			bot->AutonSetBearing(180);
		}
		break;
	}
	case 10: {
		double heading = bot->pigeon->GetFusedHeading();
		if(abs(bot->angleDifference) < 10 && abs(heading-prevHeading) < 0.2){
			stage = 11;
			bot->DriveOff();
			timer.Reset();
			timer.Start();
			//bot->AutonFreeEncoders();
			//bot->AutonInitEncoders();
		}
		this->prevHeading = heading;
		break;
	}
	case 11: {
		if (timer.Get() > 1.5){
			stage = 12;
			bot->DriveOff();
			bot->AutonSetBearing(180);
			timer.Reset();
			timer.Start();
		}
		break;
	}
	case 12: {
		if(timer.Get() > 0.5){
			stage = 13;
			bot->DriveOff();
		}
		break;
	}
	case 13: {
		break;
	}
	}
}

RightScaleGrab::~RightScaleGrab() {
	// TODO Auto-generated destructor stub
}

