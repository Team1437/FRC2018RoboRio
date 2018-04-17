/*
 * LeftScale.cpp
 *
 *  Created on: Mar 6, 2018
 *      Author: PatriotRobotics
 */

#include <AutonScripts/LeftScale.h>

LeftScale::LeftScale(RobotLogic * bot) : AutonScript(2, bot){
	int numPoints_1 = 4;
	Waypoint * points_1 = (Waypoint*)malloc(numPoints_1 * sizeof(Waypoint));
	/*Waypoint p11 = { 0, 0, 0 };
	Waypoint p12 = { 2, 1, d2r(30) };
	Waypoint p13 = {4, 2, 0};
	Waypoint p14 = {6, 1, d2r(-30)};
	Waypoint p15 = {7.5, 0, 0};*/
	Waypoint p11 = { 0, 0, 0 };
	Waypoint p12 = { 2, 0, 0 };
	Waypoint p13 = {4, 0, 0};
	Waypoint p14 = {8.75, -0.5, 0};
	//Waypoint p15 = {7.25, -0.25, 0};
	points_1[0] = p11;
	points_1[1] = p12;
	points_1[2] = p13;
	points_1[3] = p14;
	//points_1[4] = p15;

	int numPoints_2 = 3;
	Waypoint * points_2 = (Waypoint*)malloc(numPoints_2 * sizeof(Waypoint));
	Waypoint p21 = {0, 0, 0 };
	Waypoint p22 = {1.5, 0.5, d2r(30) };
	Waypoint p23 = {2.0, 2, 0};
	points_2[0] = p21;
	points_2[1] = p22;
	points_2[2] = p23;

	this->BuildTrajectory(points_1, numPoints_1, 0);
	this->BuildTrajectory(points_2, numPoints_2, 1);
}

void LeftScale::RunScript(){
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
		bot->ClawSpitSpeed(-0.60);
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
		//bot->AutonTurn();
		bot->ClawWristExtend();
		break;
	}
	/*case 9: {
		bot->ClawWristExtend();
		bot->ClawOpen();
		bot->AutonFollowTrajectory(this->leftTrajectories[1], this->rightTrajectories[1], this->trajectoriesLength[1]);
		break;
	}
	case 10: {
		bot->ClawClose();
		break;
	}*/
	}
}

void LeftScale::CheckFlags(){
	switch(stage){
	case 0: {
		if(bot->leftEncoder->finished == 1 && bot->rightEncoder->finished == 1){
			stage = 1;
			bot->DriveOff();
			bot->AutonSetBearing(90);
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
			//bot->AutonSetBearing(180);
		}
		break;
	}
	case 8: {
		/*double heading = bot->pigeon->GetFusedHeading();
		if(abs(bot->angleDifference) < 10 && abs(heading-prevHeading) < 0.2){
			//stage = 9;
			bot->DriveOff();
			//bot->AutonFreeEncoders();
			//bot->AutonInitEncoders();
		}
		this->prevHeading = heading;*/
		break;
	}
	/*case 9: {
		if(bot->leftEncoder->finished == 1 && bot->rightEncoder->finished == 1){
			stage = 10;
			bot->DriveOff();
		}
		break;
	}
	case 10: {
		break;
	}*/
	}
}

LeftScale::~LeftScale() {
	// TODO Auto-generated destructor stub
}

