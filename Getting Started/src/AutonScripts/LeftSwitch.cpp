/*
 * CenterSwitchLeft.cpp
 *
 *  Created on: Mar 7, 2018
 *      Author: patriotrobotics
 */

#include <AutonScripts/LeftSwitch.h>

LeftSwitch::LeftSwitch(RobotLogic * bot): AutonScript(1, bot){
	int numPoints_1 = 3;
	Waypoint * points_1 = (Waypoint*)malloc(numPoints_1 * sizeof(Waypoint));
	Waypoint p11 = {0, 0, 0};
	Waypoint p12 = {2, 0.5, d2r(15)};
	Waypoint p13 = {4, 0.75, 0};
	points_1[0] = p11;
	points_1[1] = p12;
	points_1[2] = p13;

	this->BuildTrajectory(points_1, numPoints_1, 0);
}

void LeftSwitch::RunScript(){
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
		bot->AutonMoveArm(PID_AUTO_TARGET, this->armRaiseMultiplier);
		break;
	}
	case 3: {
		bot->ClawSpitFast();
		break;
	}
	case 4: {
		break;
	}
	case 5: {
		bot->AutonMoveArm(PID_LOW_TARGET, this->armLowerMultiplier);
		break;
	}
	}
}

void LeftSwitch::CheckFlags(){
	switch(stage){
	case 0: {
		if(bot->leftEncoder->finished == 1 && bot->rightEncoder->finished == 1){
			stage = 1;
			bot->AutonSetBearing(-90);
		}
		break;
	}
	case 1: {
		double heading = bot->pigeon->GetFusedHeading();
		if(abs(bot->angleDifference) < 10 && abs(heading-prevHeading) < 0.2){
			stage = 2;
			bot->DriveOff();
			//timer.Start();
		}
		this->prevHeading = heading;
		break;
	}
	case 2: {
		if(abs(bot->armTarget - PID_AUTO_TARGET) < PID_ARM_MULTIPLIER * armEndingRangeMultiplier * armRaiseMultiplier){
			stage = 3;
			timer.Start();
		}
		break;
	}
	case 3: {
		if(timer.Get() > 1){
			stage = 4;
			timer.Reset();
			timer.Start();
		}
		break;
	}
	case 4: {
		if(timer.Get() > 1){
			stage = 5;
			bot->ClawNeutralSuck();
		}
		break;
	}
	case 5: {
		break;
	}
	}
}

// OLD LEFT SWITCH CODE
/*
LeftSwitch::LeftSwitch(RobotLogic * bot) : AutonScript(3, bot){
	int numPoints_1 = 3;
	Waypoint * points_1 = (Waypoint*)malloc(numPoints_1 * sizeof(Waypoint));
	Waypoint p11 ={0, 0, 0};
	Waypoint p12 = {1.5, -0.75, d2r(-15)};
	Waypoint p13 = {3, -1.25, 0};
	points_1[0] = p11;
	points_1[1] = p12;
	points_1[2] = p13;

	int numPoints_2 = 3;
	Waypoint * points_2 = (Waypoint*)malloc(numPoints_2 * sizeof(Waypoint));
	Waypoint p21 = {0, 0, 0};
	Waypoint p22 = {1, 0, 0};
	Waypoint p23 = {2, 0, 0};
	points_2[0] = p21;
	points_2[1] = p22;
	points_2[2] = p23;

	int numPoints_3 = 6;
	Waypoint * points_3 = (Waypoint*)malloc(numPoints_3 * sizeof(Waypoint));
	Waypoint p31 = {0, 0, 0};
	Waypoint p32 = {2, -0.5, d2r(-25)};
	Waypoint p33 = {3, -1.25, d2r(-70)};
	Waypoint p34 = {4, -1.75, d2r(-100)};
	Waypoint p35 = {3.5, -2.5, d2r(-150)};
	Waypoint p36 = {3.25, -3.0, d2r(-160)};
	points_3[0] = p31;
	points_3[1] = p32;
	points_3[2] = p33;
	points_3[3] = p34;
	points_3[4] = p35;
	points_3[5] = p36;

	this->BuildTrajectory(points_1, numPoints_1, 0);
	this->BuildTrajectory(points_2, numPoints_2, 1);
	this->BuildTrajectory(points_3, numPoints_3, 2);
}

void LeftSwitch::RunScript(){
	switch(stage){
	case 0: {
		bot->AutonFollowTrajectory(this->leftTrajectories[0], this->rightTrajectories[0], this->trajectoriesLength[0]);
		break;
	}
	case 1: {
		bot->ClawSpitFast();
		break;
	}
	case 2: {
		bot->AutonTurn();
		break;
	}
	case 3: {
		bot->AutonFollowTrajectory(this->leftTrajectories[1], this->rightTrajectories[1], this->trajectoriesLength[1]);
		break;
	}
	case 4: {
		bot->AutonTurn();
		break;
	}
	case 5: {
		bot->ClawWristExtend();
		bot->ClawOpen();
		break;
	}
	case 6: {
		bot->AutonFollowTrajectory(this->leftTrajectories[2], this->rightTrajectories[2], this->trajectoriesLength[2]);
		break;
	}
	case 7: {
		bot->ClawClose();
		break;
	}
	}
}

void LeftSwitch::CheckFlags(){
	switch(stage){
	case 0: {
		if(bot->leftEncoder->finished == 1 && bot->rightEncoder->finished == 1){
			stage = 1;
			timer.Start();
		}
		break;
	}
	case 1:{
		if(timer.Get() > 1.0){
			stage = 2;
			bot->ClawNeutralSuck();
			bot->AutonSetBearing(90);
		}
		break;
	}
	case 2:{
		double heading = bot->pigeon->GetFusedHeading();
		if(abs(bot->angleDifference) < 10 && abs(heading-prevHeading) < 0.2){
			stage = 3;
			bot->AutonFreeEncoders();
			bot->AutonInitEncoders();
			bot->DriveOff();
		}
		this->prevHeading = heading;
		break;
	}
	case 3:{
		if(bot->leftEncoder->finished == 1 && bot->rightEncoder->finished == 1){
			bot->AutonSetBearing(0);
			stage = 4;
		}
		break;
	}
	case 4:{
		double heading = bot->pigeon->GetFusedHeading();
		if(abs(bot->angleDifference) < 10 && abs(heading-prevHeading) < 0.2){
			stage = 5;
			timer.Reset();
			timer.Start();
			bot->DriveOff();
		}
		this->prevHeading = heading;
		break;
	}
	case 5: {
		if(timer.Get() > 1){
			stage = 6;
			bot->AutonFreeEncoders();
			bot->AutonInitEncoders();
		}
		break;
	}
	case 6: {
		if(bot->leftEncoder->finished == 1 && bot->rightEncoder->finished == 1){
			stage = 7;
		}
		break;
	}
	case 7:{
		break;
	}
	}
}
 */
LeftSwitch::~LeftSwitch() {
	// TODO Auto-generated destructor stub
}

