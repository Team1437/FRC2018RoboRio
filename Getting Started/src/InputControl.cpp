/*
 * InputControl.cpp
 *
 *  Created on: Feb 8, 2018
 *      Author: PatriotRobotics
 */

#include <InputControl.h>
#include <Joystick.h>
#include "ctre/Phoenix.h"

InputControl::InputControl(int type){
	mode = type;
	delete joy1;
	switch(mode){
	case FINAL_CONTROL:
		joy1 = new frc::Joystick(3);
		joy2 = new frc::Joystick(5);
		break;
	case FINAL_CONTROL_SINGLE_ARCADE:
		joy1 = new frc::Joystick(3);
		joy2 = new frc::Joystick(5);
	default:
		break;
	}
}

void InputControl::SetLeftThrottleMult(double leftThrottleMult){
	this->leftThrottleMult = leftThrottleMult;
}
void InputControl::SetRightThrottleMult(double rightThrottleMult){
	this->rightThrottleMult = rightThrottleMult;
}
void InputControl::SetLeftTurnMult(double leftTurnMult){
	this->leftTurnMult = leftTurnMult;
}
void InputControl::SetRightTurnMult(double rightTurnMult){
	this->rightTurnMult = rightTurnMult;
}
void InputControl::SetDriveMultipliers(double leftThrottleMult, double rightThrottleMult, double leftTurnMult, double rightTurnMult){
	this->leftThrottleMult = leftThrottleMult;
	this->rightThrottleMult = rightThrottleMult;
	this->leftTurnMult = leftTurnMult;
	this->rightTurnMult = rightTurnMult;
}
void InputControl::SetArmMultiplier(double armMult){
	this->armMult = armMult;
}

void InputControl::SetThrottleSetPoint(){
	this->throttleSetPoint = this->GetRawAxisArm();
}

double InputControl::GetRawAxisLeftThrottle(){
	switch(mode){
	case FINAL_CONTROL:
		return joy1->GetRawAxis(1) * -1;
	case FINAL_CONTROL_SINGLE_ARCADE:
		return joy1->GetRawAxis(1) * -1;
	default:
		return 0.0;
	}
}
double InputControl::GetRawAxisRightThrottle(){
	switch(mode){
	case FINAL_CONTROL:
		return joy1->GetRawAxis(1) * -1;
	case FINAL_CONTROL_SINGLE_ARCADE:
		return joy1->GetRawAxis(1) * -1;
	default:
		return 0.0;
	}
}
double InputControl::GetRawAxisLeftTurn(){
	switch(mode){
	case FINAL_CONTROL:
		return joy1->GetRawAxis(2);
	case FINAL_CONTROL_SINGLE_ARCADE:
		return joy1->GetRawAxis(2);
	default:
		return 0.0;
	}
}
double InputControl::GetRawAxisRightTurn(){
	switch(mode){
	case FINAL_CONTROL:
		return joy1->GetRawAxis(2);
	case FINAL_CONTROL_SINGLE_ARCADE:
		return joy1->GetRawAxis(2);
	default:
		return 0.0;
	}
}
double InputControl::GetRawAxisArm(){
	switch(mode){
	case FINAL_CONTROL:
	{
		int angle = joy2->GetPOV(0);
		bool down = (angle == 225 || angle == 180 || angle == 135);
		bool up = (angle == 0 || angle == 315 || angle == 45);
		double active = 0.0;
		if(up){
			active = -1.0;
		}
		if(down){
			active = 1.0;
		}
		return active;
	}
	case FINAL_CONTROL_SINGLE_ARCADE:
	{
		int angle = joy1->GetPOV(0);
		bool down = (angle == 225 || angle == 180 || angle == 135);
		bool up = (angle == 0 || angle == 315 || angle == 45);
		double active = 0.0;
		if(up){
			active = -1.0;
		}
		if(down){
			active = 1.0;
		}
		return active;
	}
	default:
		return 0.0;
	}
}

double InputControl::GetAxisLeftThrottle(){
	return this->GetRawAxisLeftThrottle() * this->leftThrottleMult;
}
double InputControl::GetAxisRightThrottle(){
	return this->GetRawAxisRightThrottle() * this->rightThrottleMult;
}
double InputControl::GetAxisLeftTurn(){
	return this->GetRawAxisLeftTurn() * this->leftTurnMult;
}
double InputControl::GetAxisRightTurn(){
	return this->GetRawAxisRightTurn() * this->rightTurnMult;
}
int InputControl::GetAxisArmChange(){
	switch(mode){
	case FINAL_CONTROL:
	{
		return this->GetRawAxisArm()*this->armMult;
	}
	case FINAL_CONTROL_SINGLE_ARCADE:
	{
		return this->GetRawAxisArm()*this->armMult;
	}
	default:
		return 0.0;
	}
}

bool InputControl::GetButtonClawTogglePressed(){
	switch(mode){
	case FINAL_CONTROL:
		return joy2->GetRawButtonPressed(6);
	case FINAL_CONTROL_SINGLE_ARCADE:
		return joy1->GetRawButtonPressed(6);
	default:
		return false;
	}
}

bool InputControl::GetButtonClawToggle(){
	switch(mode){
	case FINAL_CONTROL:
		return joy2->GetRawButton(6);
	case FINAL_CONTROL_SINGLE_ARCADE:
		return joy1->GetRawButton(6);
	default:
		return false;
	}
}

bool InputControl::GetButtonClawWristToggle(){
	switch(mode){
	case FINAL_CONTROL:
		return joy2->GetRawButtonPressed(1);
	case FINAL_CONTROL_SINGLE_ARCADE:
		return joy1->GetRawButtonPressed(1);
	default:
		return false;
	}
}
bool InputControl::GetButtonClawSuck(){
	switch(mode){
	case FINAL_CONTROL:
		return joy2->GetRawButton(8);
	case FINAL_CONTROL_SINGLE_ARCADE:
		return joy1->GetRawButton(8);
	default:
		return false;
	}
}
bool InputControl::GetButtonClawSpitFast(){
	switch(mode){
	case FINAL_CONTROL:
		return joy2->GetRawButton(5);
	case FINAL_CONTROL_SINGLE_ARCADE:
		return joy1->GetRawButton(5);
	default:
		return false;
	}
}
bool InputControl::GetButtonClawSpitSlow(){
	switch(mode){
	case FINAL_CONTROL:
		return joy2->GetRawButton(7);
	case FINAL_CONTROL_SINGLE_ARCADE:
		return joy1->GetRawButton(7);
	default:
		return false;
	}
}
bool InputControl::GetButtonKill(){
	switch(mode){
	case FINAL_CONTROL:
		return false;
	case FINAL_CONTROL_SINGLE_ARCADE:
		return false;
	default:
		return false;
	}
}
bool InputControl::GetButtonArmCalibrate(){
	switch(mode){
	case FINAL_CONTROL:
		return false;
	case FINAL_CONTROL_SINGLE_ARCADE:
		return false;
	default:
		return false;
	}
}
bool InputControl::GetButtonAuton(){
	switch(mode){
	case FINAL_CONTROL:
		return false;
	case FINAL_CONTROL_SINGLE_ARCADE:
		return false;
	default:
		return false;
	}
}
bool InputControl::GetButtonPID(){
	switch(mode){
	case FINAL_CONTROL:
		return false;
	case FINAL_CONTROL_SINGLE_ARCADE:
		return false;
	default:
		return false;
	}

}

bool InputControl::GetButtonArmLow(){
	switch(mode){
	case FINAL_CONTROL:
		return joy2->GetRawButtonPressed(2);
	case FINAL_CONTROL_SINGLE_ARCADE:
		return joy1->GetRawButtonPressed(2);
	default:
		return false;
	}

}
bool InputControl::GetButtonArmMid(){
	switch(mode){
	case FINAL_CONTROL:
		return joy2->GetRawButtonPressed(3);
	case FINAL_CONTROL_SINGLE_ARCADE:
		return joy1->GetRawButtonPressed(3);
	default:
		return false;
	}

}

bool InputControl::GetButtonArmHigh(){
	switch(mode){
	case FINAL_CONTROL:
		return joy2->GetRawButtonPressed(4);
	case FINAL_CONTROL_SINGLE_ARCADE:
		return joy1->GetRawButtonPressed(4);
	default:
		return false;
	}

}

void InputControl::ToggleMode(){
	switch(mode){
	case FINAL_CONTROL:
		break;
	case FINAL_CONTROL_SINGLE_ARCADE:
		break;
	default:
		break;
	}
}

int InputControl::GetMode(){
	return this->mode;
}

InputControl::~InputControl() {
}

