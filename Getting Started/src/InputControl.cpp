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
	case ARCADE_JOYSTICK:
		joy1 = new frc::Joystick(0);
		joy2 = joy1;
		break;
	case ARCADE_PS4:
		joy1 = new frc::Joystick(2);
		joy2 = joy1;
		break;
	case TANK_JOYSTICK:
		joy1 = new frc::Joystick(0);
		joy2 = new frc::Joystick(3);
		break;
	case TANK_PS4:
		joy1 = new frc::Joystick(2);
		joy2 = joy1;
		break;
	case DUAL_JOYSTICK:
		joy1 = new frc::Joystick(0);
		joy2 = new frc::Joystick(3);
		break;
	case JOYSTICK_THROTTLE:
		joy1 = new frc::Joystick(0);
		joy2 = new frc::Joystick(1);
		break;
	default:
		break;
	}
}


void InputControl::SetLeftMotor(TalonSRX * left){
	this->left = left;
}
void InputControl::SetRightMotor(TalonSRX * right){
	this->right = right;
}
void InputControl::SetArmMotor(TalonSRX * arm){
	this->arm = arm;
}
void InputControl::SetLeftClawMotor(TalonSRX * leftClaw){
	this->leftClaw = leftClaw;
}
void InputControl::SetRightClawMotor(TalonSRX * rightClaw){
	this->rightClaw = rightClaw;
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

double InputControl::GetRawAxisLeftThrottle(){
	switch(mode){
	case ARCADE_JOYSTICK:
		return joy1->GetY();
	case ARCADE_PS4:
		return joy1->GetY();
	case TANK_JOYSTICK:
		return joy2->GetY();
	case TANK_PS4:
		return joy1->GetY();
	case DUAL_JOYSTICK:
		return joy1->GetY();
	case JOYSTICK_THROTTLE:
		return joy1->GetY();
	default:
		return 0.0;
	}
}
double InputControl::GetRawAxisRightThrottle(){
	switch(mode){
	case ARCADE_JOYSTICK:
		return joy1->GetY();
	case ARCADE_PS4:
		return joy1->GetY();
	case TANK_JOYSTICK:
		return joy1->GetY();
	case TANK_PS4:
		return joy1->GetRawAxis(5);
	case DUAL_JOYSTICK:
		return joy1->GetY();
	case JOYSTICK_THROTTLE:
		return joy1->GetY();
	default:
		return 0.0;
	}
}
double InputControl::GetRawAxisLeftTurn(){
	switch(mode){
	case ARCADE_JOYSTICK:
		return joy1->GetX();
	case ARCADE_PS4:
		return joy1->GetX();
	case TANK_JOYSTICK:
		return 0.0;
	case TANK_PS4:
		return 0.0;
	case DUAL_JOYSTICK:
		return joy1->GetX();
	case JOYSTICK_THROTTLE:
		return joy1->GetX();
	default:
		return 0.0;
	}
}
double InputControl::GetRawAxisRightTurn(){
	switch(mode){
	case ARCADE_JOYSTICK:
		return joy1->GetX();
	case ARCADE_PS4:
		return joy1->GetX();
	case TANK_JOYSTICK:
		return 0.0;
	case TANK_PS4:
		return 0.0;
	case DUAL_JOYSTICK:
		return joy1->GetX();
	case JOYSTICK_THROTTLE:
		return joy1->GetX();
	default:
		return 0.0;
	}
}
double InputControl::GetRawAxisArm(){
	switch(mode){
	case ARCADE_JOYSTICK:
		return joy1->GetZ();
	case ARCADE_PS4:
	{
		double val = joy1->GetRawAxis(5);
		if(abs(val) < 0.001){
			val = 0.0;
		}
		this->armTarget = this->armTarget + val * 750;
		return this->armTarget;
		//return joy1->GetRawAxis(5);
	}
	case TANK_JOYSTICK:
		return joy1->GetZ();
	case TANK_PS4:
	{
		double val = 0.0;
		bool leftTrigger = joy1->GetRawButton(7);
		bool rightTrigger = joy1->GetRawButton(8);
		if(leftTrigger && rightTrigger){
			val = 0.0;
		} else if(leftTrigger){
			val = (joy1->GetRawAxis(3) + 1.0)/2.0;
		} else if(rightTrigger){
			val = (joy1->GetRawAxis(4) + 1.0)/2.0;
			val = val * -1;
		}
		return val;
	}
	case DUAL_JOYSTICK:
		return joy2->GetY();
	case JOYSTICK_THROTTLE:
	{
		double val = -1*joy2->GetX();
		double joyMin = -1.0;
		double joyMax = 1.0;
		//double min = this->arm->ConfigGetParameter(ParamEnum::eReverseSoftLimitThreshold, 0, 10);
		//double max = this->arm->ConfigGetParameter(ParamEnum::eForwardSoftLimitThreshold, 0, 10);
		double min = 0;
		double max = -135000;
		double target = min + ((max - min)/(joyMax - joyMin))* (val - joyMin);
		return target;
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
double InputControl::GetAxisArm(){
	return this->GetRawAxisArm() * this->armMult;
}

bool InputControl::GetButtonClawToggle(){
	switch(mode){
	case ARCADE_JOYSTICK:
		return joy1->GetRawButtonPressed(1);
	case ARCADE_PS4:
		return joy1->GetRawButtonPressed(2);
	case TANK_JOYSTICK:
		return joy1->GetRawButtonPressed(1);
	case TANK_PS4:
		return joy1->GetRawButtonPressed(2);
	case DUAL_JOYSTICK:
		return joy1->GetRawButtonPressed(1);
	case JOYSTICK_THROTTLE:
		return joy1->GetRawButtonPressed(1);
	default:
		return false;
	}
}
bool InputControl::GetButtonClawWristToggle(){
	switch(mode){
	case ARCADE_JOYSTICK:
		return joy1->GetRawButtonPressed(2);
	case ARCADE_PS4:
		return joy1->GetRawButtonPressed(4);
	case TANK_JOYSTICK:
		return joy1->GetRawButtonPressed(2);
	case TANK_PS4:
		return joy1->GetRawButtonPressed(4);
	case DUAL_JOYSTICK:
		return joy1->GetRawButtonPressed(2);
	case JOYSTICK_THROTTLE:
		return joy1->GetRawButtonPressed(2);
	default:
		return false;
	}
}
bool InputControl::GetButtonClawSuck(){
	switch(mode){
	case ARCADE_JOYSTICK:
		return joy1->GetRawButton(13);
	case ARCADE_PS4:
	{
		int angle = joy1->GetPOV(0);
		return (angle == 225 || angle == 180 || angle == 135);
	}
	case TANK_JOYSTICK:
		return joy1->GetRawButton(13);
	case TANK_PS4:
	{
		int angle = joy1->GetPOV(0);
		return (angle == 225 || angle == 180 || angle == 135);
	}
	case DUAL_JOYSTICK:
		return joy1->GetRawButton(13);
	case JOYSTICK_THROTTLE:
		return joy1->GetRawButton(13);
	default:
		return false;
	}
}
bool InputControl::GetButtonClawSpit(){
	switch(mode){
	case ARCADE_JOYSTICK:
		return joy1->GetRawButton(11);
	case ARCADE_PS4:
	{
		int angle = joy1->GetPOV(0);
		return (angle == 0 || angle == 315 || angle == 45);
	}
	case TANK_JOYSTICK:
		return joy1->GetRawButton(11);
	case TANK_PS4:
	{
		int angle = joy1->GetPOV(0);
		return (angle == 0 || angle == 315 || angle == 45);
	}
	case DUAL_JOYSTICK:
		return joy1->GetRawButton(11);
	case JOYSTICK_THROTTLE:
		return joy1->GetRawButton(11);
	default:
		return false;
	}
}
bool InputControl::GetButtonKill(){
	switch(mode){
	case ARCADE_JOYSTICK:
		return joy1->GetRawButtonPressed(6);
	case ARCADE_PS4:
		return joy1->GetRawButtonPressed(13);
	case TANK_JOYSTICK:
		return joy1->GetRawButtonPressed(6);
	case TANK_PS4:
		return joy1->GetRawButtonPressed(13);
	case DUAL_JOYSTICK:
		return joy1->GetRawButtonPressed(6);
	case JOYSTICK_THROTTLE:
		return joy1->GetRawButtonPressed(6);
	default:
		return false;
	}
}
bool InputControl::GetButtonArmCalibrate(){
	switch(mode){
	case ARCADE_JOYSTICK:
		return joy1->GetRawButtonPressed(5);
	case ARCADE_PS4:
		return joy1->GetRawButtonPressed(9);
	case TANK_JOYSTICK:
		return joy1->GetRawButtonPressed(5);
	case TANK_PS4:
		return joy1->GetRawButtonPressed(9);
	case DUAL_JOYSTICK:
		return joy1->GetRawButtonPressed(5);
	case JOYSTICK_THROTTLE:
		return joy1->GetRawButtonPressed(5);
	default:
		return false;
	}
}
bool InputControl::GetButtonAuton(){
	switch(mode){
	case ARCADE_JOYSTICK:
		return joy1->GetRawButtonPressed(3);
	case ARCADE_PS4:
		return joy1->GetRawButtonPressed(5);
	case TANK_JOYSTICK:
		return joy1->GetRawButtonPressed(3);
	case TANK_PS4:
		return joy1->GetRawButtonPressed(5);
	case DUAL_JOYSTICK:
		return joy1->GetRawButtonPressed(3);
	case JOYSTICK_THROTTLE:
		return joy1->GetRawButtonPressed(3);
	default:
		return false;
	}
}
bool InputControl::GetButtonPID(){
	switch(mode){
	case ARCADE_JOYSTICK:
		return joy1->GetRawButtonPressed(4);
	case ARCADE_PS4:
		return joy1->GetRawButtonPressed(6);
	case TANK_JOYSTICK:
		return joy1->GetRawButtonPressed(4);
	case TANK_PS4:
		return joy1->GetRawButtonPressed(6);
	case DUAL_JOYSTICK:
		return joy1->GetRawButtonPressed(4);
	case JOYSTICK_THROTTLE:
		return joy1->GetRawButtonPressed(4);
	default:
		return false;
	}

}

bool InputControl::GetButtonLower(){
	switch(mode){
	case ARCADE_JOYSTICK:
		return joy1->GetRawButtonPressed(7);
	case ARCADE_PS4:
		return joy1->GetRawButtonPressed(10);
	case TANK_JOYSTICK:
		return joy1->GetRawButtonPressed(7);
	case TANK_PS4:
		return joy1->GetRawButtonPressed(10);
	case DUAL_JOYSTICK:
		return joy1->GetRawButtonPressed(7);
	case JOYSTICK_THROTTLE:
		return joy1->GetRawButtonPressed(7);
	default:
		return false;
	}

}

bool InputControl::GetButtonRaise(){
	switch(mode){
	case ARCADE_JOYSTICK:
		return joy1->GetRawButtonPressed(9);
	case ARCADE_PS4:
		return joy1->GetRawButtonPressed(9);
	case TANK_JOYSTICK:
		return joy1->GetRawButtonPressed(9);
	case TANK_PS4:
		return joy1->GetRawButtonPressed(9);
	case DUAL_JOYSTICK:
		return joy1->GetRawButtonPressed(9);
	case JOYSTICK_THROTTLE:
		return joy1->GetRawButtonPressed(9);
	default:
		return false;
	}

}

int InputControl::GetMode(){
	return this->mode;
}

void InputControl::Drive(){
	double leftThrottle = this->GetRawAxisLeftThrottle() * this->leftThrottleMult;
	double rightThrottle = this->GetRawAxisRightThrottle() * this->rightThrottleMult;
	double leftTurn = this->GetRawAxisLeftTurn() * this->leftTurnMult;
	double rightTurn = this->GetRawAxisRightTurn() * this->rightTurnMult;
	this->left->Set(ControlMode::PercentOutput, leftThrottle - leftTurn);
	this->right->Set(ControlMode::PercentOutput, rightThrottle + rightTurn);
}

void InputControl::MoveArm(){
	if(mode == JOYSTICK_THROTTLE){
		int target = (int) this->GetAxisArm();
		this->arm->Set(ControlMode::Position, target);
	} else if (mode == ARCADE_PS4){
		int target = this->GetRawAxisArm();
		this->arm->Set(ControlMode::Position, target);
	}
		else {
		double armPower = this->GetAxisArm();
		this->arm->Set(ControlMode::PercentOutput, armPower);
	}
}

InputControl::~InputControl() {
	// TODO Auto-generated destructor stub
}

