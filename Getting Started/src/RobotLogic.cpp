/*
 * RobotLogic.cpp
 *
 *  Created on: Feb 10, 2018
 *      Author: PatriotRobotics
 */

#include <RobotLogic.h>

RobotLogic::RobotLogic(InputControl * control){
	this->control = control;
}

void RobotLogic::SetLeftMotor(TalonSRX * left){
	this->left = left;
}
void RobotLogic::SetRightMotor(TalonSRX * right){
	this->right = right;
}
void RobotLogic::SetArmMotor(TalonSRX * arm){
	this->arm = arm;
}
void RobotLogic::SetLeftClawMotor(TalonSRX * leftClaw){
	this->leftClaw = leftClaw;
}
void RobotLogic::SetRightClawMotor(TalonSRX * rightClaw){
	this->rightClaw = rightClaw;
}
void RobotLogic::SetClaw(frc::DoubleSolenoid * claw){
	this->claw = claw;
}
void RobotLogic::SetClawWrist(frc::DoubleSolenoid * clawWrist){
	this->clawWrist = clawWrist;
}


void RobotLogic::SetMode(){
	if(this->mode != KILLED){
		if(this->control->GetButtonAuton()){
			if(this->mode == AUTON){
				this->mode = DIRECT;
			} else {
				this->mode = AUTON;
			}
		} else if(this->control->GetButtonPID()){
			if(this->mode == PID){
				this->mode = DIRECT;
			} else {
				this->mode = PID;
			}
		}
	}
}

void RobotLogic::ResetClaw(){
	this->claw->Set(frc::DoubleSolenoid::Value::kOff);
	this->clawWrist->Set(frc::DoubleSolenoid::Value::kOff);
	this->clawEnabled = false;
	this->clawWristEnabled = false;
}

void RobotLogic::ResetAuton(){

}
void RobotLogic::ResetPID(){

}

void RobotLogic::Kill(){
	this->left->Set(ControlMode::PercentOutput, 0.0);
	this->right->Set(ControlMode::PercentOutput, 0.0);
	this->arm->Set(ControlMode::PercentOutput, 0.0);
	this->leftClaw->Set(ControlMode::PercentOutput, 0.0);
	this->rightClaw->Set(ControlMode::PercentOutput, 0.0);
	claw->Set(frc::DoubleSolenoid::Value::kOff);
	clawWrist->Set(frc::DoubleSolenoid::Value::kOff);
}

void RobotLogic::PIDMoveArm(int target){
	this->arm->Set(ControlMode::Position, target);
}

void RobotLogic::DirectDrive(){
	double leftThrottle = this->control->GetAxisLeftThrottle();
	double rightThrottle = this->control->GetAxisRightThrottle();
	double leftTurn = this->control->GetAxisLeftTurn();
	double rightTurn = this->control->GetAxisRightTurn();
	this->left->Set(ControlMode::PercentOutput, leftThrottle - leftTurn);
	this->right->Set(ControlMode::PercentOutput, rightThrottle - rightTurn);
}

void RobotLogic::DirectMoveArm(){
	if(this->control->GetMode() == JOYSTICK_THROTTLE){
		int armTarget = (int) this->control->GetRawAxisArm();
		this->arm->Set(ControlMode::Position, armTarget);
	} else {
		double armPower = this->control->GetAxisArm();
		this->arm->Set(ControlMode::PercentOutput, armPower);
	}
}

void RobotLogic::DirectControlClaw(){
	//Claw Pneumatics
	if(control->GetButtonClawToggle()){
		this->clawEnabled = !this->clawEnabled;
	}
	if(control->GetButtonClawWristToggle()){
		this->clawWristEnabled = !this->clawWristEnabled;
	}

	if(this->clawEnabled){
		this->claw->Set(frc::DoubleSolenoid::Value::kReverse);
	} else {
		this->claw->Set(frc::DoubleSolenoid::Value::kForward);
	}
	if(clawWristEnabled){
		this->clawWrist->Set(frc::DoubleSolenoid::Value::kForward);
	} else {
		this->clawWrist->Set(frc::DoubleSolenoid::Value::kReverse);
	}

	//Claw Motors
	if(control->GetButtonClawSuck()){
		leftClaw->Set(ControlMode::PercentOutput, 0.5);
		rightClaw->Set(ControlMode::PercentOutput, 0.5);
	} else if(control->GetButtonClawSpit()){
		leftClaw->Set(ControlMode::PercentOutput, -1.0);
		rightClaw->Set(ControlMode::PercentOutput, -1.0);
	} else {
		leftClaw->Set(ControlMode::PercentOutput, 0.1);
		rightClaw->Set(ControlMode::PercentOutput, 0.1);
	}
}

RobotLogic::~RobotLogic() {
	// TODO Auto-generated destructor stub
}

