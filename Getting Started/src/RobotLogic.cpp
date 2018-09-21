/*
 * RobotLogic.cpp
 *
 *  Created on: Feb 10, 2018
 *      Author: PatriotRobotics
 */



#include <RobotLogic.h>
#include <pathfinder.h>
#include <cmath>

#include <SmartDashboard/SmartDashboard.h>
#include <SmartDashboard/SendableChooser.h>

RobotLogic::RobotLogic(InputControl * control){
	this->control = control;
}

void RobotLogic::SetLeftMasterMotor(TalonSRX * leftMaster){
	this->leftMaster = leftMaster;
}
void RobotLogic::SetRightMasterMotor(TalonSRX * rightMaster){
	this->rightMaster = rightMaster;
}
void RobotLogic::SetLeftFollowerMotor(VictorSPX * leftFollower){
	this->leftFollower = leftFollower;
}
void RobotLogic::SetRightFollowerMotor(VictorSPX * rightFollower){
	this->rightFollower = rightFollower;
}
void RobotLogic::SetArmMotor(TalonSRX * arm){
	this->arm = arm;
}
void RobotLogic::SetLeftClawMotor(VictorSPX * leftClaw){
//void RobotLogic::SetLeftClawMotor(TalonSRX * leftClaw){
	this->leftClaw = leftClaw;
}
void RobotLogic::SetRightClawMotor(TalonSRX * rightClaw){
	this->rightClaw = rightClaw;
}
void RobotLogic::SetClaw(frc::DoubleSolenoid * claw){
	this->claw = claw;
}
void RobotLogic::SetPigeon(PigeonIMU * pigeon){
	this->pigeon = pigeon;
}

int RobotLogic::GetArmTarget(){
	return this->armTarget;
}

void RobotLogic::InitializeMotors(){
	int leftMasterID = this->leftMaster->GetDeviceID();
	int rightMasterID = this->rightMaster->GetDeviceID();
	leftFollower->Set(ControlMode::Follower, leftMasterID);
	rightFollower->Set(ControlMode::Follower, rightMasterID);
	leftMaster->EnableVoltageCompensation(true);
	rightMaster->EnableVoltageCompensation(true);
	leftFollower->EnableVoltageCompensation(true);
	rightFollower->EnableVoltageCompensation(true);
	arm->EnableVoltageCompensation(true);
	leftMaster->SetInverted(LEFT_INVERTED);
	leftFollower->SetInverted(LEFT_INVERTED);
	leftClaw->SetInverted(LEFT_CLAW_INVERTED);
	rightMaster->SetInverted(RIGHT_INVERTED);
	rightFollower->SetInverted(RIGHT_INVERTED);
	rightClaw->SetInverted(RIGHT_CLAW_INVERTED);
	arm->SetInverted(ARM_INVERTED);
}

void RobotLogic::SetBotParameters(double timeStep, double maxVel, double maxAcc, double maxJer, double wheelbaseWidth, double wheelCircumference){
	this->timeStep = timeStep;
	this->maxVel = maxVel;
	this->maxAcc = maxAcc;
	this->maxJer = maxJer;
	this->wheelbaseWidth = wheelbaseWidth;
	this->wheelCircumference = wheelCircumference;
}

void RobotLogic::ConfigureOpenRampTime(double leftRamptime, double rightRamptime, double armRamptime){
	this->leftMaster->ConfigOpenloopRamp(leftRamptime, this->timeoutMS);
	this->leftFollower->ConfigOpenloopRamp(leftRamptime, this->timeoutMS);
	this->rightMaster->ConfigOpenloopRamp(rightRamptime, this->timeoutMS);
	this->rightFollower->ConfigOpenloopRamp(rightRamptime, this->timeoutMS);
	this->leftClaw->ConfigOpenloopRamp(0.0, this->timeoutMS);
	this->rightClaw->ConfigOpenloopRamp(0.0, this->timeoutMS);
	this->arm->ConfigOpenloopRamp(armRamptime, this->timeoutMS);
}
void RobotLogic::ConfigureClosedRampTime(double leftRamptime, double rightRamptime, double armRamptime){
	this->leftMaster->ConfigClosedloopRamp(leftRamptime, this->timeoutMS);
	this->leftFollower->ConfigClosedloopRamp(leftRamptime, this->timeoutMS);
	this->rightMaster->ConfigClosedloopRamp(rightRamptime, this->timeoutMS);
	this->rightFollower->ConfigClosedloopRamp(rightRamptime, this->timeoutMS);
	this->leftClaw->ConfigClosedloopRamp(0.0, this->timeoutMS);
	this->rightClaw->ConfigClosedloopRamp(0.0, this->timeoutMS);
	this->arm->ConfigClosedloopRamp(armRamptime, this->timeoutMS);
}
void RobotLogic::ConfigureOpenRampTime(double ramptime){
	this->ConfigureOpenRampTime(ramptime, ramptime, ramptime);
}
void RobotLogic::ConfigureClosedRampTime(double ramptime){
	this->ConfigureClosedRampTime(ramptime, ramptime, ramptime);
}

void RobotLogic::ConfigureLeftPID(double F, double P, double I, double D){
	this->leftMaster->ConfigSelectedFeedbackSensor(QuadEncoder, this->PIDLoopIDX, this->timeoutMS);
	this->leftMaster->SetSensorPhase(false);
	this->leftMaster->SetSelectedSensorPosition(0, this->PIDLoopIDX, this->timeoutMS);
	this->leftMaster->ConfigNominalOutputForward(0, this->timeoutMS);
	this->leftMaster->ConfigNominalOutputReverse(0, this->timeoutMS);
	this->leftMaster->ConfigPeakOutputForward(1, this->timeoutMS);
	this->leftMaster->ConfigPeakOutputReverse(-1, this->timeoutMS);
	this->leftMaster->Config_kF(this->PIDLoopIDX, F, this->timeoutMS);
	this->leftMaster->Config_kP(this->PIDLoopIDX, P, this->timeoutMS);
	this->leftMaster->Config_kI(this->PIDLoopIDX, I, this->timeoutMS);
	this->leftMaster->Config_kD(this->PIDLoopIDX, D, this->timeoutMS);
}
void RobotLogic::ConfigureRightPID(double F, double P, double I, double D){
	this->rightMaster->ConfigSelectedFeedbackSensor(QuadEncoder, this->PIDLoopIDX, this->timeoutMS);
	this->rightMaster->SetSensorPhase(false);
	this->rightMaster->SetSelectedSensorPosition(0, this->PIDLoopIDX, this->timeoutMS);
	this->rightMaster->ConfigNominalOutputForward(0, this->timeoutMS);
	this->rightMaster->ConfigNominalOutputReverse(0, this->timeoutMS);
	this->rightMaster->ConfigPeakOutputForward(1, this->timeoutMS);
	this->rightMaster->ConfigPeakOutputReverse(-1, this->timeoutMS);
	this->rightMaster->Config_kF(this->PIDLoopIDX, F, this->timeoutMS);
	this->rightMaster->Config_kP(this->PIDLoopIDX, P, this->timeoutMS);
	this->rightMaster->Config_kI(this->PIDLoopIDX, I, this->timeoutMS);
	this->rightMaster->Config_kD(this->PIDLoopIDX, D, this->timeoutMS);
}
void RobotLogic::ConfigureArmPID(double F, double P, double I, double D){
	this->arm->ConfigSelectedFeedbackSensor(QuadEncoder, this->PIDLoopIDX, this->timeoutMS);
	this->arm->SetSensorPhase(ARM_ENCODER_PHASE);
	this->arm->SetSelectedSensorPosition(0, this->PIDLoopIDX, this->timeoutMS);
	this->arm->ConfigNominalOutputForward(0, this->timeoutMS);
	this->arm->ConfigNominalOutputReverse(0, this->timeoutMS);
	this->arm->ConfigPeakOutputForward(1, this->timeoutMS);
	this->arm->ConfigPeakOutputReverse(-1, this->timeoutMS);
	this->arm->Config_kF(this->PIDLoopIDX, F, this->timeoutMS);
	this->arm->Config_kP(this->PIDLoopIDX, P, this->timeoutMS);
	this->arm->Config_kI(this->PIDLoopIDX, I, this->timeoutMS);
	this->arm->Config_kD(this->PIDLoopIDX, D, this->timeoutMS);
}
void RobotLogic::ConfigureArmPIDSoftlimit(){
	this->arm->ConfigForwardSoftLimitThreshold(-135000, this->timeoutMS);
	this->arm->ConfigReverseSoftLimitThreshold(10000, this->timeoutMS);
	this->arm->ConfigForwardSoftLimitEnable(false, this->timeoutMS);
	this->arm->ConfigReverseSoftLimitEnable(false, this->timeoutMS);
}

void RobotLogic::ConfigureVision(char* str, double imgWidth){
	this->table = nt::NetworkTableInstance::GetDefault().GetTable(str);
	this->imgWidth = imgWidth;
}


void RobotLogic::SetMode(){
	if(this->control->GetButtonKill()){
		this->mode = KILLED;
	}

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
	this->clawEnabled = false;
}

void RobotLogic::ResetAuton(){

}
void RobotLogic::ResetPID(){

}

void RobotLogic::ResetArmPID(){
	this->arm->SetSelectedSensorPosition(0, this->PIDLoopIDX, this->timeoutMS);
}

void RobotLogic::Kill(){
	this->leftMaster->Set(ControlMode::PercentOutput, 0.0);
	this->rightMaster->Set(ControlMode::PercentOutput, 0.0);
	this->leftFollower->Set(ControlMode::PercentOutput, 0.0);
	this->rightFollower->Set(ControlMode::PercentOutput, 0.0);
	this->arm->Set(ControlMode::PercentOutput, 0.0);
	this->leftClaw->Set(ControlMode::PercentOutput, 0.0);
	this->rightClaw->Set(ControlMode::PercentOutput, 0.0);
	claw->Set(frc::DoubleSolenoid::Value::kOff);
}



void RobotLogic::ClawOpen(){
	this->claw->Set(frc::DoubleSolenoid::Value::kReverse);
	this->clawEnabled = true;
}
void RobotLogic::ClawClose(){
	this->claw->Set(frc::DoubleSolenoid::Value::kForward);
	this->clawEnabled = false;
}

void RobotLogic::ClawNeutralSuck(){
	leftClaw->Set(ControlMode::PercentOutput, 0.1);
	rightClaw->Set(ControlMode::PercentOutput, 0.1);
}
void RobotLogic::ClawSuck(){
	leftClaw->Set(ControlMode::PercentOutput, 0.6);
	rightClaw->Set(ControlMode::PercentOutput, 0.6);
}
void RobotLogic::ClawSpitSlow(){
	leftClaw->Set(ControlMode::PercentOutput, -0.6);
	rightClaw->Set(ControlMode::PercentOutput, -0.6);
}
void RobotLogic::ClawSpitFast(){
	leftClaw->Set(ControlMode::PercentOutput, -0.8);
	rightClaw->Set(ControlMode::PercentOutput, -0.8);
}

void RobotLogic::ClawSpitSpeed(double speed){
	leftClaw->Set(ControlMode::PercentOutput, speed);
	rightClaw->Set(ControlMode::PercentOutput, speed);
}

void RobotLogic::PIDSetPositionLow(){
	this->armTarget = PID_LOW_TARGET;
	this->control->SetThrottleSetPoint();
}
void RobotLogic::PIDSetPositionMid(){
	this->armTarget = PID_MID_TARGET;
	this->control->SetThrottleSetPoint();
}
void RobotLogic::PIDSetPositionHigh(){
	this->armTarget = PID_HIGH_TARGET;
	this->control->SetThrottleSetPoint();
}

void RobotLogic::PIDSetPositions(){
	if(this->control->GetButtonArmLow()){
		this->PIDSetPositionLow();
	}
	if (this->control->GetButtonArmMid()){
		this->PIDSetPositionMid();
	}
	if (this->control->GetButtonArmHigh()){
		this->PIDSetPositionHigh();
	}
}

void RobotLogic::PIDMoveArm(){
	/*if(control->GetMode() == DUAL_DRIVER_LOGITECH_THROTTLE){
		this->PIDMoveArmThrottle();
	} else {*/
	double change = this->control->GetAxisArmChange(); // GET RAW AXIS ARM NEEDS TO BE CHANGED WHEN ROBOTLOGIC IS IMPLEMENTED
	int temp = this->armTarget + change;
	if(!(temp > PID_INITIAL || temp < PID_MAX)){
		this->armTarget = temp;
	}
	this->arm->Set(ControlMode::Position, this->armTarget);
	//}
}

void RobotLogic::PIDMoveArmThrottle(){
	double change = this->control->GetAxisArmChange();
	this->arm->Set(ControlMode::Position, this->armTarget + change);
}

void RobotLogic::DirectDrive(){
	double leftThrottle = this->control->GetAxisLeftThrottle();
	double rightThrottle = this->control->GetAxisRightThrottle();
	double leftTurn = this->control->GetAxisLeftTurn();
	double rightTurn = this->control->GetAxisRightTurn();
	this->leftMaster->Set(ControlMode::PercentOutput, leftThrottle + leftTurn);
	this->leftFollower->Set(ControlMode::PercentOutput, leftThrottle + leftTurn);
	this->rightMaster->Set(ControlMode::PercentOutput, rightThrottle - rightTurn);
	this->rightFollower->Set(ControlMode::PercentOutput, rightThrottle - rightTurn);
}

void RobotLogic::DirectMoveArm(){
	/*if(this->control->GetMode() == JOYSTICK_THROTTLE){
		int armTarget = (int) this->control->GetRawAxisArm();
		this->arm->Set(ControlMode::Position, armTarget);
	} else {*/
	double armPower = this->control->GetRawAxisArm();
	this->arm->Set(ControlMode::PercentOutput, armPower);
	//}
}

void RobotLogic::DirectControlClaw(){
	//Claw Pneumatics
	if(control->GetButtonClawTogglePressed()){
		this->clawEnabled = !this->clawEnabled;
	}

	if(control->GetButtonClawClose()){
		clawEnabled = false;
	}
	if(control->GetButtonClawOpen()){
		clawEnabled = true;
	}

	if(clawEnabled){
		this->ClawOpen();
	} else {
		this->ClawClose();
	}

	//Claw Motors
	if(control->GetButtonClawSuck() || control->GetButtonClawToggle() || control->GetButtonClawClose()){
		this->ClawSuck();
	} else if(control->GetButtonClawSpitFast()){
		this->ClawSpitFast();
	} else if(control->GetButtonClawSpitSlow()){
		this->ClawSpitSlow();
	} else{
		this->ClawNeutralSuck();
	}
}

double RobotLogic::Mod(double a, double modulus){
	double result = a;
	while(result > modulus){
		result -= modulus;
	}
	while(result < 0){
		result += modulus;
	}
	return result;
}

void RobotLogic::DriveOff(){
	this->leftMaster->Set(ControlMode::PercentOutput, 0.0);
	this->leftFollower->Set(ControlMode::PercentOutput, 0.0);
	this->rightMaster->Set(ControlMode::PercentOutput, 0.0);
	this->rightFollower->Set(ControlMode::PercentOutput, 0.0);
}

double RobotLogic::AngleDifference(double desired, double current){
	double diff = (desired > current ? desired - current : current - desired);
	double mod_diff = std::fmod(diff, 360);
	double distance = (mod_diff < 180 ? mod_diff : 360 - mod_diff);

	double sgn_diff = this->Mod(desired, 360) - this->Mod(current, 360);
	int sign = (sgn_diff >= 0 ? 1 : -1);
	if (sgn_diff < 180.0 && sgn_diff > -180.0){
		sign = sign * -1;
	}
	return distance*sign;
}

void RobotLogic::AutonInitEncoders(){
	//this->leftMaster->SetSelectedSensorPosition(0, this->PIDLoopIDX, this->timeoutMS);
	//this->rightMaster->SetSelectedSensorPosition(0, this->PIDLoopIDX, this->timeoutMS);
	this->leftEncoder = new EncoderFollower();
	this->rightEncoder = new EncoderFollower();

	this->leftEncoder->last_error = 0;
	this->leftEncoder->segment = 0;
	this->leftEncoder->finished = 0;
	this->rightEncoder->last_error = 0;
	this->rightEncoder->segment = 0;
	this->rightEncoder->finished = 0;

	this->leftEncoderConfig = new EncoderConfig();
	this->rightEncoderConfig = new EncoderConfig();

	this->leftEncoderConfig->initial_position = this->leftMaster->GetSelectedSensorPosition(this->PIDLoopIDX);
	this->leftEncoderConfig->ticks_per_revolution = 1024;
	this->leftEncoderConfig->wheel_circumference = this->wheelCircumference;
	this->leftEncoderConfig->kp = 0.1;
	this->leftEncoderConfig->ki = 0.0;
	this->leftEncoderConfig->kd = 0.0;
	this->leftEncoderConfig->kv = 1.0 / this->maxVel;
	this->leftEncoderConfig->ka = 0.0;
	this->rightEncoderConfig->initial_position = this->rightMaster->GetSelectedSensorPosition(this->PIDLoopIDX);
	this->rightEncoderConfig->ticks_per_revolution = 1024;
	this->rightEncoderConfig->wheel_circumference = this->wheelCircumference;
	this->rightEncoderConfig->kp = 0.1;
	this->rightEncoderConfig->ki = 0.0;
	this->rightEncoderConfig->kd = 0.0;
	this->rightEncoderConfig->kv = 1.0 / this->maxVel;
	this->rightEncoderConfig->ka = 0.0;

	this->pigeonReference = this->pigeonReference + this->pigeon->GetFusedHeading();
	this->pigeon->SetFusedHeading(0.0, this->timeoutMS);
}

void RobotLogic::AutonFollowTrajectory(Segment * leftTrajectory, Segment * rightTrajectory, int trajectoryLength){
	double l = pathfinder_follow_encoder(*leftEncoderConfig, leftEncoder, leftTrajectory, trajectoryLength, this->leftMaster->GetSelectedSensorPosition(this->PIDLoopIDX));
	double r = pathfinder_follow_encoder(*rightEncoderConfig, rightEncoder, rightTrajectory, trajectoryLength, this->rightMaster->GetSelectedSensorPosition(this->PIDLoopIDX));

	double gyroHeading = this->pigeon->GetFusedHeading();
	double desiredHeading = r2d(this->leftEncoder->heading);
	double angleDifference = std::fmod(desiredHeading - gyroHeading, 360.0);
	if(angleDifference > 180.0){
		angleDifference = angleDifference - 360;
	}
	//double angleDifference = desiredHeading - gyroHeading;

	double turn = 0.4 * (-1.0/80) * angleDifference;

	leftOutput = l + turn;
	rightOutput = r - turn;

	this->leftMaster->Set(ControlMode::PercentOutput, l + turn);
	this->leftFollower->Set(ControlMode::PercentOutput, l + turn);
	this->rightMaster->Set(ControlMode::PercentOutput, r - turn);
	this->rightFollower->Set(ControlMode::PercentOutput, r - turn);
}

void RobotLogic::AutonFollowReverseTrajectory(Segment * leftTrajectory, Segment * rightTrajectory, int trajectoryLength){
	double r = pathfinder_follow_encoder(*leftEncoderConfig, leftEncoder, leftTrajectory, trajectoryLength, -1*this->leftMaster->GetSelectedSensorPosition(this->PIDLoopIDX));
	double l = pathfinder_follow_encoder(*rightEncoderConfig, rightEncoder, rightTrajectory, trajectoryLength, -1*this->rightMaster->GetSelectedSensorPosition(this->PIDLoopIDX));

	double gyroHeading = this->pigeon->GetFusedHeading();
	double desiredHeading = r2d(this->leftEncoder->heading);
	double angleDifference = std::fmod(desiredHeading - gyroHeading, 360.0);
	if(angleDifference > 180.0){
		angleDifference = angleDifference - 360;
	}
	//double angleDifference = desiredHeading - gyroHeading;

	double turn = 0.4 * (-1.0/80) * angleDifference;


	leftOutput = -1*l + turn;
	rightOutput = -1*r - turn;

	this->leftMaster->Set(ControlMode::PercentOutput, leftOutput);
	this->leftFollower->Set(ControlMode::PercentOutput, leftOutput);
	this->rightMaster->Set(ControlMode::PercentOutput, rightOutput);
	this->rightFollower->Set(ControlMode::PercentOutput, rightOutput);
}

void RobotLogic::AutonSetBearing(int degrees){
	this->targetBearing = degrees - this->pigeonReference;
}

void RobotLogic::AutonTurn(){
	this->AutonTurn(this->targetBearing);
}

void RobotLogic::AutonTurn(int degrees){
	double gyroHeading = this->pigeon->GetFusedHeading();
	//double desiredHeading = degrees - this->Mod(this->pigeonReference, 360);
	double desiredHeading = degrees;
	/*double angleDifference = std::fmod(desiredHeading - gyroHeading, 360.0);
		if(angleDifference > 180.0){
			angleDifference = angleDifference - 360;
		}*/
	//double angleDifference = desiredHeading - gyroHeading;
	double angleDifference = this->AngleDifference(desiredHeading, gyroHeading);
	this->angleDifference = angleDifference;
	double derivative = (angleDifference - this->turnPrevError);
	double turn = 3.0/80 * angleDifference + 25.0/80 * derivative;
	this->turnPrevError = angleDifference;

	this->leftOutput = turn;
	this->rightOutput = -turn;

	this->leftMaster->Set(ControlMode::PercentOutput, turn);
	this->leftFollower->Set(ControlMode::PercentOutput, turn);
	this->rightMaster->Set(ControlMode::PercentOutput, -turn);
	this->rightFollower->Set(ControlMode::PercentOutput, -turn);
}

void RobotLogic::AutonMoveArm(){
	this->arm->Set(ControlMode::Position, this->armTarget);
}

void RobotLogic::AutonMoveArm(int target, int multiplier){
	if(abs(this->armTarget - target) > PID_ARM_MULTIPLIER * multiplier){
		int dir = 1;
		if (target < this->armTarget){
			dir = -1;
		}
		this->armTarget = this->armTarget + dir*PID_ARM_MULTIPLIER * multiplier;
	}
	this->AutonMoveArm();
}

void RobotLogic::AutonFreeEncoders(){
	delete this->leftEncoder;
	delete this->rightEncoder;
	delete this->leftEncoderConfig;
	delete this->rightEncoderConfig;
}

void RobotLogic::AutonCleanTrajectory(){
	//free(trajectory);
	//free(leftTrajectory);
	//free(rightTrajectory);
}

double RobotLogic::AutonVisionTrack(){
	std::vector<double> contour = table.get()->GetNumberArray("Contour", llvm::ArrayRef<double>());
	double turn = 0.0;
	if(contour.size() > 0){
		double x = contour[0];
		double w = contour[2];
		double rectCenterX = (x + w/2.0 - this->imgWidth/2.0)/(imgWidth/2.0);

		turn = rectCenterX * 0.25;
	}
	this->leftMaster->Set(ControlMode::PercentOutput, turn);
	this->leftFollower->Set(ControlMode::PercentOutput, turn);
	this->rightMaster->Set(ControlMode::PercentOutput, -turn);
	this->rightFollower->Set(ControlMode::PercentOutput, -turn);

	return turn;
}

double RobotLogic::AutonVisionFollow(double speed){
	std::vector<double> contour = table.get()->GetNumberArray("Contour", llvm::ArrayRef<double>());
	double turn = 0.0;
	if(contour.size() > 0){
		double x = contour[0];
		double w = contour[2];
		double rectCenterX = (x + w/2.0 - this->imgWidth/2.0)/(imgWidth/2.0);

		turn = rectCenterX * 0.10;
	}
	this->leftMaster->Set(ControlMode::PercentOutput, speed + turn);
	this->leftFollower->Set(ControlMode::PercentOutput, speed + turn);
	this->rightMaster->Set(ControlMode::PercentOutput, speed - turn);
	this->rightFollower->Set(ControlMode::PercentOutput, speed - turn);

	return turn;
}

void RobotLogic::AutonFollow(double speed){
	int degrees = this->targetBearing;
	double gyroHeading = this->pigeon->GetFusedHeading();
	//double desiredHeading = degrees - this->Mod(this->pigeonReference, 360);
	double desiredHeading = degrees;
	/*double angleDifference = std::fmod(desiredHeading - gyroHeading, 360.0);
		if(angleDifference > 180.0){
			angleDifference = angleDifference - 360;
		}*/
	//double angleDifference = desiredHeading - gyroHeading;
	double angleDifference = this->AngleDifference(desiredHeading, gyroHeading);
	this->angleDifference = angleDifference;
	double derivative = (angleDifference - this->turnPrevError);
	double turn = 3.0/80 * angleDifference + 25.0/80 * derivative;
	this->turnPrevError = angleDifference;

	this->leftOutput = turn;
	this->rightOutput = -turn;

	this->leftMaster->Set(ControlMode::PercentOutput, speed + turn);
	this->leftFollower->Set(ControlMode::PercentOutput, speed + turn);
	this->rightMaster->Set(ControlMode::PercentOutput, speed - turn);
	this->rightFollower->Set(ControlMode::PercentOutput, speed - turn);
}

RobotLogic::~RobotLogic() {
	// TODO Auto-generated destructor stub
}

