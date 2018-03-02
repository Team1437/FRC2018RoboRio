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
	leftMaster->SetInverted(true);
	leftFollower->SetInverted(true);
	leftClaw->SetInverted(true);
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
	this->arm->SetSensorPhase(false);
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
	this->clawWrist->Set(frc::DoubleSolenoid::Value::kOff);
	this->clawEnabled = false;
	this->clawWristEnabled = false;
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
	clawWrist->Set(frc::DoubleSolenoid::Value::kOff);
}

void RobotLogic::PIDSetPositions(){
	int low = -5;
	int mid = -330;
	int high = -425;
	if(this->control->GetButtonArmLow()){
		this->armTarget = low;
		this->control->SetThrottleSetPoint();
	}
	if (this->control->GetButtonArmMid()){
		this->armTarget = mid;
		this->control->SetThrottleSetPoint();
	}
	if (this->control->GetButtonArmHigh()){
		this->armTarget = high;
		this->control->SetThrottleSetPoint();
	}
}

void RobotLogic::PIDMoveArm(){
	if(control->GetMode() == DUAL_DRIVER_LOGITECH_THROTTLE){
		this->PIDMoveArmThrottle();
	} else {
		double change = this->control->GetAxisArmChange(); // GET RAW AXIS ARM NEEDS TO BE CHANGED WHEN ROBOTLOGIC IS IMPLEMENTED
		this->armTarget = this->armTarget + change;
		this->arm->Set(ControlMode::Position, this->armTarget);
	}
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
	this->leftMaster->Set(ControlMode::PercentOutput, leftThrottle - leftTurn);
	this->leftFollower->Set(ControlMode::PercentOutput, leftThrottle - leftTurn);
	this->rightMaster->Set(ControlMode::PercentOutput, rightThrottle + rightTurn);
	this->rightFollower->Set(ControlMode::PercentOutput, rightThrottle + rightTurn);
}

void RobotLogic::DirectMoveArm(){
	if(this->control->GetMode() == JOYSTICK_THROTTLE){
		int armTarget = (int) this->control->GetRawAxisArm();
		this->arm->Set(ControlMode::Position, armTarget);
	} else {
		double armPower = this->control->GetRawAxisArm();
		this->arm->Set(ControlMode::PercentOutput, armPower);
	}
}

void RobotLogic::DirectControlClaw(){
	//Claw Pneumatics
	if(control->GetButtonClawTogglePressed()){
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
		this->clawWrist->Set(frc::DoubleSolenoid::Value::kReverse);
	} else {
		this->clawWrist->Set(frc::DoubleSolenoid::Value::kForward);
	}

	//Claw Motors
	if(control->GetButtonClawSuck() || control->GetButtonClawToggle()){
		leftClaw->Set(ControlMode::PercentOutput, 0.5);
		rightClaw->Set(ControlMode::PercentOutput, 0.5);
	} else if(control->GetButtonClawSpitFast()){
		leftClaw->Set(ControlMode::PercentOutput, -1.0);
		rightClaw->Set(ControlMode::PercentOutput, -1.0);
	} else if(control->GetButtonClawSpitSlow()){
		leftClaw->Set(ControlMode::PercentOutput, -0.8);
		rightClaw->Set(ControlMode::PercentOutput, -0.8);
	} else{
		leftClaw->Set(ControlMode::PercentOutput, 0.1);
		rightClaw->Set(ControlMode::PercentOutput, 0.1);
	}
}

void RobotLogic::AutonInitEncoders(){
	//leftEncoder = malloc(sizeof(EncoderFollower));
	//rightEncoder = malloc(sizeof(EncoderFollower));
	leftEncoder.last_error = 0;
	leftEncoder.segment = 0;
	leftEncoder.finished = 0;
	rightEncoder.last_error = 0;
	rightEncoder.segment = 0;
	rightEncoder.finished = 0;

	leftEncoderConfig.initial_position = this->leftMaster->GetSelectedSensorPosition(this->PIDLoopIDX);
	leftEncoderConfig.ticks_per_revolution = 4096;
	leftEncoderConfig.wheel_circumference = this->wheelCircumference;
	leftEncoderConfig.kp = 1.0;
	leftEncoderConfig.ki = 0.0;
	leftEncoderConfig.kp = 0.0;
	leftEncoderConfig.kv = 1.0 / this->maxVel;
	leftEncoderConfig.ka = 0.0;
	rightEncoderConfig.initial_position = this->rightMaster->GetSelectedSensorPosition(this->PIDLoopIDX);
	rightEncoderConfig.ticks_per_revolution = 4096;
	rightEncoderConfig.wheel_circumference = this->wheelCircumference;
	rightEncoderConfig.kp = 1.0;
	rightEncoderConfig.ki = 0.0;
	rightEncoderConfig.kp = 0.0;
	rightEncoderConfig.kv = 1.0 / this->maxVel;
	rightEncoderConfig.ka = 0.0;
}

void RobotLogic::AutonFollowTrajectory(){
	double l = pathfinder_follow_encoder(leftEncoderConfig, &leftEncoder, leftTrajectory, trajectoryLength, this->leftMaster->GetSelectedSensorPosition(this->PIDLoopIDX));
	double r = pathfinder_follow_encoder(rightEncoderConfig, &rightEncoder, rightTrajectory, trajectoryLength, this->rightMaster->GetSelectedSensorPosition(this->PIDLoopIDX));
	this->leftMaster->Set(ControlMode::PercentOutput, l);
	this->rightMaster->Set(ControlMode::PercentOutput, r);
}

void RobotLogic::AutonBuildTrajectory(Waypoint * points, int numPoints){
	TrajectoryCandidate candidate;
	pathfinder_prepare(points, numPoints, FIT_HERMITE_CUBIC, PATHFINDER_SAMPLES_HIGH, this->timeStep, this->maxVel, this->maxAcc, this->maxJer, &candidate);
	trajectoryLength = candidate.length;
	trajectory = (Segment*)malloc(trajectoryLength * sizeof(Segment));
	leftTrajectory = (Segment*)malloc(trajectoryLength * sizeof(Segment));
	rightTrajectory = (Segment*)malloc(trajectoryLength * sizeof(Segment));
	pathfinder_generate(&candidate, trajectory);
	pathfinder_modify_tank(trajectory, trajectoryLength, leftTrajectory, rightTrajectory, this->wheelbaseWidth);
}

void RobotLogic::AutonFreeEncoders(){
	//free(this->leftEncoder);
	//free(this->rightEncoder);
}

void RobotLogic::AutonCleanTrajectory(){
	free(trajectory);
	free(leftTrajectory);
	free(rightTrajectory);
}

RobotLogic::~RobotLogic() {
	// TODO Auto-generated destructor stub
}

