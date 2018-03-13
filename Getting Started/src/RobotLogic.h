/*
 * RobotLogic.h
 *
 *  Created on: Feb 10, 2018
 *      Author: PatriotRobotics
 */

#include <InputControl.h>
#include <DoubleSolenoid.h>
#include <pathfinder.h>

#ifndef SRC_ROBOTLOGIC_H_
#define SRC_ROBOTLOGIC_H_

#define KILLED 0
#define DIRECT 1
#define AUTON 2
#define PID 3

//SET THIS DEFINE TO EITHER PRACTICE_BOT OR COMPETITION_BOT
#define PRACTICE_BOT

#ifdef PRACTICE_BOT
#define PID_INITIAL 0
#define PID_LOW_TARGET -10000
// PID AUTON SCORE TARGET -20000 or -35000?
#define PID_AUTO_TARGET -30000
#define PID_MID_TARGET -90000
#define PID_HIGH_TARGET -125000
#define PID_ARM_MULTIPLIER 500.0
#define LEFT_MASTER_ID 1
#define LEFT_FOLLOW_ID 4
#define RIGHT_MASTER_ID 3
#define RIGHT_FOLLOW_ID 2
#define ARM_ID 50
#define LEFT_CLAW_ID 11
#define RIGHT_CLAW_ID 10
#define CLAW_PCM_ID_1 0
#define CLAW_PCM_ID_2 1
#define CLAW_WRIST_PCM_ID_1 4
#define CLAW_WRIST_PCM_ID_2 5
#define ARM_P 0.1
#define ARM_I 0.00001
#define ARM_D 15.0
#define LEFT_INVERTED false
#define LEFT_CLAW_INVERTED true
#define RIGHT_INVERTED true
#define RIGHT_CLAW_INVERTED false
#define ARM_INVERTED false
#define ARM_ENCODER_PHASE true
#endif

#ifdef COMPETITION_BOT
#define PID_INITIAL -15
#define PID_LOW_TARGET -5
#define PID_MID_TARGET -330
#define PID_HIGH_TARGET -420
#define PID_ARM_MULTIPLIER 6.0
#define LEFT_MASTER_ID 4
#define LEFT_FOLLOW_ID 3
#define RIGHT_MASTER_ID 8
#define RIGHT_FOLLOW_ID 7
#define ARM_ID 6
#define LEFT_CLAW_ID 5
#define RIGHT_CLAW_ID 2
#define CLAW_PCM_ID_1 0
#define CLAW_PCM_ID_2 1
#define CLAW_WRIST_PCM_ID_1 2
#define CLAW_WRIST_PCM_ID_2 3
#define ARM_P 8.0
#define ARM_I 0.001
#define ARM_D 1200.0
#define LEFT_INVERTED true
#define LEFT_CLAW_INVERTED true
#define RIGHT_INVERTED false
#define RIGHT_CLAW_INVERTED false
#define ARM_INVERTED false
#define ARM_ENCODER_PHASE false
#endif

class RobotLogic {
private:
	int mode = DIRECT;
	bool clawEnabled = false;
	bool clawWristEnabled = true;

	int timeoutMS = 10;
	int PIDLoopIDX = 0;

	TalonSRX * leftMaster = NULL;
	TalonSRX * rightMaster = NULL;
	VictorSPX * leftFollower = NULL;
	VictorSPX * rightFollower = NULL;
	TalonSRX * arm = NULL;
	TalonSRX * leftClaw = NULL;
	TalonSRX * rightClaw = NULL;

	frc::DoubleSolenoid * claw = NULL;
	frc::DoubleSolenoid * clawWrist = NULL;

	InputControl * control = NULL;

public:
	double leftOutput = 0;
	double rightOutput = 0;

	int armTarget = PID_INITIAL;

	double timeStep = 0.0;
	double maxVel = 0.0;
	double maxAcc = 0.0;
	double maxJer = 0.0;
	double wheelbaseWidth = 0.0;
	double wheelCircumference = 0.0;

	double pigeonReference = 0.0;
	double targetBearing = 0.0;
	double angleDifference = 0.0;

	double turnPrevError = 0.0;
	//double turnPrevTime = 0.0;

	PigeonIMU * pigeon = NULL;

	EncoderFollower * leftEncoder;
	EncoderFollower * rightEncoder;
	EncoderConfig * leftEncoderConfig;
	EncoderConfig * rightEncoderConfig;

	RobotLogic(InputControl *);

	void SetLeftMasterMotor(TalonSRX *);
	void SetRightMasterMotor(TalonSRX *);
	void SetLeftFollowerMotor(VictorSPX *);
	void SetRightFollowerMotor(VictorSPX *);
	void SetArmMotor(TalonSRX *);
	void SetLeftClawMotor(TalonSRX *);
	void SetRightClawMotor(TalonSRX *);
	void SetClaw(frc::DoubleSolenoid *);
	void SetClawWrist(frc::DoubleSolenoid *);
	void SetPigeon(PigeonIMU *);

	int GetArmTarget();

	void InitializeMotors();
	void SetBotParameters(double, double, double, double, double, double);
	void ConfigureOpenRampTime(double, double, double);
	void ConfigureClosedRampTime(double, double, double);
	void ConfigureOpenRampTime(double);
	void ConfigureClosedRampTime(double);

	void ConfigureLeftPID(double, double, double, double);
	void ConfigureRightPID(double, double, double, double);
	void ConfigureArmPID(double, double, double, double);
	void ConfigureArmPIDSoftlimit();

	void SetMode();

	void ResetClaw();
	void ResetAuton();
	void ResetPID();
	void ResetArmPID();
	void Kill();

	void ClawClose();
	void ClawOpen();
	void ClawWristRetract();
	void ClawWristExtend();
	void ClawNeutralSuck();
	void ClawSuck();
	void ClawSpitSlow();
	void ClawSpitFast();

	void PIDSetPositionLow();
	void PIDSetPositionMid();
	void PIDSetPositionHigh();
	void PIDSetPositions();
	void PIDMoveArm();
	void PIDMoveArmThrottle();

	void DirectDrive();
	void DirectMoveArm();
	void DirectControlClaw();

	double Mod(double, double);
	double AngleDifference(double, double);

	void DriveOff();

	void AutonInitEncoders();
	void AutonFollowTrajectory(Segment *, Segment *, int);
	void AutonFollowReverseTrajectory(Segment *, Segment *, int);
	void AutonSetBearing(int);
	void AutonTurn();
	void AutonTurn(int);
	void AutonMoveArm();
	void AutonMoveArm(int, int);
	void AutonFreeEncoders();
	void AutonCleanTrajectory();

	virtual ~RobotLogic();
};

#endif /* SRC_ROBOTLOGIC_H_ */
