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

class RobotLogic {
private:
	int mode = DIRECT;
	bool clawEnabled = false;
	bool clawWristEnabled = false;

	bool throttleChanged = false;

	int armTarget = -15;
	int leftTarget = 0;
	int rightTarget = 0;

	int timeoutMS = 10;

	int PIDLoopIDX = 0;

	double timeStep = 0.0;
	double maxVel = 0.0;
	double maxAcc = 0.0;
	double maxJer = 0.0;
	double wheelbaseWidth = 0.0;
	double wheelCircumference = 0.0;

	TalonSRX * leftMaster = NULL;
	TalonSRX * rightMaster = NULL;
	VictorSPX * leftFollower = NULL;
	VictorSPX * rightFollower = NULL;
	TalonSRX * arm = NULL;
	VictorSPX * leftClaw = NULL;
	TalonSRX * rightClaw = NULL;

	frc::DoubleSolenoid * claw = NULL;
	frc::DoubleSolenoid * clawWrist = NULL;

	InputControl * control = NULL;

	int trajectoryLength = 0;
	Segment * trajectory;
	Segment * leftTrajectory;
	Segment * rightTrajectory;

	EncoderFollower leftEncoder;
	EncoderFollower rightEncoder;
	EncoderConfig leftEncoderConfig;
	EncoderConfig rightEncoderConfig;
public:
	RobotLogic(InputControl *);

	void SetLeftMasterMotor(TalonSRX *);
	void SetRightMasterMotor(TalonSRX *);
	void SetLeftFollowerMotor(VictorSPX *);
	void SetRightFollowerMotor(VictorSPX *);
	void SetArmMotor(TalonSRX *);
	void SetLeftClawMotor(VictorSPX *);
	void SetRightClawMotor(TalonSRX *);
	void SetClaw(frc::DoubleSolenoid *);
	void SetClawWrist(frc::DoubleSolenoid *);

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

	void PIDSetPositions();
	void PIDMoveArm();
	void PIDMoveArmThrottle();

	void DirectDrive();
	void DirectMoveArm();
	void DirectControlClaw();

	void AutonInitEncoders();
	void AutonBuildTrajectory(Waypoint *, int);
	void AutonFollowTrajectory();
	void AutonFreeEncoders();
	void AutonCleanTrajectory();

	virtual ~RobotLogic();
};

#endif /* SRC_ROBOTLOGIC_H_ */
