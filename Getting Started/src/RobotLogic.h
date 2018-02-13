/*
 * RobotLogic.h
 *
 *  Created on: Feb 10, 2018
 *      Author: PatriotRobotics
 */

#include <InputControl.h>
#include <DoubleSolenoid.h>

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

	int armTarget = 0;
	int leftTarget = 0;
	int rightTarget = 0;

	TalonSRX * left = NULL;
	TalonSRX * right = NULL;
	TalonSRX * arm = NULL;
	TalonSRX * leftClaw = NULL;
	TalonSRX * rightClaw = NULL;

	frc::DoubleSolenoid * claw = NULL;
	frc::DoubleSolenoid * clawWrist = NULL;

	InputControl * control = NULL;
public:
	RobotLogic(InputControl *);

	void SetLeftMotor(TalonSRX *);
	void SetRightMotor(TalonSRX *);
	void SetArmMotor(TalonSRX *);
	void SetLeftClawMotor(TalonSRX *);
	void SetRightClawMotor(TalonSRX *);
	void SetClaw(frc::DoubleSolenoid *);
	void SetClawWrist(frc::DoubleSolenoid *);

	void SetMode();

	void ResetClaw();
	void ResetAuton();
	void ResetPID();
	void Kill();

	void PIDMoveArm(int);

	void DirectDrive();
	void DirectMoveArm();

	void DirectControlClaw();

	virtual ~RobotLogic();
};

#endif /* SRC_ROBOTLOGIC_H_ */
