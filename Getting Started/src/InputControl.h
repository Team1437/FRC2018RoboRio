/*
 * InputControl.h
 *
 *  Created on: Feb 8, 2018
 *      Author: PatriotRobotics
 */

#include <Joystick.h>
#include "ctre/Phoenix.h"

#ifndef SRC_INPUTCONTROL_H_
#define SRC_INPUTCONTROL_H_

#define ARCADE_JOYSTICK 0
#define ARCADE_PS4 1
#define TANK_JOYSTICK 2
#define TANK_PS4 3
#define DUAL_JOYSTICK 4
#define JOYSTICK_THROTTLE 5

class InputControl {
private:
	TalonSRX * left = NULL;
	TalonSRX * right = NULL;
	TalonSRX * arm = NULL;
	TalonSRX * leftClaw = NULL;
	TalonSRX * rightClaw = NULL;

	double leftThrottleMult = 1.0;
	double rightThrottleMult = 1.0;
	double leftTurnMult = 1.0;
	double rightTurnMult = 1.0;
	double armMult = 1.0;

	int mode;

public:
	Joystick * joy1 = new Joystick(0);
	Joystick * joy2 = joy1;

	int armTarget = 0;

	InputControl(int);
	virtual ~InputControl();

	void SetLeftMotor(TalonSRX *);
	void SetRightMotor(TalonSRX *);
	void SetArmMotor(TalonSRX *);
	void SetLeftClawMotor(TalonSRX *);
	void SetRightClawMotor(TalonSRX *);
	void SetLeftThrottleMult(double);
	void SetRightThrottleMult(double);
	void SetLeftTurnMult(double);
	void SetRightTurnMult(double);
	void SetDriveMultipliers(double, double, double, double);
	void SetArmMultiplier(double);

	double GetRawAxisLeftThrottle();
	double GetRawAxisRightThrottle();
	double GetRawAxisLeftTurn();
	double GetRawAxisRightTurn();
	double GetRawAxisArm();

	double GetAxisLeftThrottle();
	double GetAxisRightThrottle();
	double GetAxisLeftTurn();
	double GetAxisRightTurn();
	double GetAxisArm();

	bool GetButtonClawToggle();
	bool GetButtonClawWristToggle();
	bool GetButtonClawSuck();
	bool GetButtonClawSpit();
	bool GetButtonKill();
	bool GetButtonArmCalibrate();
	bool GetButtonAuton();
	bool GetButtonPID();

	bool GetButtonLower();
	bool GetButtonRaise();

	int GetMode();

	void Drive();
	void MoveArm();
};

#endif /* SRC_INPUTCONTROL_H_ */
