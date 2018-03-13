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

/*#define ARCADE_JOYSTICK 0
#define ARCADE_PS4 1
#define TANK_JOYSTICK 2
#define TANK_PS4 3
#define DUAL_JOYSTICK 4
#define JOYSTICK_THROTTLE 5
#define ARCADE_LOGITECH 6
#define TANK_LOGITECH 7
#define DUAL_DRIVER_LOGITECH_THROTTLE 8*/
#define FINAL_CONTROL 9
#define FINAL_CONTROL_SINGLE_ARCADE 10

class InputControl {
private:

	double leftThrottleMult = 1.0;
	double rightThrottleMult = 1.0;
	double leftTurnMult = 1.0;
	double rightTurnMult = 1.0;
	double armMult = 1.0;

	int mode;

	double throttleSetPoint = 0.0;

public:
	Joystick * joy1 = new Joystick(0);
	Joystick * joy2 = joy1;

	int armTarget = 0;

	InputControl(int);
	virtual ~InputControl();

	void SetLeftThrottleMult(double);
	void SetRightThrottleMult(double);
	void SetLeftTurnMult(double);
	void SetRightTurnMult(double);
	void SetDriveMultipliers(double, double, double, double);
	void SetArmMultiplier(double);

	void SetThrottleSetPoint();

	double GetRawAxisLeftThrottle();
	double GetRawAxisRightThrottle();
	double GetRawAxisLeftTurn();
	double GetRawAxisRightTurn();
	double GetRawAxisArm();

	double GetAxisLeftThrottle();
	double GetAxisRightThrottle();
	double GetAxisLeftTurn();
	double GetAxisRightTurn();
	int GetAxisArmChange();

	bool GetButtonClawToggle();
	bool GetButtonClawTogglePressed();
	bool GetButtonClawWristToggle();
	bool GetButtonClawSuck();
	bool GetButtonClawSpitFast();
	bool GetButtonClawSpitSlow();
	bool GetButtonKill();
	bool GetButtonArmCalibrate();
	bool GetButtonAuton();
	bool GetButtonPID();

	void ToggleMode();

	bool GetButtonArmLow();
	bool GetButtonArmMid();
	bool GetButtonArmHigh();

	int GetMode();
};

#endif /* SRC_INPUTCONTROL_H_ */
