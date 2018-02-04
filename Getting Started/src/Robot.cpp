/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <Drive/DifferentialDrive.h>
#include <IterativeRobot.h>
#include <Joystick.h>
#include <LiveWindow/LiveWindow.h>
#include <Spark.h>
#include <Timer.h>
#include <CameraServer.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <DigitalInput.h>
#include <iostream>

#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>

#include "ctre/Phoenix.h"

class Robot : public frc::IterativeRobot {
private:
	//Unused initial variables
	//frc::Joystick m_stick{0};
	frc::LiveWindow& m_lw = *frc::LiveWindow::GetInstance();
	//frc::Timer m_timer;

	//Variable declaration for camera and NetworkTable
	cs::UsbCamera cam;
	std::shared_ptr<nt::NetworkTable> table;

	//Variable declaration for all the motors
	TalonSRX * leftFront;
	TalonSRX * rightFront;
	VictorSPX * leftRear;
	VictorSPX * rightRear;
	TalonSRX * arm;

	//Declaration for the joystick and a button that is connected to a DigitalInput port on the RoboRIO
	Joystick * joy;
	frc::DigitalInput * limitSwitch;

	//Variable declaration for variables that are used while the robot is running
	bool auton;
	std::vector<double> contour;
	double imgWidth;

	double previous[4];
	int counter;

	bool editTarget;
	bool PIDControl;

	int armTarget;
	int leftTarget;
	int rightTarget;

	bool killed;
public:
	//This function runs every time the robot first turns on, will not run again until it is turned off and back on again
	Robot() {

		//Initialize the motors with their respective ID's on the CAN bus
		leftFront = new TalonSRX(1);
		rightFront = new TalonSRX(3);
		leftRear = new VictorSPX(4);
		rightRear = new VictorSPX(2);
		arm = new TalonSRX(50);

		joy = new Joystick(0);

		//Start the camera stream with a resolution of 640x480
		cam = CameraServer::GetInstance()->StartAutomaticCapture();
		cam.SetResolution(640, 480);
		//CameraServer::GetInstance()->PutVideo("Rectangle", 640, 480);
		limitSwitch = new frc::DigitalInput(0);

		//Motor configuration:
		// -- Make sure the motors all turn the right directions'
		// -- Voltage compensation to have Set(ControlMode::PercentOutput, 0.1) tell the motors
		//    to go to 10% of total output rather than 10% of maximum voltage (10% of max. voltage
		//    would not actually turn on the motors)
		// -- Set the rear motors to mimic any commands sent to the front motors, this is done since
		//    all the motors on one side are connected to the same chain and need to be doign the same
		//    commands as each other
		leftFront->SetInverted(true);
		leftRear->SetInverted(true);

		leftFront->EnableVoltageCompensation(true);
		rightFront->EnableVoltageCompensation(true);
		leftRear->EnableVoltageCompensation(true);
		rightRear->EnableVoltageCompensation(true);
		arm->EnableVoltageCompensation(true);

		rightRear->Set(ControlMode::Follower, 3);
		leftRear->Set(ControlMode::Follower, 1);

		// Setup variables for auton (follow ball) mode
		auton = false;
		table = nt::NetworkTableInstance::GetDefault().GetTable("Vision");
		imgWidth = 640;
		counter = 0;

		//Initialization of the rest of the variables
		killed = false;
		PIDControl = false;
		editTarget = false;
		armTarget = 0;
		leftTarget = 0;
		rightTarget = 0;
	}

	void AutonomousInit() override {
	}

	void AutonomousPeriodic() override {
	}

	//This function is run every time the TeleOperator mode is enabled from the driver station
	//It will not run again until the TeleOperator mode is re-enabled
	void TeleopInit() override {
		//Initialization of variables to make sure everything is setup properly whenever TeleOp is re-enabled
		killed = false;

		int PIDLoopIdx = 0;
		int timeoutMs = 0;
		double rampTime = 0.5;
		armTarget = -5000;
		leftTarget = 0;
		rightTarget = 0;

		// Open/Closed Loop Ramp forces the robot to take a certain amount of time to reach the speed
		// it is told. It takes (rampTime) # of seconds to reach the full speed it's told. This avoids
		// having the robot have enormous changes in speed so that parts don't jerk around on the bot.

		// OpenloopRamp is used when the robot is being controlled directly, whereas ClosedloopRamp is
		// used when the motors are in a control loop, such as PID control. For now, they both use the
		// same ramp time.
		leftFront->ConfigOpenloopRamp(rampTime, timeoutMs);
		rightFront->ConfigOpenloopRamp(rampTime, timeoutMs);
		leftRear->ConfigOpenloopRamp(rampTime, timeoutMs);
		rightRear->ConfigOpenloopRamp(rampTime, timeoutMs);
		arm->ConfigOpenloopRamp(rampTime, timeoutMs);

		leftFront->ConfigClosedloopRamp(rampTime, timeoutMs);
		rightFront->ConfigClosedloopRamp(rampTime, timeoutMs);
		leftRear->ConfigClosedloopRamp(rampTime, timeoutMs);
		rightRear->ConfigClosedloopRamp(rampTime, timeoutMs);
		arm->ConfigClosedloopRamp(rampTime, timeoutMs);

		// Configuration setup for the sensor on the arm, PIDLoopIdx is the default PID control loop.
		// SensorPhase dictates which the direction of positive or negative movement for the encoder
		// The Forward/Reverse soft limits try to force the motor to keep the encoder within the range
		// of values specified
		arm->ConfigSelectedFeedbackSensor(QuadEncoder, PIDLoopIdx, timeoutMs);
		arm->SetSensorPhase(true);
		arm->ConfigForwardSoftLimitThreshold(75000, timeoutMs);
		arm->ConfigReverseSoftLimitThreshold(-160000, timeoutMs);
		arm->ConfigForwardSoftLimitEnable(true, timeoutMs);
		arm->ConfigReverseSoftLimitEnable(true, timeoutMs);

		// NominalOutput is the default output when the PID loop isn't influencing the motor output, we
		// want the motor to remain still if it has reached its target, so the nominal output is 0.
		// Peak Output is the maximum output that the motor can have during PID Control (in percent)
		arm->ConfigNominalOutputForward(0, timeoutMs);
		arm->ConfigNominalOutputReverse(0, timeoutMs);
		arm->ConfigPeakOutputForward(1, timeoutMs);
		arm->ConfigPeakOutputReverse(-1, timeoutMs);

		//Setup for the PID constants for the arm, these are found through trial and error
		arm->Config_kF(PIDLoopIdx, 0.0, timeoutMs);
		arm->Config_kP(PIDLoopIdx, 0.05, timeoutMs);
		arm->Config_kI(PIDLoopIdx, 0.000001, timeoutMs);
		arm->Config_kD(PIDLoopIdx, 11.0, timeoutMs);

		// These commands setup the encoders for the left and right motors, functions very similarly to
		// the arm's encoder setup
		leftFront->ConfigSelectedFeedbackSensor(QuadEncoder, PIDLoopIdx, timeoutMs);
		rightFront->ConfigSelectedFeedbackSensor(QuadEncoder, PIDLoopIdx, timeoutMs);
		leftFront->SetSensorPhase(false);
		rightFront->SetSensorPhase(false);
		leftFront->SetSelectedSensorPosition(0, PIDLoopIdx, timeoutMs);
		rightFront->SetSelectedSensorPosition(0, PIDLoopIdx, timeoutMs);

		leftFront->ConfigNominalOutputForward(0, timeoutMs);
		leftFront->ConfigNominalOutputReverse(0, timeoutMs);
		leftFront->ConfigPeakOutputForward(1, timeoutMs);
		leftFront->ConfigPeakOutputReverse(-1, timeoutMs);
		rightFront->ConfigNominalOutputForward(0, timeoutMs);
		rightFront->ConfigNominalOutputReverse(0, timeoutMs);
		rightFront->ConfigPeakOutputForward(1, timeoutMs);
		rightFront->ConfigPeakOutputReverse(-1, timeoutMs);

		leftFront->Config_kF(PIDLoopIdx, 0.0, timeoutMs);
		leftFront->Config_kP(PIDLoopIdx, 1.0, timeoutMs);
		leftFront->Config_kI(PIDLoopIdx, 0.0, timeoutMs);
		leftFront->Config_kD(PIDLoopIdx, 0.0, timeoutMs);

		rightFront->Config_kF(PIDLoopIdx, 0.0, timeoutMs);
		rightFront->Config_kP(PIDLoopIdx, 1.0, timeoutMs);
		rightFront->Config_kI(PIDLoopIdx, 0.0, timeoutMs);
		rightFront->Config_kD(PIDLoopIdx, 0.0, timeoutMs);

		//Initialization of variables, their use is explained later on
		auton = false;
		PIDControl = false;
		counter = 0;
		editTarget = false;
	}


	void TeleopPeriodic() override {
		/* This function is the main loop when the TeleOperator mode is enabled.
		 *
		 * Encapsulating the entire function is a functionality based on the variable "killed"
		 * If "killed" is set to true, the robot will not turn on any motors until TeleOp is
		 * re-enabled through the driver station.
		 *
		 * There are three main modes throughout this loop: default, auton, and PIDControl
		 *
		 * default:
		 *
		 * When TeleOp is enabled, this mode is first engaged. This gives direct control of the bot
		 * to the driver. Left/Right on the joystick rotates the robot, while Forward/Reverse on the bot
		 * controls the throttle. The twisting of the joystick (Yaw) controls the position of the arm.
		 * Whenever another mode is disabled, the robot reverts to giving the driver control in this mode.
		 *
		 * Auton:
		 *
		 * This mode is toggled by pressing the trigger on the Saitek-X55 Joystick. In this mode,
		 * the robot will use the inputs that it receives from the NetworkTable(an interface used
		 * to communicate with an external processor) to try to track and follow an object within
		 * view of the camera.
		 *
		 * PIDControl:
		 *
		 * This mode is toggled by pressing the thumb button on the joystick.
		 * This mode enables PIDControl for all the motors on the bot. The driver has the same control
		 * scheme as in the default control mode, but this time the driver is actually changing the targets
		 * for the robot's position, rather than the PercentOutput of the motors. This means that the robot
		 * is trying to use PID control to drive to a specified location from the driver. This mode of PID
		 * control is only used on the drive motors, not for the arm. The arm uses PID control to move
		 * to a pre-defined location in the code. This target location can be edited through another toggle,
		 * the large red-button on the top of the joystick. Toggling this button allows the driver to use
		 * the yaw of the joystick to move the target of the arm's position. Then, the bot will use PID
		 * control to reach that specified target.
		 *
		 * Throughout any of these modes, using your pinkie, the driver can pull the large gray switch
		 * on the back of the joystick to kill all motors should the robot go out of control. The only
		 * way to get out of this killed mode is to disable and then re-enable TeleOp mode through the
		 * driver station.
		 *
		 * One feature to keep in mind is that the toggles prioritize auton mode. That is, if auton mode
		 * is enabled, and then PIDControl mode is enabled, the robot will remain in auton mode. In addition,
		 * if both auton and PIDControl mode are enabled, and then the driver disables auton mode, the bot
		 * will enter PIDControl mode, rather than revert to default. In summary, the modes have a priority
		 * (from high to low) of [Auton, PIDControl, default]. This program DOES NOT reset any of the modes
		 * until it is disabled, so keep this in mind while driving.
		 *
		 * P.S.
		 * Occasionally, the encoder on the arm becomes out of sync with the programming. This means that
		 * it's position needs to be reset using a known location for the sensor value. To perform this
		 * calibration, move the arm so that it is touching the protective covering next to the speed
		 * controllers. Then, press the red button on the shaft of the joystick (behind the large gray switch,
		 * accessible with the pinkie) to set the encoder position. After doing this, the arm PID control
		 * will function correctly.
		 *
		 */

		// These following commands output the position, velocity, and target values for all the motors
		// connected to the robot.
		SmartDashboard::PutNumber("Arm Position", arm->GetSelectedSensorPosition(0));
		SmartDashboard::PutNumber("Arm Velocity", arm->GetSelectedSensorVelocity(0));
		SmartDashboard::PutNumber("Arm Target", armTarget);

		SmartDashboard::PutNumber("Left Position", leftFront->GetSelectedSensorPosition(0));
		SmartDashboard::PutNumber("Left Velocity", leftFront->GetSelectedSensorVelocity(0));
		SmartDashboard::PutNumber("Right Position", rightFront->GetSelectedSensorPosition(0));
		SmartDashboard::PutNumber("Right Velocity", rightFront->GetSelectedSensorVelocity(0));
		SmartDashboard::PutNumber("Left Target", leftTarget);
		SmartDashboard::PutNumber("Right Target", rightTarget);

		if(!killed){
			if (auton){
				// This command gets the information about the targeted object from the
				// NetworkTable, and stores it in the variable "contour"
				contour = table.get()->GetNumberArray("Contour", llvm::ArrayRef<double>());
				if(contour.size() > 0){
					double x = contour[0];
					double y = contour[1];
					double w = contour[2];
					double h = contour[3];
					double area = contour[4];
					std::cout << x;

					/* These lines of code process two control variables: "rectCenterX" and "error"
					 *
					 * "rectCenterX" is a variable between -1.0 and 1.0, representing the object's
					 * positioning within the screen. -1.0 is on the very left of the screen, 1.0
					 * on the very right, with 0.0 being exactly in the middle. This variable is
					 * used to dictate the turning of the robot to follow the target. It is scaled
					 * by a factor of 2.0 (in the line "rectCenterX /= 2.0") to be in the appropriate
					 * range to control the motors.
					 *
					 * "error" is an approximate of the target's distance from the robot. It's scale
					 * is not exactly precise, and is scaled quite a bit to be in the appropriate
					 * range for the robot. In the end, the robot targets 0.5 meters (approximate) when
					 * trying to follow a green/yellow vex ball.
					 *
					 */
					double rectCenterX = (x + w/2.0 - imgWidth/2.0)/(imgWidth/2.0);
					rectCenterX /= 2.0;
					double a = 16821.5;
					double dist = a/sqrt(w*h);
					double error = (dist -150)/50;
					error /= -25.0;
					// The motors outputs are dictated by "error" and "rectCenterX"
					leftFront->Set(ControlMode::PercentOutput, error - rectCenterX);
					rightFront->Set(ControlMode::PercentOutput, error + rectCenterX);

					/* The following if/else statement tries to detect when the robot has been sitting still
					 * for a short amount of time. It decides when this condition is true by
					 *
					 */

					if( pow(x - previous[0], 2) + pow((y - previous[1]), 2) < pow(5, 2) && pow(w*h - previous[2] * previous[3], 2) < pow(10, 2)){
						if (counter >= 3){
							leftFront->Set(ControlMode::PercentOutput, 0.3);
							rightFront->Set(ControlMode::PercentOutput, 0.3);
						}
						counter++;
					} else {
						counter = 0;
					}
					previous[0] = x;
					previous[1] = y;
					previous[2] = w;
					previous[3] = h;
				} else {
					leftFront->Set(ControlMode::PercentOutput, 0);
					rightFront->Set(ControlMode::PercentOutput, 0);
				}
			} else if(PIDControl){
				double joyX = joy->GetX();
				double joyY = joy->GetY();
				double joyZ = joy->GetZ();
				double throttleScale = 50.0;
				double turnScale = 20.0;

				leftFront->Set(ControlMode::Position, leftTarget);
				rightFront->Set(ControlMode::Position, rightTarget);
				arm->Set(ControlMode::Position, armTarget);

				leftTarget += joyY*throttleScale - joyX*turnScale;
				rightTarget += joyY*throttleScale + joyX*turnScale;

				if(joy->GetRawButtonPressed(2)){
					editTarget = !editTarget;
				}
				if(editTarget){
					armTarget += joyZ*2500;
				}
			} else {
				double joyX = joy->GetX();
				double joyY = joy->GetY();
				double joyZ = joy->GetZ();
				leftFront->Set(ControlMode::PercentOutput, joyY-joyX*0.5);
				rightFront->Set(ControlMode::PercentOutput, joyY+joyX*0.5);
				arm->Set(ControlMode::PercentOutput, joyZ);

				if(!limitSwitch->Get()){
					leftFront->Set(ControlMode::PercentOutput, 0.1);
					rightFront->Set(ControlMode::PercentOutput, 0.1);
				}
			}

			if(joy->GetRawButtonPressed(1)){
				auton = !auton;
				counter = 0;
			}
			if(joy->GetRawButtonPressed(4)){
				PIDControl = !PIDControl;
				leftFront->SetSelectedSensorPosition(0, 0, 10);
				rightFront->SetSelectedSensorPosition(0, 0, 10);
				leftTarget = 0;
				rightTarget = 0;
			}
			if(joy->GetRawButtonPressed(5)){
				arm->SetSelectedSensorPosition(100000, 0, 10);
			}

			//KILL SWITCH
			if(joy->GetRawButtonPressed(6)){
				killed = true;
				leftFront->Set(ControlMode::PercentOutput, 0.0);
				rightFront->Set(ControlMode::PercentOutput, 0.0);
				arm->Set(ControlMode::PercentOutput, 0.0);
			}
		}

	}

	void TestPeriodic() override {}
};

START_ROBOT_CLASS(Robot)
