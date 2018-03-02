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
#include <DoubleSolenoid.h>
#include <Compressor.h>
#include <iostream>
#include <unistd.h>
#include <stdio.h>

#include <PowerDistributionPanel.h>

#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>

#include "ctre/Phoenix.h"

#include <pathfinder.h>

#include <InputControl.h>
#include <RobotLogic.h>

#define ARCADE_JOYSTICK 0
#define ARCADE_PS4 1

class Robot : public frc::IterativeRobot {
private:
	//Unused initial variables
	//frc::Joystick m_stick{0};
	frc::LiveWindow& m_lw = *frc::LiveWindow::GetInstance();
	//frc::Timer m_timer;

	frc::PowerDistributionPanel * power = new PowerDistributionPanel(0);

	//Variable declaration for camera and NetworkTable
	cs::UsbCamera cam;
	std::shared_ptr<nt::NetworkTable> table;

	//Variable declaration for all the motors
	TalonSRX * leftRear;
	TalonSRX * rightRear;
	VictorSPX * leftFront;
	VictorSPX * rightFront;
	TalonSRX * arm;
	VictorSPX * leftClaw;
	TalonSRX * rightClaw;

	Compressor * compressor;

	frc::DoubleSolenoid claw {0, 1};
	frc::DoubleSolenoid clawWrist {2, 3};

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

	bool clawEnabled;
	bool clawWristEnabled;

	bool killed;

	InputControl * control;
	RobotLogic * bot;
public:
	//This function runs every time the robot first turns on, will not run again until it is turned off and back on again
	Robot() {

		//Initialize the motors with their respective ID's on the CAN bus
		leftFront = new VictorSPX(3);
		rightFront = new VictorSPX(7);
		leftRear = new TalonSRX(4);
		rightRear = new TalonSRX(8);
		arm = new TalonSRX(6);
		leftClaw = new VictorSPX(5);
		rightClaw = new TalonSRX(2);

		compressor = new Compressor(0);
		compressor->SetClosedLoopControl(true);

		control = new InputControl(ARCADE_LOGITECH);
		//control->SetLeftMotor(leftFront);
		//control->SetRightMotor(rightFront);
		//control->SetArmMotor(arm);
		//control->SetLeftClawMotor(leftClaw);
		//control->SetRightClawMotor(rightClaw);
		control->SetDriveMultipliers(1.0, 1.0, 0.5, 0.5);
		control->SetArmMultiplier(6.0);

		bot = new RobotLogic(control);
		bot->SetLeftMasterMotor(leftRear);
		bot->SetLeftFollowerMotor(leftFront);
		bot->SetRightMasterMotor(rightRear);
		bot->SetRightFollowerMotor(rightFront);
		bot->SetArmMotor(arm);
		bot->SetLeftClawMotor(leftClaw);
		bot->SetRightClawMotor(rightClaw);
		bot->SetClaw(&claw);
		bot->SetClawWrist(&clawWrist);

		bot->InitializeMotors();



		joy = control->joy1;

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
		//leftFront->SetInverted(true);
		//leftRear->SetInverted(true);
		//leftClaw->SetInverted(true);
		//arm->SetInverted(false);

		/*leftFront->EnableVoltageCompensation(true);
		rightFront->EnableVoltageCompensation(true);
		leftRear->EnableVoltageCompensation(true);
		rightRear->EnableVoltageCompensation(true);
		arm->EnableVoltageCompensation(true);*/
		//leftClaw->EnableVoltageCompensation(true);
		//rightClaw->EnableVoltageCompensation(true);

		//rightRear->Set(ControlMode::Follower, 3);
		//leftRear->Set(ControlMode::Follower, 1);

		//leftRear->SetInverted(true);
		//leftFront->SetInverted(true);

		//rightFront->Set(ControlMode::Follower, 8);
		//leftFront->Set(ControlMode::Follower, 3);

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

		clawEnabled = false;
		clawWristEnabled = false;



	}

	void RobotInit() override {
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
		double rampTime = 0.01;//0.5 to 0.1 2/15/18 CS
		armTarget = -5000;
		leftTarget = 0;
		rightTarget = 0;

		bot->ConfigureOpenRampTime(rampTime);
		bot->ConfigureClosedRampTime(rampTime);

		bot->ConfigureLeftPID(0.0, 0.0, 0.0, 0.0);
		bot->ConfigureRightPID(0.0, 0.0, 0.0, 0.0);
		bot->ConfigureArmPID(0.0, 8.0, 0.001, 1200.0);

		//Initialization of variables, their use is explained later on
		auton = false;
		PIDControl = false;
		counter = 0;
		editTarget = false;
		clawEnabled = false;
		clawWristEnabled = false;

		control->armTarget = 0;
	}


	void TeleopPeriodic() override {

		SmartDashboard::PutNumber("Arm Position", arm->GetSelectedSensorPosition(0));
		SmartDashboard::PutNumber("Arm Velocity", arm->GetSelectedSensorVelocity(0));
		SmartDashboard::PutNumber("Arm Target", bot->GetArmTarget() + control->GetAxisArmChange());

		SmartDashboard::PutNumber("Left Position", leftRear->GetSelectedSensorPosition(0));
		SmartDashboard::PutNumber("Left Velocity", leftRear->GetSelectedSensorVelocity(0));
		SmartDashboard::PutNumber("Right Position", rightRear->GetSelectedSensorPosition(0));
		SmartDashboard::PutNumber("Right Velocity", rightRear->GetSelectedSensorVelocity(0));

		SmartDashboard::PutNumber("Voltage", power->GetVoltage());
		SmartDashboard::PutNumber("Current", power->GetTotalCurrent());
		SmartDashboard::PutNumber("Temperature", power->GetTemperature());
		SmartDashboard::PutNumber("Power", power->GetTotalPower());
		SmartDashboard::PutNumber("Energy", power->GetTotalEnergy());

		SmartDashboard::PutNumber("STAGE", 1);

		SmartDashboard::PutBoolean("Compressor Enabled", compressor->Enabled());
		SmartDashboard::PutBoolean("Pressure Switch Enabled", compressor->GetPressureSwitchValue());
		SmartDashboard::PutBoolean("Compressor High Current Fault", compressor->GetCompressorCurrentTooHighFault());
		SmartDashboard::PutBoolean("Compressor Not Connected", compressor->GetCompressorNotConnectedFault());
		SmartDashboard::PutBoolean("Compressor Shorted Fault", compressor->GetCompressorShortedFault());
		SmartDashboard::PutBoolean("Closed Loop Control", compressor->GetClosedLoopControl());
		SmartDashboard::PutNumber("Compressor Current", compressor->GetCompressorCurrent());

		//SmartDashboard::PutNumber("Joystick 2", control->joy1->GetX());
		if(!killed){
			SmartDashboard::PutNumber("STAGE", 2);
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
				SmartDashboard::PutNumber("STAGE", 3);
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
				bot->DirectDrive();
				bot->PIDMoveArm();
				bot->PIDSetPositions();
				bot->DirectControlClaw();

				control->ToggleMode();
				bot->SetMode();


			}
		}

	}

	void TestPeriodic() override {}
};

START_ROBOT_CLASS(Robot)
