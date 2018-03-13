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
#include <DriverStation.h>

#include <PowerDistributionPanel.h>

#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>

#include "ctre/Phoenix.h"

#include <pathfinder.h>

#include <InputControl.h>
#include <RobotLogic.h>
#include <AutonScripts/AutonScripts.h>

#define LEFT 0
#define CENTER 1
#define RIGHT 2
#define FORWARD 3

class Robot : public frc::IterativeRobot {
private:
	//Unused initial variables
	//frc::Joystick m_stick{0};
	frc::LiveWindow& m_lw = *frc::LiveWindow::GetInstance();
	frc::Timer timer;

	std::string gameData;

	frc::PowerDistributionPanel * power = new PowerDistributionPanel(0);

	//Variable declaration for camera and NetworkTable
	cs::UsbCamera cam;
	std::shared_ptr<nt::NetworkTable> table;

	//Variable declaration for all the motors
	TalonSRX * leftMaster;
	TalonSRX * rightMaster;
	VictorSPX * leftFollower;
	VictorSPX * rightFollower;
	TalonSRX * arm;
	TalonSRX * leftClaw;
	TalonSRX * rightClaw;

	PigeonIMU * pigeon;

	Compressor * compressor;

	frc::DoubleSolenoid claw {CLAW_PCM_ID_1, CLAW_PCM_ID_2};
	frc::DoubleSolenoid clawWrist {CLAW_WRIST_PCM_ID_1, CLAW_WRIST_PCM_ID_2};

	//Declaration for the joystick and a button that is connected to a DigitalInput port on the RoboRIO
	Joystick * joy;
	frc::DigitalInput * pinRed;
	frc::DigitalInput * pinNothing;
	int position;

	//Variable declaration for variables that are used while the robot is running
	bool auton;
	std::vector<double> contour;
	double imgWidth;

	double previous[4];
	int counter;

	bool editTarget;
	bool PIDControl;

	bool killed;

	InputControl * control;
	RobotLogic * bot;

	AutonScript * script;

	//int trajectories;
	//int numPoints_1;
	//Waypoint * points_1;
	//int numPoints_2;
	//Waypoint * points_2;

	//bool stage0;
	//bool stage1;
	//bool stage2;
public:
	//This function runs every time the robot first turns on, will not run again until it is turned off and back on again
	Robot() {

		//Initialize the motors with their respective ID's on the CAN bus
		leftFollower = new VictorSPX(LEFT_FOLLOW_ID);
		rightFollower = new VictorSPX(RIGHT_FOLLOW_ID);
		leftMaster = new TalonSRX(LEFT_MASTER_ID);
		rightMaster = new TalonSRX(RIGHT_MASTER_ID);
		arm = new TalonSRX(ARM_ID);
		leftClaw = new TalonSRX(LEFT_CLAW_ID);
		rightClaw = new TalonSRX(RIGHT_CLAW_ID);

		pigeon = new PigeonIMU(0);
		pigeon->SetFusedHeading(0, 10);

		compressor = new Compressor(0);
		compressor->SetClosedLoopControl(true);

		control = new InputControl(FINAL_CONTROL_SINGLE_ARCADE);
		control->SetDriveMultipliers(1.0, 1.0, 0.5, 0.5);
		control->SetArmMultiplier(PID_ARM_MULTIPLIER);

		bot = new RobotLogic(control);
		bot->SetLeftMasterMotor(leftMaster);
		bot->SetLeftFollowerMotor(leftFollower);
		bot->SetRightMasterMotor(rightMaster);
		bot->SetRightFollowerMotor(rightFollower);
		bot->SetArmMotor(arm);
		bot->SetLeftClawMotor(leftClaw);
		bot->SetRightClawMotor(rightClaw);
		bot->SetClaw(&claw);
		bot->SetClawWrist(&clawWrist);
		bot->SetPigeon(pigeon);
		position = FORWARD;

		bot->InitializeMotors();

		bot->SetBotParameters(0.020, 6.0, 5.0, 10.0, 0.65405, 0.478778720);

		script = NULL;

		joy = control->joy1;

		//Start the camera stream with a resolution of 640x480
		////cam = CameraServer::GetInstance()->StartAutomaticCapture();
		cam.SetResolution(640, 480);
		//CameraServer::GetInstance()->PutVideo("Rectangle", 640, 480);
		pinRed = new frc::DigitalInput(9);
		pinNothing = new frc::DigitalInput(8);

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

		double rampTime = 0.01;//0.5 to 0.1 2/15/18 CS
		bot->ConfigureOpenRampTime(rampTime);
		bot->ConfigureClosedRampTime(rampTime);

		bot->ConfigureLeftPID(0.0, 0.0, 0.0, 0.0);
		bot->ConfigureRightPID(0.0, 0.0, 0.0, 0.0);
		bot->ConfigureArmPID(0.0, ARM_P, ARM_I, ARM_D);



	}

	void RobotInit() override {
	}

	void AutonomousInit() override {
		gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();
		bot->pigeonReference = 0;
		pigeon->SetFusedHeading(0, 10);

		if(!pinRed->Get()){
			if(!pinNothing->Get()){
				position = FORWARD;
			} else {
				position = CENTER;
			}
		} else {
			if(!pinNothing->Get()){
				position = RIGHT;
			} else {
				position = LEFT;
			}
		}
		SmartDashboard::PutNumber("POSITION", position);
		SmartDashboard::PutString("Game Data", gameData);

		if(position == LEFT){
			if(gameData.length() > 0){
				if(gameData[0] == 'L'){
					script = new LeftSwitch(bot);
				} else if(gameData[1] == 'L'){
					script = new LeftScale(bot);
				} else {
					script = new LeftHookSwitch(bot);
				}
			}
			//script = new LeftScale(bot);
		} else if (position == CENTER){
			if(gameData[0] == 'L'){
				script = new CenterSwitchLeft(bot);
			} else {
				script = new CenterSwitchRight(bot);
			}
		} else if (position == RIGHT){
			if(gameData.length() > 0){
				if(gameData[0] == 'R'){
					script = new RightSwitch(bot);
				} else if(gameData[1] == 'R'){
					script = new RightScale(bot);
				} else {
					script = new RightHookSwitch(bot);
				}
			}
		} else {

		}
		//script = new LeftSwitch(bot);


		bot->AutonInitEncoders();
		bot->ClawClose();
		bot->ClawWristRetract();
		bot->ClawNeutralSuck();
	}

	void AutonomousPeriodic() override {
		SmartDashboard::PutNumber("Arm Position", arm->GetSelectedSensorPosition(0));
		SmartDashboard::PutNumber("Arm Velocity", arm->GetSelectedSensorVelocity(0));
		SmartDashboard::PutNumber("Arm Target", bot->GetArmTarget() + control->GetAxisArmChange());

		SmartDashboard::PutNumber("Left Position", leftMaster->GetSelectedSensorPosition(0));
		SmartDashboard::PutNumber("Left Velocity", leftMaster->GetSelectedSensorVelocity(0));
		SmartDashboard::PutNumber("Right Position", rightMaster->GetSelectedSensorPosition(0));
		SmartDashboard::PutNumber("Right Velocity", rightMaster->GetSelectedSensorVelocity(0));

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

		SmartDashboard::PutNumber("Heading", pigeon->GetFusedHeading());
		SmartDashboard::PutNumber("Target Heading", bot->targetBearing);
		SmartDashboard::PutNumber("Angle Difference", bot->angleDifference);
		SmartDashboard::PutNumber("Pigeon Reference", bot->pigeonReference);

		SmartDashboard::PutNumber("LEFT OUTPUT", bot->leftOutput);
		SmartDashboard::PutNumber("RIGHT OUTPUT", bot->rightOutput);

		SmartDashboard::PutNumber("Left Encoder Error", bot->leftEncoder->last_error);
		SmartDashboard::PutNumber("Left Encoder Segment", bot->rightEncoder->last_error);
		SmartDashboard::PutNumber("Left Encoder Finished", bot->leftEncoder->finished);
		SmartDashboard::PutNumber("Timer", timer.Get());

		//SmartDashboard::PutNumber("Trajectory Length", bot->trajectoryLength[0]);
		//SmartDashboard::PutNumber("Trajectory Length 2", bot->trajectoryLength[1]);

		script->RunScript();
		script->CheckFlags();

	}

	void TeleopInit() override {
		killed = false;

		//control->armTarget = 0;
		//bot->ClawClose();
		//bot->ClawWristRetract();
	}


	void TeleopPeriodic() override {

		SmartDashboard::PutNumber("Arm Position", arm->GetSelectedSensorPosition(0));
		SmartDashboard::PutNumber("Arm Velocity", arm->GetSelectedSensorVelocity(0));
		SmartDashboard::PutNumber("Arm Target", arm->GetClosedLoopTarget(0));

		SmartDashboard::PutNumber("Left Position", leftMaster->GetSelectedSensorPosition(0));
		SmartDashboard::PutNumber("Left Velocity", leftMaster->GetSelectedSensorVelocity(0));
		SmartDashboard::PutNumber("Right Position", rightMaster->GetSelectedSensorPosition(0));
		SmartDashboard::PutNumber("Right Velocity", rightMaster->GetSelectedSensorVelocity(0));

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
		SmartDashboard::PutNumber("ARM OUTPUT", arm->GetMotorOutputPercent());

		SmartDashboard::PutNumber("Heading", pigeon->GetFusedHeading());




		//SmartDashboard::PutNumber("LEFT OUTPUT", 0);
		//SmartDashboard::PutNumber("RIGHT OUTPUT", 0);
		if(!killed){
			SmartDashboard::PutNumber("STAGE", 2);
			if (auton){
				// This command gets the information about the targeted object from the
				// NetworkTable, and stores it in the variable "contour"
				contour = table.get()->GetNumberArray("Contour", llvm::ArrayRef<double>());
			} else {
				bot->DirectDrive();
				bot->PIDMoveArm();
				bot->PIDSetPositions();
				bot->DirectControlClaw();

				//control->ToggleMode();
				//bot->SetMode();

			}
		}

	}

	void TestPeriodic() override {}
};

START_ROBOT_CLASS(Robot)
