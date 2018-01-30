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

#include "ctre/Phoenix.h"

class Robot : public frc::IterativeRobot {
private:

	frc::Joystick m_stick{0};
	frc::LiveWindow& m_lw = *frc::LiveWindow::GetInstance();
	frc::Timer m_timer;
	cs::UsbCamera cam;
	std::shared_ptr<nt::NetworkTable> table;

	TalonSRX * leftFront;
	TalonSRX * rightFront;
	VictorSPX * leftRear;
	VictorSPX * rightRear;

	Joystick * joy;

	frc::DigitalInput * limitSwitch;

	bool auton;
	std::vector<double> contour;
	double imgWidth;

	double previous[4];
	int counter;
public:
	Robot() {
		m_timer.Start();

		leftFront = new TalonSRX(1);
		rightFront = new TalonSRX(3);
		leftRear = new VictorSPX(4);
		rightRear = new VictorSPX(2);

		joy = new Joystick(0);

		cam = CameraServer::GetInstance()->StartAutomaticCapture();
		cam.SetResolution(640, 480);
		//CameraServer::GetInstance()->PutVideo("Rectangle", 640, 480);
		limitSwitch = new frc::DigitalInput(0);

		leftFront->SetInverted(true);
		leftRear->SetInverted(true);

		leftFront->EnableVoltageCompensation(true);
		rightFront->EnableVoltageCompensation(true);
		leftRear->EnableVoltageCompensation(true);
		rightRear->EnableVoltageCompensation(true);

		rightRear->Set(ControlMode::Follower, 3);
		leftRear->Set(ControlMode::Follower, 1);

		auton = false;
		table = nt::NetworkTableInstance::GetDefault().GetTable("Vision");
		imgWidth = 640;
		counter = 0;
	}

	void AutonomousInit() override {
	}

	void AutonomousPeriodic() override {
	}

	void TeleopInit() override {
		leftFront->ConfigOpenloopRamp(0.5, 0);
		rightFront->ConfigOpenloopRamp(0.5, 0);
		leftRear->ConfigOpenloopRamp(0.5, 0);
		rightRear->ConfigOpenloopRamp(0.5, 0);

		rightFront->ConfigSelectedFeedbackSensor(QuadEncoder, 0, 0);
		rightFront->SetSensorPhase(false);
		auton = false;
		counter = 0;
	}


	void TeleopPeriodic() override {
/*		This mode of operation checks if "auton" is enabled, which can be toggled by pressing
		the trigger button on a joystick. This mode tracks a yellow vex ball, and will have the
		robot try its best to move towards it. The robot receives info about the position of the
		ball from the network tables, then calculates how much power it should give to each motor.
		It has also been programmed to back away from the ball if it does not move for a short period of time,
		this has been done in trying to detect when the bot should try and slam into a wall after
		in order to align itself with the portal.

		When not in auton mode, a joystick acts as the main control input, with left/right turning
		the robot and forward/back moving it either forwards or backwards.
 	  */
		if (auton){
			contour = table.get()->GetNumberArray("Contour", llvm::ArrayRef<double>());
			if(contour.size() > 0){
				double x = contour[0];
				double y = contour[1];
				double w = contour[2];
				double h = contour[3];
				double area = contour[4];
				std::cout << x;

				double rectCenterX = (x + w/2.0 - imgWidth/2.0)/(imgWidth/2.0);
				rectCenterX /= 2.0;
				double a = 16821.5;
				double dist = a/sqrt(w*h);
				double error = (dist -150)/50;
				error /= -25.0;
				leftFront->Set(ControlMode::PercentOutput, error - rectCenterX);
				rightFront->Set(ControlMode::PercentOutput, error + rectCenterX);
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
		} else {
			double joyX = joy->GetX();
			double joyY = joy->GetY();
			leftFront->Set(ControlMode::PercentOutput, joyY-joyX*0.5);
			rightFront->Set(ControlMode::PercentOutput, joyY+joyX*0.5);
			if(!limitSwitch->Get()){
				leftFront->Set(ControlMode::PercentOutput, 0.1);
				rightFront->Set(ControlMode::PercentOutput, 0.1);
			}
		}

		if (joy->GetRawButtonPressed(1)){
			auton = !auton;
			counter = 0;
		}
	}

	void TestPeriodic() override {}
};

START_ROBOT_CLASS(Robot)