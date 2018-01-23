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
	// Robot drive system
	//frc::Spark m_left{0};
	//frc::Spark m_right{1};
	//frc::DifferentialDrive m_robotDrive{m_left, m_right};

	frc::Joystick m_stick{0};
	frc::LiveWindow& m_lw = *frc::LiveWindow::GetInstance();
	frc::Timer m_timer;
	cs::UsbCamera cam;
	//frc::SmartDashboard dash;
	std::shared_ptr<nt::NetworkTable> table;

	TalonSRX * left;
	TalonSRX * right;
	Joystick * joy;

	frc::DigitalInput * limitSwitch;

	bool auton;
	std::vector<double> contour;
	double imgWidth;

	double previous[4];
	int counter;
public:
	Robot() {
		//m_robotDrive.SetExpiration(0.1);
		m_timer.Start();

		left = new TalonSRX(0);
		right = new TalonSRX(1);

		joy = new Joystick(0);

		cam = CameraServer::GetInstance()->StartAutomaticCapture();
		cam.SetResolution(640, 480);
		//CameraServer::GetInstance()->PutVideo("Rectangle", 640, 480);
		//dash = new SmartDashboard::init();
		//netTableInstance = nt::NetworkTableInstance::GetDefault();
		limitSwitch = new frc::DigitalInput(0);

		right->SetInverted(true);

		left->EnableVoltageCompensation(true);
		right->EnableVoltageCompensation(true);

		auton = false;
		table = nt::NetworkTableInstance::GetDefault().GetTable("Vision");
		imgWidth = 640;
		counter = 0;

		//	new std::vector<double>()
		//previous = new std::vector<double>(4);

	}

	void AutonomousInit() override {
		//m_timer.Reset();
		//m_timer.Start();
	}

	void AutonomousPeriodic() override {
	}

	void TeleopInit() override {
		left->ConfigOpenloopRamp(0.5, 0);
		right->ConfigOpenloopRamp(0.5, 0);

		right->ConfigSelectedFeedbackSensor(QuadEncoder, 0, 0);
		right->SetSensorPhase(false);
		auton = false;
		counter = 0;
	}

	void TeleopPeriodic() override {
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
				left->Set(ControlMode::PercentOutput, error + rectCenterX);
				right->Set(ControlMode::PercentOutput, error - rectCenterX);
				if( pow(x - previous[0], 2) + pow((y - previous[1]), 2) < pow(5, 2) && pow(w*h - previous[2] * previous[3], 2) < pow(10, 2)){
					if (counter >= 3){
						left->Set(ControlMode::PercentOutput, 0.3);
						right->Set(ControlMode::PercentOutput, 0.3);
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
				left->Set(ControlMode::PercentOutput, 0);
				right->Set(ControlMode::PercentOutput, 0);
			}
		} else {
			double joyX = joy->GetX();
			double joyY = joy->GetY();
			left->Set(ControlMode::PercentOutput, joyY+joyX*0.5);
			right->Set(ControlMode::PercentOutput, joyY-joyX*0.5);
			if(!limitSwitch->Get()){
				left->Set(ControlMode::PercentOutput, 0.1);
				right->Set(ControlMode::PercentOutput, 0.1);
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
