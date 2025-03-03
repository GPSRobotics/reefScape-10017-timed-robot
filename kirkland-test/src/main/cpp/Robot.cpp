// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/Joystick.h>
#include <frc/TimedRobot.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/motorcontrol/PWMSparkMax.h>
#include <frc/XboxController.h>
#include <frc/Encoder.h>
#include <rev/SparkMax.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix/motorcontrol/can/WPI_VictorSPX.h>
#include <studica/AHRS.h>

#define BRUSHED rev::spark::SparkLowLevel::MotorType::kBrushed
#define BRUSHLESS rev::spark::SparkLowLevel::MotorType::kBrushless

/**
 * This is a demo program showing the use of the DifferentialDrive class.
 * Runs the motors with arcade steering.
 */
class Robot : public frc::TimedRobot {
  // rev::spark::SparkMax leftMotor{0, BRUSHED};
  // rev::spark::SparkMax rightMotor{1, BRUSHED};
  ctre::phoenix::motorcontrol::can::WPI_VictorSPX rearLeftDrive{0};
  ctre::phoenix::motorcontrol::can::WPI_VictorSPX frontLeftDrive{1};
  ctre::phoenix::motorcontrol::can::WPI_VictorSPX rearRightDrive{2};
  ctre::phoenix::motorcontrol::can::WPI_VictorSPX frontRightDrive{3};
  rev::spark::SparkMax dummy{2, BRUSHLESS};
  ctre::phoenix6::hardware::TalonFX lift{0};
  studica::AHRS navx{studica::AHRS::NavXComType::kMXP_UART};
  frc::Encoder leftDrive{0, 1};
  frc::Encoder rightDrive{2, 3};
  frc::XboxController controller{0};

  ctre::phoenix6::controls::PositionVoltage targetPos{0_tr};
  ctre::phoenix6::controls::VelocityVoltage targetVel{0_tps};

  rev::spark::SparkClosedLoopController revController{dummy.GetClosedLoopController()};


 public:
  Robot() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    rearLeftDrive.SetInverted(true);
    frontLeftDrive.SetInverted(true);
  }

  void AutonomousInit() override {
    // Turn until drive train is over 90 degrees 
    double currentAngle = navx.GetAngle();
    while(currentAngle < 90) {
      currentAngle = navx.GetAngle();
      rearLeftDrive.Set(1);
      frontLeftDrive.Set(1);
      rearRightDrive.Set(-1);
      frontRightDrive.Set(-1);
    }
    rearLeftDrive.Set(0);
    frontLeftDrive.Set(0);
    rearRightDrive.Set(0);
    frontRightDrive.Set(0);
  }

  void AutonomousPeriodic() override {
  }

  void TeleopInit() override {
  
  }

  void TeleopPeriodic() override {
    // Drive code
    double y = controller.GetLeftY();
    double x = -controller.GetRightX();

    if(fabs(x) < 0.1) x = 0.0;
    if(fabs(y) < 0.1) y = 0.0;

    double leftSpeed = y + x;
    double rightSpeed = y - x;

    rearLeftDrive.Set(leftSpeed);
    rearLeftDrive.Set(leftSpeed);
    rearRightDrive.Set(rightSpeed);
    frontRightDrive.Set(rightSpeed);

    // Generic power-based control scheme
    // lift.Set(controller.GetRightTriggerAxis() - controller.GetLeftTriggerAxis());

    // CTRE Positional PID control scheme
    if(controller.GetAButtonPressed()) {
      units::angle::turn_t target{2.0};
      lift.SetControl(targetPos
      .WithPosition(target));
    }

    // REV Positional PID control scheme
    if(controller.GetBButtonPressed()) {
      
    }

  }
};

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
