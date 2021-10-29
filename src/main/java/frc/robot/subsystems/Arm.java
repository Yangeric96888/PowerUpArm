// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;


public class Arm extends SubsystemBase {
  private WPI_TalonSRX leftTalon = new WPI_TalonSRX(Constants.leftWheelPort);
  private WPI_TalonSRX rightTalon = new WPI_TalonSRX(Constants.rightWheelPort);
  private WPI_TalonSRX armTalon = new WPI_TalonSRX(Constants.arm);

  public Arm() {
    leftTalon.configFactoryDefault();
    rightTalon.configFactoryDefault();
    armTalon.configFactoryDefault();

    armTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 10);
  }

  @Override
  public void periodic() {
    if (RobotContainer.getJoystick().getRawButton(1)) {
      leftTalon.set(ControlMode.PercentOutput, 0.4);
      rightTalon.set(ControlMode.PercentOutput, 0.4);
    } else {
      leftTalon.set(ControlMode.PercentOutput, 0.0);
      rightTalon.set(ControlMode.PercentOutput, 0.0);
    }
  }

  public double getArmAngle() {
    return (armTalon.getSelectedSensorPosition() / 4096.0) * 360;
  }

  public void setArmPower(double power) {
    armTalon.set(ControlMode.PercentOutput,power);
  }
  
}
