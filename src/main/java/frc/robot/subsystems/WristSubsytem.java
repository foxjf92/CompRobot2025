// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.MoveWristCommand;

public class WristSubsytem extends SubsystemBase {

  private SparkMax wristMotor;
  private SparkMaxConfig wristMotorConfig;

  public RelativeEncoder wristEncoder;
  public double currentPosition;
  
  public WristSubsytem() {
    wristMotorConfig = new SparkMaxConfig();

    wristMotorConfig
      .smartCurrentLimit(20)
      .idleMode(IdleMode.kBrake);

    wristMotor = new SparkMax(11, MotorType.kBrushless);
    wristMotor.configure(wristMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    wristEncoder = wristMotor.getEncoder();
  }

  public void moveWrist(double wristSpeed) {
    wristMotor.set(wristSpeed);
  }

  @Override
  public void periodic() {
    currentPosition = wristEncoder.getPosition();

    SmartDashboard.putNumber("Wrist Position", currentPosition);
    SmartDashboard.putNumber("Wrist Setpoint", MoveWristCommand.wristSetpoint);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
