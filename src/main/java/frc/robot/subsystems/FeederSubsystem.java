// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FeederSubsystem extends SubsystemBase {

  private SparkMax feedMotor;
  private SparkMaxConfig feedMotorConfig;

  public FeederSubsystem() {

    feedMotorConfig = new SparkMaxConfig();
    feedMotorConfig
      .smartCurrentLimit(20)
      .idleMode(IdleMode.kBrake);

    feedMotor = new SparkMax(14, MotorType.kBrushless);
    feedMotor.configure(feedMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  
  }

  public void spinFeeder(double speed){
    feedMotor.set(speed);
  }

}
