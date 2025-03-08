// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkAnalogSensor;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

  private SparkMax frontIntakeRoller;
  private SparkMax backIntakeRoller;
  private SparkMaxConfig intakeMotorConfig;

  private double backRollerMultiplier = 3.0;
  private SparkAnalogSensor algaeSensor;
  public static boolean algaeCollectedStatus;
  
  
    public IntakeSubsystem() {
      intakeMotorConfig = new SparkMaxConfig();
      intakeMotorConfig
        .smartCurrentLimit(40)
        .idleMode(IdleMode.kBrake);
  
        
      frontIntakeRoller = new SparkMax(13, MotorType.kBrushless);
      frontIntakeRoller.configure(intakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  
      backIntakeRoller = new SparkMax(12, MotorType.kBrushless);
      backIntakeRoller.configure(intakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      
      algaeSensor = frontIntakeRoller.getAnalog();
    }
  
    public static boolean algaeCollected() {
      return algaeCollectedStatus;
  }

  public void spinIntake(double speed){
    frontIntakeRoller.set(speed);
    backIntakeRoller.set(-speed);
  }

  @Override
  public void periodic() {
    if(algaeSensor.getVoltage() > 0.1) {
      algaeCollectedStatus = false;
    }
    else {
      algaeCollectedStatus = true;    
    }   

    SmartDashboard.putBoolean("Algae Present?", algaeCollected());
  }

}
