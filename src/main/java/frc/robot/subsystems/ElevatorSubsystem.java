// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.MoveWristCommand;

public class ElevatorSubsystem extends SubsystemBase {

  private SparkMax elevatorLeft;
  private SparkMax elevatorRight;
  private SparkMaxConfig elevatorLeftConfig;
  private SparkMaxConfig elevatorRightConfig;

  public RelativeEncoder elevatorRightEncoder;

  public static double currentPosition; 

  public ElevatorSubsystem() {

    elevatorLeftConfig = new SparkMaxConfig();
    elevatorLeftConfig
      .smartCurrentLimit(10) // TODO increase if needed
      .idleMode(IdleMode.kBrake)
      .follow( 10, true);

    elevatorRightConfig = new SparkMaxConfig();
    elevatorRightConfig
      .smartCurrentLimit(20) // TODO increase if needed
      .idleMode(IdleMode.kBrake)
      .softLimit
        .forwardSoftLimit(0.0)
        .forwardSoftLimitEnabled(false) // TODO update to true after setting soft limits
        .reverseSoftLimit(0.0)
        .reverseSoftLimitEnabled(false);
      
    elevatorLeft = new SparkMax(9, MotorType.kBrushless);
    elevatorLeft.configure(elevatorLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    elevatorRight = new SparkMax(10, MotorType.kBrushless);
    elevatorRight.configure(elevatorRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    elevatorRightEncoder = elevatorRight.getEncoder();
  }

  public void moveElevator(double speed) {
    elevatorRight.set(speed);
  }

  @Override
  public void periodic() {
    currentPosition = elevatorRightEncoder.getPosition();
    SmartDashboard.putNumber("Elevator Position: ", currentPosition);
    SmartDashboard.putNumber("Elevator Setpoint", ElevatorCommand.elevatorSetpoint);

    

    // SmartDashboard.putNumber("Selected Gampeiece", selectGamepiece);

    // if(RobotContainer.operatorController.back().getAsBoolean()){
    //   selectGamepiece = 1;
    // }
      
    // if(RobotContainer.operatorController.start().getAsBoolean()){
    //   selectGamepiece = 2;
    // }
  }

}
