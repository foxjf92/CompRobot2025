// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeCommand extends Command{
  private final IntakeSubsystem m_intake;
  public double intakeSpeed; 

  public IntakeCommand(IntakeSubsystem intake, double speed){
      m_intake = intake;
      intakeSpeed = speed; 
      addRequirements(m_intake);
  }
  

  @Override
  public void initialize(){}

  @Override
  public void execute(){
      m_intake.spinIntake(intakeSpeed);
  }

  @Override
  public boolean isFinished(){
    if(IntakeSubsystem.algaeCollected())
        return true;  
    return false;
  }
}
