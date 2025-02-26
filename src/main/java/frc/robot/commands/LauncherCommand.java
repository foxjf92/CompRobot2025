// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.LauncherSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class LauncherCommand extends Command{
  private final LauncherSubsystem m_launcher;
  public double launchSpeed; 

  public LauncherCommand(LauncherSubsystem launcher, double speed){
      m_launcher = launcher;
      launchSpeed = speed; 
      addRequirements(m_launcher);
  }
  

  @Override
  public void initialize(){
      
  }

  @Override
  public void execute(){
      m_launcher.launch(launchSpeed);
  }

  @Override
  public boolean isFinished(){
      return false;
  }
}
