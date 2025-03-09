// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.FeederSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class FeederCommand extends Command{
  private final FeederSubsystem m_feeder;
  public double feedSpeed; 

  public FeederCommand(FeederSubsystem feeder, double speed){
      m_feeder = feeder;
      feedSpeed = speed; 
      addRequirements(m_feeder);
  }
  

  @Override
  public void initialize(){}

  @Override
  public void execute(){
      m_feeder.spinFeeder(feedSpeed);
  }

  @Override
  public boolean isFinished(){
      return false;
  }
}
