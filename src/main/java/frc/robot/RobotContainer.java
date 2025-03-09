// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.FeederCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.LauncherCommand;
import frc.robot.commands.MoveWristCommand;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.commands.swervedrive.drivebase.AbsoluteFieldDrive;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.WristSubsytem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;

import swervelib.SwerveInputStream;


public class RobotContainer
{
  final CommandXboxController driverXbox = new CommandXboxController(1);
  final CommandXboxController operatorXbox = new CommandXboxController(0);

  private final IntakeSubsystem intake = new IntakeSubsystem();
  private final WristSubsytem wrist = new WristSubsytem();
  private final FeederSubsystem feeder = new FeederSubsystem();
  private final LauncherSubsystem launcher = new LauncherSubsystem();
  private final ElevatorSubsystem elevator = new ElevatorSubsystem();

  private final SwerveSubsystem drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve/neo"));

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> driverXbox.getLeftY() * -1,
                                                                () -> driverXbox.getLeftX() * -1)
                                                            .withControllerRotationAxis(driverXbox::getRightX)
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(1.0) // changed from .8, might increase speed?
                                                            .allianceRelativeControl(true)
                                                            .headingOffset(true);
                                                                        
                  
  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverXbox::getRightX,
                                                                                             driverXbox::getRightY)
                                                           .headingWhile(true);

  // Intake function commands
  Command intakeStill = new IntakeCommand(intake, 0);
  Command intakeCollect = new IntakeCommand(intake, -0.9);
  Command intakeEject = new IntakeCommand(intake, 0.5);
  Command intakePulse = new IntakeCommand(intake, -0.1);
  Command intakeFeed = new IntakeCommand(intake, -0.9);
  Command intakeAutoCollect = new IntakeCommand(intake, -0.9);
  Command intakeAutoStill = new IntakeCommand(intake, 0);
  Command intakeAutoFeed = new IntakeCommand(intake, -0.3);


  // Wrist position commands
  Command wristStow = new MoveWristCommand(wrist, 0);
  Command wristGroundIntake = new MoveWristCommand(wrist, 1);
  Command wristHold = new MoveWristCommand(wrist, 2);  
  Command wristProcessor = new MoveWristCommand(wrist, 3);
  Command wristReefIntake = new MoveWristCommand(wrist, 4);
  Command wristLaunch = new MoveWristCommand(wrist, 6);
  Command wristAutoReef = new MoveWristCommand(wrist, 4);
  Command wristAutoStow = new MoveWristCommand(wrist, 0);
  Command wristAutoLaunch = new MoveWristCommand(wrist, 6);

  // Command wristCoralTop = new MoveWristCommand(wrist, 7);
  
  
  // Elevator position commands
  Command elevatorGroundIntake = new ElevatorCommand(elevator, 1);
  Command elevatorL2Intake = new ElevatorCommand(elevator, 3);  
  Command elevatorL3Intake = new ElevatorCommand(elevator, 4);
  Command elevatorLaunch = new ElevatorCommand(elevator, 5);
  Command elevatorAutoReef = new ElevatorCommand(elevator, 3);
  Command elevatorAutoLaunch = new ElevatorCommand(elevator, 5);

  // Command elevatorCoralTop = new ElevatorCommand(elevator, 3);
  // Command elevatorProcessor = new ElevatorCommand(elevator, 3);
  // Command elevatorClimb = new ElevatorCommand(elevator, 3);

  // Feeder commands
  Command feederLaunch = new FeederCommand(feeder, -0.33);
  Command feederStill = new FeederCommand(feeder, 0);
  Command feederAutoLaunch = new FeederCommand(feeder, -0.33);
  Command feederAutoStill = new FeederCommand(feeder, 0);
  
  // Launcher commands
  Command launchDelay = new WaitCommand(1.0);
  Command launchGamepiece = new LauncherCommand(launcher, -0.45);
  Command launchStill = new LauncherCommand(launcher, 0);
  Command autoLaunchDelay = new WaitCommand(1.0);
  Command autoLaunchGamepiece = new LauncherCommand(launcher, -0.45);
  Command autoLaunchStill = new LauncherCommand(launcher, 0);

  
  //Swerve Commands
  Command driveFieldOrientedAngularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
  Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);
  // Command alignDrive = new AbsoluteFieldDrive(drivebase, () -> 0, () -> 0, () -> 0, () -> true);
  
  Command driveWithHeadingSnaps = new AbsoluteDriveAdv(drivebase,
                                                                   () -> -MathUtil.applyDeadband(driverXbox.getLeftY(),
                                                                                                OperatorConstants.DEADBAND),
                                                                   () -> -MathUtil.applyDeadband(driverXbox.getLeftX(),
                                                                                                OperatorConstants.DEADBAND),
                                                                   () -> MathUtil.applyDeadband(driverXbox.getRightX(),
                                                                                                OperatorConstants.RIGHT_X_DEADBAND),
                                                                   driverXbox.getHID()::getYButtonPressed,
                                                                   driverXbox.getHID()::getAButtonPressed,
                                                                   driverXbox.getHID()::getXButtonPressed,
                                                                   driverXbox.getHID()::getBButtonPressed);

  // Auto Commands
  private double autoDriveStraightX = 0.5;
  private double autoDriveStraightY = 0.0;  
  private double autoRotation = 0.0;

  private double autoDriveSidewaysX = 0.4;
  private double autoDriveSidewaysY = 0.8; 

  Command autoDriveStraight = new AbsoluteFieldDrive(drivebase,
                                              () -> -autoDriveStraightX,
                                              () -> -autoDriveStraightY,
                                              () -> -autoRotation,
                                              () -> false)
                                              .withTimeout(7.0);

  Command autoDriveStraightCollect = new AbsoluteFieldDrive(drivebase,
                                              () -> -autoDriveStraightX,
                                              () -> -autoDriveStraightY,
                                              () -> -autoRotation,
                                              () -> false).withTimeout(7.0);

  Command autoDriveSideways = new AbsoluteFieldDrive(drivebase,
                                              () -> autoDriveSidewaysX,
                                              () -> autoDriveSidewaysY,
                                              () -> -autoRotation,
                                              () -> false)
                                              .withTimeout(5.0);

  Command autoReefCollect = elevatorAutoReef
                            .alongWith(wristAutoReef
                            .alongWith(intakeAutoCollect))
                            .until(IntakeSubsystem::algaeCollected)
                            .andThen(wristAutoStow
                            .alongWith(intakeAutoStill));

  Command autoLaunchCommand = autoLaunchGamepiece
                                .alongWith(elevatorAutoLaunch
                                .alongWith(wristAutoLaunch
                                .alongWith(autoLaunchDelay
                                    .andThen(intakeAutoFeed
                                    .alongWith(feederAutoLaunch)))));
                                
  // Auto Sequences
  // Command autoCollectAndLaunchSequence = autoReefCollect.withTimeout(5.0)
  //                                         .andThen(autoLaunchCommand).withTimeout(2.0);
  Command autoCollectAndMoveSequence = autoDriveStraightCollect
                                        .alongWith(autoReefCollect)
                                        .andThen(autoDriveSideways);


  public RobotContainer()
  {
    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    drivebase.setDefaultCommand(driveWithHeadingSnaps);
    // drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);
    // drivebase.setDefaultCommand(driveFieldOrientedDirectAngle);
    intake.setDefaultCommand(intakeStill);
    wrist.setDefaultCommand(wristStow);
    feeder.setDefaultCommand(feederStill);
    launcher.setDefaultCommand(launchStill);
  }

  
  private void configureBindings()
  {
    // Driver Bindings
    driverXbox.leftBumper().onTrue(new InstantCommand(drivebase::zeroGyro)); 
    // driverXbox.rightBumper().whileTrue(alignDrive);

    // Oerator Bindings
    // operatorXbox.rightBumper().whileTrue(new ConditionalCommand(wristGroundIntake, wristReefIntake, elevator::checkGroundPosition)
    //                           .alongWith(intakeCollect)
    //                           .until(() -> IntakeSubsystem.algaeCollected()));

    operatorXbox.rightBumper().whileTrue(new ConditionalCommand(wristGroundIntake, wristReefIntake, elevator::checkGroundPosition)
                              .alongWith(intakeCollect)
                              .until(() -> IntakeSubsystem.algaeCollected()));

    operatorXbox.leftBumper().whileTrue(wristProcessor.alongWith(intakeEject));
    operatorXbox.rightTrigger().whileTrue(launchGamepiece
                                .alongWith(wristLaunch
                                .alongWith(launchDelay
                                    .andThen(intakeFeed
                                    .alongWith(feederLaunch)))));

    operatorXbox.a().onTrue(elevatorGroundIntake);
    operatorXbox.x().onTrue(elevatorL2Intake);
    operatorXbox.y().onTrue(elevatorL3Intake);
    operatorXbox.b().onTrue(elevatorLaunch);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // return null;
    // return driveToReefAuto;
    // return autoDriveSideways;
    // return autoReefCollect;
    // return autoLaunchCommand;
    // return autoDriveStraight.raceWith(autoReefCollect);
    // return autoCollectAndLaunchSequence;
    return autoCollectAndMoveSequence;
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}