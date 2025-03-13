// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.swervedrive.drivebase;

// import edu.wpi.first.math.Vector;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants;
// import frc.robot.subsystems.swervedrive.SwerveSubsystem;
// import java.util.List;
// import java.util.function.BooleanSupplier;
// import java.util.function.DoubleSupplier;
// import swervelib.SwerveController;
// import swervelib.math.SwerveMath;

// /**
//  * An example command that uses an example subsystem.
//  */
// public class AbsoluteFieldDrive extends Command
// {

//   private final SwerveSubsystem swerve;
//   private final DoubleSupplier  vX, vY, heading;
//   private final BooleanSupplier autoAlign;

//   private final PIDController yController = new PIDController(0.05, 0, 0);
//   private final PIDController xController = new PIDController(0.05, 0, 0);
//   private final PIDController thetaController = new PIDController(1., 0, 0); 
//   private double tx = 0;
//   private double ty = 0;
//   /**
//    * Used to drive a swerve robot in full field-centric mode.  vX and vY supply translation inputs, where x is
//    * torwards/away from alliance wall and y is left/right. headingHorzontal and headingVertical are the Cartesian
//    * coordinates from which the robot's angle will be derived— they will be converted to a polar angle, which the robot
//    * will rotate to.
//    *
//    * @param swerve  The swerve drivebase subsystem.
//    * @param vX      DoubleSupplier that supplies the x-translation joystick input.  Should be in the range -1 to 1 with
//    *                deadband already accounted for.  Positive X is away from the alliance wall.
//    * @param vY      DoubleSupplier that supplies the y-translation joystick input.  Should be in the range -1 to 1 with
//    *                deadband already accounted for.  Positive Y is towards the left wall when looking through the driver
//    *                station glass.
//    * @param heading DoubleSupplier that supplies the robot's heading angle.
//    */
//   public AbsoluteFieldDrive(SwerveSubsystem swerve, DoubleSupplier vX, DoubleSupplier vY,
//                             DoubleSupplier heading, BooleanSupplier autoAlign)
//   {
//     this.swerve = swerve;
//     this.vX = vX;
//     this.vY = vY;
//     this.heading = heading;
//     this.autoAlign = autoAlign;
//     addRequirements(swerve);
//   }

//   @Override
//   public void initialize()
//   {
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute()
//   {
//     ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
//     tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    
//     // Get the desired chassis speeds based on a 2 joystick module.

//     ChassisSpeeds desiredSpeeds = swerve.getTargetSpeeds(vX.getAsDouble(), vY.getAsDouble(),
//                                                          new Rotation2d(heading.getAsDouble() * Math.PI));

//     // Limit velocity to prevent tippy
//     Translation2d translation = SwerveController.getTranslation2d(desiredSpeeds);
//     translation = SwerveMath.limitVelocity(translation, swerve.getFieldVelocity(), swerve.getPose(),
//                                            Constants.LOOP_TIME, Constants.ROBOT_MASS, List.of(Constants.CHASSIS),
//                                            swerve.getSwerveDriveConfiguration());
//     SmartDashboard.putNumber("LimitedTranslation", translation.getX());
//     SmartDashboard.putString("Translation", translation.toString());

//     // Make the robot move
//     if(this.autoAlign.getAsBoolean()){
//       //var translationAuto = SwerveController.getTranslation2d(new ChassisSpeeds(xController.calculate(tx, 0), yController.calculate(ty, 0), 0));
//       //swerve.drive(translationAuto, 0, false);
//     }
//     else{
//       swerve.drive(translation, desiredSpeeds.omegaRadiansPerSecond, true);
//     }
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted)
//   {
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished()
//   {
//     return false;
//   }


// }
