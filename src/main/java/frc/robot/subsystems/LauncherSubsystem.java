package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LauncherSubsystem extends SubsystemBase {
    
    private SparkMax launcherMotor1;
    private SparkMax launcherMotor2;
    private SparkMaxConfig launcherConfig;
   
    public LauncherSubsystem() {
        launcherConfig = new SparkMaxConfig();
        launcherConfig
          .smartCurrentLimit(80)
          .idleMode(IdleMode.kBrake);

        launcherMotor1 = new SparkMax(12, MotorType.kBrushless);
        launcherMotor1.configure(launcherConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        launcherMotor2 = new SparkMax(13, MotorType.kBrushless);
        launcherMotor2.configure(launcherConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public void launch(double launchSpeed) {
        launcherMotor1.set(launchSpeed);
        launcherMotor2.set(-launchSpeed);
    }

}
