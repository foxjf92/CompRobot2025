package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LauncherSubsystem extends SubsystemBase {
    
    private SparkFlex launcherLeft;
    private SparkFlex launcherRight;
    private SparkFlexConfig launcherConfig;
   
    public LauncherSubsystem() {
        launcherConfig = new SparkFlexConfig();
        
        launcherConfig
          .smartCurrentLimit(80)
          .idleMode(IdleMode.kBrake);

        launcherLeft = new SparkFlex(15, MotorType.kBrushless);
        launcherLeft.configure(launcherConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        launcherRight = new SparkFlex(16, MotorType.kBrushless);
        launcherRight.configure(launcherConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public void launch(double launchSpeed) {
        launcherLeft.set(launchSpeed);
        launcherRight.set(-launchSpeed);
    }

}
