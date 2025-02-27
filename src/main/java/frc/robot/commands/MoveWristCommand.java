package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.WristSubsytem;

public class MoveWristCommand extends Command{
    private final WristSubsytem m_wrist;
    private int targetPosition; //Symbolic arm position where 1 = ground intake, 2 = amp position, 3 = launch position
    public static double wristSetpoint; // Encoder position value that corresponds to arm position

    public final double kP = 0.01; //2nd 
    public final double kI = 0.0; //4th
    public final double kD = 0.0; //3rd
    public final double arbFF = 0.0; // Start Here
    
    private PIDController m_wristPID = new PIDController(kP,kI,kD); // look @ profiled PID maybe?

    public MoveWristCommand(WristSubsytem wrist, int m_position){
        m_wrist = wrist; 
        targetPosition = m_position;
        addRequirements(m_wrist);
    }


    @Override
    public void initialize(){

        if (targetPosition == 1) {
            wristSetpoint = WristConstants.wristGroundIntakePosition;
        }
        if (targetPosition == 2) {
            wristSetpoint = WristConstants.wristHoldPosition;
        }
        if (targetPosition == 3) {
            wristSetpoint = WristConstants.wristProcessorPosiiton;
        }
        if (targetPosition == 4) {
            wristSetpoint = WristConstants.wristL2IntakePosition;
        }
        if (targetPosition == 5) {
            wristSetpoint = WristConstants.wristL3IntakePosition;
        }
        if (targetPosition == 6) {
            wristSetpoint = WristConstants.wristLaunchPosition;
        }
        
    }

    @Override
    public void execute(){

        double controlEffort = m_wristPID.calculate(m_wrist.wristEncoder.getPosition(), wristSetpoint); // adds FF input to fight gravity, subtract PID output due to encoder inversion

        m_wrist.moveWrist(controlEffort);
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}