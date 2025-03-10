package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorCommand extends Command {
    private final ElevatorSubsystem m_elevator;
    private int targetPosition; //Symbolic arm position where 1 = ground intake, 2 = amp position, 3 = launch position
    public static double elevatorSetpoint; // Encoder position value that corresponds to arm position

    public final double kP = 0.02; //2nd was .01
    public final double kI = 0.0; //4th
    public final double kD = 0.0; //3rd
    public final double arbFF = 0.05; // Start Here
    
    // private final TrapezoidProfile.Constraints;
    
    // private ProfiledPIDController
    private PIDController m_elevatorPID = new PIDController(kP,kI,kD); // look @ profiled PID maybe?

    public ElevatorCommand(ElevatorSubsystem elevator, int m_position) {
        m_elevator = elevator;
        targetPosition = m_position;
        addRequirements(m_elevator);
    }


    @Override
    public void initialize(){
        
        if (targetPosition == 1) {
            elevatorSetpoint = ElevatorConstants.elevatorGroundIntakePosition; // Set in Constants
        }
        // if (targetPosition == 2) {
        //     elevatorSetpoint = ElevatorConstants.elevatorCoralTopIntakePosition; // Set in Constants
        // }
        if (targetPosition == 3) {
            elevatorSetpoint = ElevatorConstants.elevatorL2IntakePosition; 
        }
        if (targetPosition == 4) {
            elevatorSetpoint = ElevatorConstants.eleavtorL3IntakePosition;
        }
        if (targetPosition == 5) {
            elevatorSetpoint = ElevatorConstants.elevatorLaunchPosition;
        }
        // if (targetPosition == 6) {
        //     elevatorSetpoint = ElevatorConstants.elevatorProcessorPosiiton; 
        // }
        // if (targetPosition == 7) {
        //     elevatorSetpoint = ElevatorConstants.elevatorClimbPosition;
        // }
    }

    @Override
    public void execute() {
        double controlEffort = - arbFF + m_elevatorPID.calculate(m_elevator.elevatorRightEncoder.getPosition(), elevatorSetpoint); // adds FF input to fight gravity, negative encoder is up

        m_elevator.moveElevator(controlEffort);
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}