package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorClimbCommand extends Command {
    private final ElevatorSubsystem m_elevator;
    public static double elevatorSetpoint; // Encoder position value that corresponds to arm position

    public final double kP = 0.0; //start at .01?
    public final double kI = 0.0; //4th
    public final double kD = 0.0; //3rd
    public final double arbFF = 0.05; // Start Here
    
    // private final TrapezoidProfile.Constraints;
    
    // private ProfiledPIDController
    private PIDController m_elevatorPID = new PIDController(kP,kI,kD); // look @ profiled PID maybe?

    public ElevatorClimbCommand(ElevatorSubsystem elevator) {
        m_elevator = elevator;
        addRequirements(m_elevator);
    }


    @Override
    public void initialize(){

    }

    @Override
    public void execute() {
        // double controlEffort = - arbFF + m_elevatorPID.calculate(m_elevator.elevatorRightEncoder.getPosition(), Constants.ElevatorConstants.elevatorClimbPosition); // includes FF input to fight gravity, negative encoder is up

        // m_elevator.moveElevator(controlEffort);

        //Try to figure out PID but test slowly with this first
        m_elevator.moveElevator(0.1);
        
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}