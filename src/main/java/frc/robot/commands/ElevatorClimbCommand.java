package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorClimbCommand extends Command {
    private final ElevatorSubsystem m_elevator;
    public static double elevatorSetpoint; // Encoder position value that corresponds to arm position

    public ElevatorClimbCommand(ElevatorSubsystem elevator) {
        m_elevator = elevator;
        addRequirements(m_elevator);
    }


    @Override
    public void initialize(){

    }

    @Override
    public void execute() {
        if(ElevatorSubsystem.currentPosition < -1.0)
            m_elevator.moveElevator(0.4);
        if(ElevatorSubsystem.currentPosition > -1.0)
            m_elevator.moveElevator(0.02);
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}