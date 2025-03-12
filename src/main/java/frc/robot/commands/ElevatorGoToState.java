package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorState;

public class ElevatorGoToState extends Command {

    private ElevatorState state;
    private Elevator elevator;
    
    public ElevatorGoToState(Elevator elevator, ElevatorState state) {
        this.state = state;
        this.elevator = elevator;
    }


    @Override 
    public void initialize() {
        elevator.setState(state);
    }

    @Override 
    public void execute() {
        SmartDashboard.putBoolean("elevator at target", elevator.pidAtTarget());
    }

    @Override 
    public void end(boolean interrupted) {

    }

    @Override 
    public boolean isFinished() {
        return elevator.pidAtTarget();
    }


}
