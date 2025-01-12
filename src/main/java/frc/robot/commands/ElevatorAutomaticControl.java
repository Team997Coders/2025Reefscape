package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class ElevatorAutomaticControl extends Command{
    private Elevator elevator;

    private int state; // elevator state: (0, down) (1, L1)... etc

    private boolean up;
    private boolean down;
    
    private boolean upPrevious;
    private boolean downPrevious;

    public ElevatorAutomaticControl(Elevator elevator, boolean up, boolean down) {
        this.elevator = elevator; 
        this.up = up;
        this.down = down;

        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        state = elevator.getElevatorStateIndex();

        upPrevious = up;
        downPrevious = down;
    }

    @Override
    public void execute() {
        boolean upCurrent = up;
        if (upCurrent != upPrevious) {
            state = state+1;
        } 
        upPrevious = upCurrent;

        boolean downCurrent = down;
        if(downCurrent != downPrevious) {
            state = state-1;
        }
        downPrevious = downCurrent;

        elevator.setStateByIndex(state);
    }    
}
