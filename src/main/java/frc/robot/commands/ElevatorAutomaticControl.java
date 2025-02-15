package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorState;

public class ElevatorAutomaticControl extends Command{
    private Elevator elevator;

    private int state; // elevator state: (0, down) (1, Source) (2, L1)... etc

    private BooleanSupplier up;
    private BooleanSupplier down;
    
    private boolean upPrevious;
    private boolean downPrevious;

    public ElevatorAutomaticControl(Elevator elevator, BooleanSupplier up, BooleanSupplier down) {
        this.elevator = elevator; 
        this.up = up;
        this.down = down;

        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        state = elevator.getElevatorStateIndex();

        upPrevious = up.getAsBoolean();
        downPrevious = down.getAsBoolean();
    }

    @Override
    public void execute() {
        boolean upCurrent = up.getAsBoolean();
        if (upCurrent && !upPrevious) {
            if (state < 5) {
            state = state+1;
            } else if (state >= 5) {
                state = 5;
            }
        elevator.goToStateCommand(ElevatorState.findByIndex(state));
        } 
        

        boolean downCurrent = down.getAsBoolean();
        if(downCurrent && !downPrevious) {
            if (state > 0) {
            state = state-1;
            } else if (state <= 0) {
                state = 0;
            }
        elevator.goToStateCommand(ElevatorState.findByIndex(state));
        }

        upPrevious = upCurrent;
        downPrevious = downCurrent;
    }    
}
