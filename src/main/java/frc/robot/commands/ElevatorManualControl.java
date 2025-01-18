package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;

public class ElevatorManualControl extends Command {
    private Elevator elevator;

    private boolean up; 
    private boolean down;
     
    public ElevatorManualControl(Elevator elevator, boolean up, boolean down) {
        this.elevator = elevator;
        this.up = up;
        this.down = down;

        addRequirements(elevator);
    }

    @Override
    public void execute() {
        if (up) {
            elevator.manualControl(Constants.ElevatorConstants.defaultManualOutput);
        } else if (down) {
            elevator.manualControl(-Constants.ElevatorConstants.defaultManualOutput);
        } else {
            elevator.manualControl(0);
        }
    }
}