package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;

public class ElevatorManualControl extends Command {
    private Elevator elevator;

    private Trigger up; 
    private Trigger down;
     
    public ElevatorManualControl(Elevator elevator, Trigger up, Trigger down) {
        this.elevator = elevator;
        this.up = up;
        this.down = down;

        addRequirements(elevator);
    }

    @Override 
    public void initialize() {
      //  elevator.setEncoderPosition(0);
    }

    @Override
    public void execute() {
        if (up.getAsBoolean()) {
            elevator.manualControl(Constants.ElevatorConstants.defaultManualOutput);
        } else if (down.getAsBoolean()) {
            elevator.manualControl(-Constants.ElevatorConstants.defaultManualOutput);
        } else {
            elevator.manualControl(0);
        }
    }
}