package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorState;

public class ElevatorAutomaticControl extends Command{
    private Elevator elevator;

    private int state; // elevator state: (0, down) (1, Source) (2, L1)... etc

    private Trigger up;
    private Trigger down;

    public ElevatorAutomaticControl(Elevator elevator, Trigger up, Trigger down) {
        this.elevator = elevator; 
        this.up = up;
        this.down = down;
        

        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        state = elevator.getElevatorStateIndex();
    }

    @Override
    public void execute() {
        up.onTrue(elevator.stateUp());
        down.onTrue(elevator.stateDown());

        SmartDashboard.putBoolean("elevator up boolean", up.getAsBoolean());
        SmartDashboard.putBoolean("elevator down boolean", down.getAsBoolean());
    }    
}
