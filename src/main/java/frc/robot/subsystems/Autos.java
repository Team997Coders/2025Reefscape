package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.robot.Constants;
import frc.robot.commands.ElevatorGoToState;
import frc.robot.commands.goToLocation;
import frc.robot.subsystems.Elevator.ElevatorState;

public class Autos extends SubsystemBase {
    Optional<Alliance> ally = DriverStation.getAlliance();

    private Drivebase drivebase; 
    private Elevator elevator;
    private Coral coral;
    private Algae algae;

    public Autos(Drivebase drivebase, Elevator elevator, Coral coral, Algae algae) {
        this.drivebase = drivebase;
        this.elevator = elevator;
        this.coral = coral;
        this.algae = algae;

    }



    //use this
    public Command LeftTag21Blue() {
        Command goTo = new goToLocation(drivebase, Constants.Auto.Blue.tag21Left).withTimeout(8);
    
        return new SequentialCommandGroup( 
            new ParallelCommandGroup(
                goTo, 
                new ElevatorGoToState(elevator, ElevatorState.L2).withTimeout(3), 
                algae.AlgaeOuttake(Constants.Algae.spinnyMotorConfig).withTimeout(.25)), 
            new ElevatorGoToState(elevator, ElevatorState.L4).withTimeout(3),
            coral.manualMoveCoralMotorsOutake(), 
            new WaitCommand(.5),  
            coral.CoralStop(),
            new WaitCommand(1),
            new goToLocation(drivebase, Constants.Auto.Blue.tag21Backup),
            new ElevatorGoToState(elevator, ElevatorState.SOURCE)).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }

    //coordinates may or may not work 
    public Command LeftTag20Blue() {
        Command goTo = new goToLocation(drivebase, Constants.Auto.Blue.tag20Left).withTimeout(8);

        return new SequentialCommandGroup( 
            new ParallelCommandGroup(
                goTo, 
                new ElevatorGoToState(elevator, ElevatorState.L2).withTimeout(3), 
                algae.AlgaeOuttake(Constants.Algae.spinnyMotorConfig).withTimeout(.25)), 
            new ElevatorGoToState(elevator, ElevatorState.L4).withTimeout(3),
            coral.manualMoveCoralMotorsOutake(), 
            new WaitCommand(.5),
            coral.CoralStop(),
            new WaitCommand(1),
            new goToLocation(drivebase, Constants.Auto.Blue.tag20Backup),
            new ElevatorGoToState(elevator, ElevatorState.SOURCE)).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }

    //use this
    public Command LeftTag10Red() {
        Command goTo = new goToLocation(drivebase, Constants.Auto.Red.tag10Left).withTimeout(8);

        return new SequentialCommandGroup( 
            new ParallelCommandGroup(
                goTo, 
                new ElevatorGoToState(elevator, ElevatorState.L2).withTimeout(3), 
                algae.AlgaeOuttake(Constants.Algae.spinnyMotorConfig).withTimeout(.25)), 
            new ElevatorGoToState(elevator, ElevatorState.L4).withTimeout(3),
            coral.manualMoveCoralMotorsOutake(), 
            new WaitCommand(.5),
            coral.CoralStop(),
            new WaitCommand(1),
            new goToLocation(drivebase, Constants.Auto.Red.tag10Backup),
            new ElevatorGoToState(elevator, ElevatorState.SOURCE)).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }

    //use this
    public Command RightTag10Red() {
        Command goTo = new goToLocation(drivebase, Constants.Auto.Red.tag10Right).withTimeout(8);

        return new SequentialCommandGroup( 
            new ParallelCommandGroup(
                goTo, 
                new ElevatorGoToState(elevator, ElevatorState.L2).withTimeout(3), 
                algae.AlgaeOuttake(Constants.Algae.spinnyMotorConfig).withTimeout(.25)), 
            new ElevatorGoToState(elevator, ElevatorState.L4).withTimeout(3),
            coral.manualMoveCoralMotorsOutake(), 
            new WaitCommand(.5),
            coral.CoralStop(),
            new WaitCommand(1),
            new goToLocation(drivebase, Constants.Auto.Red.tag10Backup),
            new ElevatorGoToState(elevator, ElevatorState.SOURCE)).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }

    //use this
    public Command LeftTag11Red() {
        Command goTo = new goToLocation(drivebase, Constants.Auto.Red.tag11Left).withTimeout(8);

        return new SequentialCommandGroup( 
            new ParallelCommandGroup(
                goTo, 
                new ElevatorGoToState(elevator, ElevatorState.L2).withTimeout(3), 
                algae.AlgaeOuttake(Constants.Algae.spinnyMotorConfig).withTimeout(.25)), 
            new ElevatorGoToState(elevator, ElevatorState.L4).withTimeout(3),
            coral.manualMoveCoralMotorsOutake(), 
            new WaitCommand(.5),
            coral.CoralStop(),
            new WaitCommand(1),
            new goToLocation(drivebase, Constants.Auto.Red.tag11Backup),
            new ElevatorGoToState(elevator, ElevatorState.SOURCE)).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }

    // public Command LeftTag22Blue() { /* DON'T USE */
        
    //     Command goTo = new goToLocation(drivebase, Constants.Auto.Blue.tag22Left).withTimeout(8);
    //     Command backup =  new goToLocation(drivebase, Constants.Auto.Blue.tag22Backup);

    //     return new SequentialCommandGroup( 
    //         new ParallelCommandGroup(
    //             goTo, 
    //             new ElevatorGoToState(elevator, ElevatorState.L2).withTimeout(3), 
    //             algae.AlgaeOuttake(Constants.Algae.spinnyMotorConfig).withTimeout(.25)), 
    //         new ElevatorGoToState(elevator, ElevatorState.L4).withTimeout(3),
    //         coral.manualMoveCoralMotorsOutake(), 
    //         new WaitCommand(.5),
    //         coral.CoralStop(),
    //         new WaitCommand(1),
    //         backup,
    //         new ElevatorGoToState(elevator, ElevatorState.SOURCE)).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    // }

}
