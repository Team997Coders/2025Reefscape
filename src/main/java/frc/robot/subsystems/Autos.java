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
import frc.robot.commands.goToTag;
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

    
    public Command taxiBlue() {
        Command goTo =  new goToLocation(drivebase, Constants.Auto.Blue.taxi);
        return goTo;
    }

    public Command taxiRed() {
        Command goTo = new goToLocation(drivebase, Constants.Auto.Red.taxi);
       
        return goTo;
    }

    public Command taxi2() {
        Command goTo = ally.get() == Alliance.Blue ? new goToTag(21, 0) : new goToTag(10, 0);
        return goTo;
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
            coral.CoralStop()).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
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
            new goToLocation(drivebase, Constants.Auto.Blue.tag20Backup)).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
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
            new goToLocation(drivebase, Constants.Auto.Red.tag10Backup)).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
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
            new goToLocation(drivebase, Constants.Auto.Red.tag10Backup)).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
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
            new goToLocation(drivebase, Constants.Auto.Red.tag11Backup)).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }

}
