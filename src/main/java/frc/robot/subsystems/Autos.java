package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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

    
    public Command taxi() {
        Command goTo = ally.get() == Alliance.Blue ? new goToLocation(drivebase, Constants.Auto.Blue.taxi) : new goToLocation(drivebase, Constants.Auto.Red.taxi);
       
        return goTo;
    }

    public Command taxi2() {
        Command goTo = ally.get() == Alliance.Blue ? new goToTag(drivebase, 21, 0) : new goToTag(drivebase, 10, 0);
        return goTo;
    }


    public Command L4Left() {
        Command goTo = ally.get() == Alliance.Blue ? new goToLocation(drivebase, Constants.Auto.Blue.side4Left) : new goToLocation(drivebase, Constants.Auto.Red.side4Left);

        return new SequentialCommandGroup( 
            new ParallelCommandGroup(
                goTo, 
                new ElevatorGoToState(elevator, ElevatorState.L2), 
                algae.AlgaeOuttake(Constants.Algae.spinnyMotorConfig).withTimeout(.25)), 
            new ElevatorGoToState(elevator, ElevatorState.L4),
            coral.manualMoveCoralMotorsOutake(), 
            new WaitCommand(.5),  
            coral.CoralStop());
    }


    public Command L4Right() {
        Command goTo = ally.get() == Alliance.Blue ? new goToLocation(drivebase, Constants.Auto.Blue.side4Right) : new goToLocation(drivebase, Constants.Auto.Red.side4Right);

        return new SequentialCommandGroup( 
            new ParallelCommandGroup(
                goTo, 
                new ElevatorGoToState(elevator, ElevatorState.L2), 
                algae.AlgaeOuttake(Constants.Algae.spinnyMotorConfig).withTimeout(.25)), 
            new ElevatorGoToState(elevator, ElevatorState.L4),
            coral.manualMoveCoralMotorsOutake(), 
            new WaitCommand(.5),  
            coral.CoralStop()
            );
    }


    public Command goToSourceRight() {
        Command goTo = ally.get() == Alliance.Blue ? new goToLocation(drivebase, Constants.Auto.Blue.sourceRight) : new goToLocation(drivebase, Constants.Auto.Red.sourceRight);

        return new SequentialCommandGroup(
            new ElevatorGoToState(elevator, ElevatorState.SOURCE),
            goTo
        );
    }

    
    public Command goToSourceLeft() {
        Command goTo = ally.get() == Alliance.Blue ? new goToLocation(drivebase, Constants.Auto.Blue.sourceLeft) : new goToLocation(drivebase, Constants.Auto.Red.sourceLeft);

        return new SequentialCommandGroup(
            new ElevatorGoToState(elevator, ElevatorState.SOURCE),
            goTo
        );
    }


    public Command TwoL4() {
        
        return new SequentialCommandGroup(
            L4Left(), 
            goToSourceRight(),
            L4Right()
        );
    }
    
}
