package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.robot.Constants;
import frc.robot.commands.ElevatorGoToState;
import frc.robot.commands.goToLocation;
import frc.robot.subsystems.Elevator.ElevatorState;

public class Autos extends SubsystemBase {

    private Drivebase m_drivebase; 
    private Elevator elevator;
    private Coral coral;
    private Algae algae;

    public Autos(Drivebase drivebase, Elevator elevator, Coral coral, Algae algae) {
        this.m_drivebase = drivebase;
        this.elevator = elevator;
        this.coral = coral;
        this.algae = algae;

    }


    public Command oneCoral(Pose2d tagLocation, Pose2d backup) {
        Command goTo = new goToLocation(m_drivebase, tagLocation);
        Command backupGoTo = new goToLocation(m_drivebase, backup);

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
            backupGoTo,
            new ElevatorGoToState(elevator, ElevatorState.SOURCE)
        ).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }

    public Command twoCoral(Pose2d tagLocation1, Pose2d midLocation, Pose2d sourceLocation, Pose2d tagLocation2, Pose2d tagLocation2Backup) {
        Command firstCoral = oneCoral(tagLocation1, midLocation);
        Command secondCoral = oneCoral(tagLocation2, tagLocation2Backup);

        Command goToSource = new goToLocation(m_drivebase, sourceLocation);

        return new SequentialCommandGroup(
            firstCoral,
            goToSource.until(() -> coral.BeamBrake1()),
            new ParallelCommandGroup(
                new WaitUntilCommand(() -> coral.BeamBrake2()),
                new WaitUntilCommand(() -> !coral.BeamBrake1())
            ),
            secondCoral
        ).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);

    }
 
}
