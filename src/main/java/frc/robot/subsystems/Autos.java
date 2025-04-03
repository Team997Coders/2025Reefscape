package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
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


    public Command oneCoral(Pose2d tagLocation, Pose2d backup) {
        Command goTo = new goToLocation(drivebase, tagLocation);
        Command backupGoTo = new goToLocation(drivebase, backup);

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
            backupGoTo).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }


    //use this
    public Command LeftTag21Blue = oneCoral(Constants.Auto.Blue.tag21Left, Constants.Auto.Blue.tag21Backup);

    //coordinates may or may not work 
    public Command LeftTag20Blue = oneCoral(Constants.Auto.Blue.tag20Left, Constants.Auto.Blue.tag22Backup);

    //use this
    public Command LeftTag10Red = oneCoral(Constants.Auto.Red.tag10Left, Constants.Auto.Red.tag10Backup);

    //use this
    public Command RightTag10Red = oneCoral(Constants.Auto.Red.tag10Right, Constants.Auto.Red.tag10Backup);

    //use this
    public Command LeftTag11Red = oneCoral(Constants.Auto.Red.tag11Left, Constants.Auto.Red.tag11Backup);

 
}
