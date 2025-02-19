package frc.robot.subsystems.automation;

import edu.wpi.first.wpilibj.DriverStation.Alliance;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.CoralIntake;
import frc.robot.commands.CoralManualControl;
import frc.robot.commands.CoralOutTake;
import frc.robot.commands.ElevatorAutomaticControl;
import frc.robot.commands.ElevatorManualControl;
import frc.robot.commands.goToLocation;
import frc.robot.exceptions.elevatorNotAtTarget;
import frc.robot.exceptions.noNextAction;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorState;
import frc.robot.subsystems.buttonBox.ButtonBox;
import frc.robot.subsystems.buttonBox.buttonCommands.goPressed;

public class AutomaticSystemsCopy extends SubsystemBase
{
    private Pathplanning pathplanning;
    private ButtonBox buttonBox;
    private Status status;
    private Alliance alliance;

    private Drivebase drivebase;
    private Coral coral;
    private Elevator elevator;
    
    private boolean beamBreakTriggered;
    public boolean goClick;
    
    private XboxController driveController;

    private Command coralShootCommand;
    private Command coralIntakeCommand;
    private Command goPressedCommand;
    private Command currentDriveCommand;
    private Command manualDriveCommand;
    private Command manualElevatorCommand;
    private Command semiAutomaticElevatorCommand;
    private Command manualCoralCommand;

    public AutomaticSystemsCopy(XboxController buttonBox, Drivebase drivebase, Command driveCommand, Coral coral, Elevator elevator, CommandXboxController c_driveController)
    {
        this.buttonBox = new ButtonBox(buttonBox);
        this.pathplanning = new Pathplanning(this.buttonBox.reefSide, this.buttonBox.rightScore, this.buttonBox.leftScore, this.buttonBox.rightSource, this.buttonBox.leftSource);
        this.status = new Status();
        this.alliance = Alliance.Red;

        this.drivebase = drivebase;
        this.coral = coral;
        this.elevator = elevator;

        this.coralIntakeCommand = new CoralIntake(coral);
        this.coralShootCommand = new CoralOutTake(coral);
        this.manualDriveCommand = driveCommand;
        this.currentDriveCommand = null;
        this.buttonBox.go.onTrue(this.goPressedCommand);
        this.manualElevatorCommand = new ElevatorManualControl(elevator, () -> c_driveController.povUp().getAsBoolean(), () -> c_driveController.povDown().getAsBoolean());
        this.semiAutomaticElevatorCommand = new ElevatorAutomaticControl(elevator, () -> c_driveController.povUp().getAsBoolean(), () -> c_driveController.povDown().getAsBoolean());
        this.manualCoralCommand = new CoralManualControl(coral, driveController);
    }
