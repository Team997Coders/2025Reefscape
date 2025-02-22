package frc.robot.subsystems.automation;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.goToLocation;
import frc.robot.exceptions.allianceNotInitialized;
import frc.robot.exceptions.noSelectedButton;
import frc.robot.exceptions.outOfBounds;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.buttonBox.ButtonBox;
import frc.robot.subsystems.buttonBox.ButtonyBit;
import frc.robot.subsystems.buttonBox.ButtonyList;

public class AutomaticSystems extends SubsystemBase
{
    private Pathplanning pathplanning;
    private ButtonBox buttonBox;
    private Alliance alliance;

    private Drivebase drivebase;
    private Elevator elevator;

    private Boolean autoDrive = false;
    private Boolean autoElevator = false;
    private Boolean fullAuto = false;
    private Boolean semiAuto = false;
    private Boolean coralBeamBrake = false;
    
    private Command driveCommand;
    
    public AutomaticSystems(XboxController buttonBox, Drivebase drivebase, Elevator elevator, CommandXboxController c_driveController)
    {
        this.buttonBox = new ButtonBox(buttonBox);
        this.pathplanning = new Pathplanning(this.buttonBox.reefSide, this.buttonBox.rightScore, this.buttonBox.leftScore, this.buttonBox.rightSource, this.buttonBox.leftSource);
        this.alliance = Alliance.Red;

        this.drivebase = drivebase;
        this.elevator = elevator;

        this.buttonBox.reefSide.getBitAtIdx(0).onTrue(selectBitCommand(this.buttonBox.reefSide, 0));
        this.buttonBox.reefSide.getBitAtIdx(1).onTrue(selectBitCommand(this.buttonBox.reefSide, 1));
        this.buttonBox.reefSide.getBitAtIdx(2).onTrue(selectBitCommand(this.buttonBox.reefSide, 2));
        this.buttonBox.reefSide.getBitAtIdx(3).onTrue(selectBitCommand(this.buttonBox.reefSide, 3));
        this.buttonBox.reefSide.getBitAtIdx(4).onTrue(selectBitCommand(this.buttonBox.reefSide, 4));
        this.buttonBox.reefSide.getBitAtIdx(5).onTrue(selectBitCommand(this.buttonBox.reefSide, 5));

        this.buttonBox.elevatorLevel.getBitAtIdx(0).onTrue(selectBitCommand(this.buttonBox.elevatorLevel, 0));
        this.buttonBox.elevatorLevel.getBitAtIdx(1).onTrue(selectBitCommand(this.buttonBox.elevatorLevel, 1));
        this.buttonBox.elevatorLevel.getBitAtIdx(2).onTrue(selectBitCommand(this.buttonBox.elevatorLevel, 2));
        this.buttonBox.elevatorLevel.getBitAtIdx(3).onTrue(selectBitCommand(this.buttonBox.elevatorLevel, 3));

        this.buttonBox.scoreSide.getBitAtIdx(0).onTrue(selectBitCommand(this.buttonBox.scoreSide, 0));
        this.buttonBox.scoreSide.getBitAtIdx(1).onTrue(selectBitCommand(this.buttonBox.scoreSide, 1));

        this.buttonBox.sourceSide.getBitAtIdx(0).onTrue(selectBitCommand(this.buttonBox.sourceSide, 0));
        this.buttonBox.sourceSide.getBitAtIdx(1).onTrue(selectBitCommand(this.buttonBox.sourceSide, 1));

        this.buttonBox.autoDrive.onChange(this.switchDriveCommand());
        this.buttonBox.fullAutoElevator.onChange(this.switchElevatorCommand());
        this.buttonBox.fullAutoCycles.onChange(this.switchFullCommand());
        this.buttonBox.semiAutoCycles.onChange(this.switchSemiCommand());
        this.elevator.m_secondBeamBrake.onChange(this.switchBeamBrakeCommand());

        this.buttonBox.go.onTrue(goPressCommand());
    }

    public void switchBeamBrake()
    {
        this.coralBeamBrake = !this.coralBeamBrake;
    }

    public Command switchBeamBrakeCommand()
    {
        return this.runOnce(() -> this.switchBeamBrake());
    }

    public void switchDrive()
    {
        this.autoDrive = !this.autoDrive;
    }

    public Command switchDriveCommand()
    {
        return this.runOnce(() -> this.switchDrive());
    }

    public void switchElevator()
    {
        this.autoElevator = !this.autoElevator;
    }

    public Command switchElevatorCommand()
    {
        return this.runOnce(() -> this.switchElevator());
    }

    public void switchFull()
    {
        this.fullAuto = !this.fullAuto;
    }

    public Command switchFullCommand()
    {
        return this.runOnce(() -> this.switchFull());
    }

    public void switchSemi()
    {
        this.semiAuto = !this.semiAuto;
    }

    public Command switchSemiCommand()
    {
        return this.runOnce(() -> this.switchSemi());
    }

    public Command selectBitCommand(ButtonyList list, int bit)
    {
        return this.runOnce(() -> {
            try {
                list.selectBit(bit);
            } catch (outOfBounds e) {
                e.printStackTrace();
            }
        });
    }

    public Command elevatorGoToCommand(int index)
    {
        return this.runOnce(() -> this.elevator.setStateByIndex(index));
    }

    public void Go() 
    {
        if (this.coralBeamBrake)
        {
            if (fullAuto)
            
            {
                //Ha u though I cared enough to make this work loser
            }
            else if (semiAuto)
            {
                if (autoElevator) {
                    try 
                    {
                        this.elevator.setStateByIndex(this.buttonBox.elevatorLevel.selectedBit().id - 4);
                    } catch (noSelectedButton e) {
                        e.printStackTrace();
                    }
                }
                if (autoDrive)
                {
                    try {
                        driveCommand = new goToLocation(drivebase, pathplanning.getReefLocation(alliance, this.buttonBox.reefSide.selectedBit().id, this.buttonBox.scoreSide.selectedBit().id));
                        driveCommand.schedule();
                    } catch (allianceNotInitialized e) {
                        e.printStackTrace();
                    } catch (noSelectedButton e) {
                        e.printStackTrace();
                    }
                }
            }
        }
        else
        {
            try
            {
            driveCommand = new goToLocation(drivebase, pathplanning.getSourceLocation(this.alliance, this.buttonBox.sourceSide.selectedBit().id));
            driveCommand.schedule();
            this.elevator.setGoal(0);
            } catch(Exception e)
            {
                e.printStackTrace();
            }
        }
    }

    public Command goPressCommand()
    {
            return this.runOnce(() -> Go());
    }

    @Override
    public void periodic() {
        //loggers();
    }

    public void loggers()
    {
        SmartDashboard.putBoolean("Auto Drive", autoDrive);
        SmartDashboard.putBoolean("Auto Elevator", autoElevator);
        SmartDashboard.putBoolean("Semi Cycles", semiAuto);
        SmartDashboard.putBoolean("Full Cycles", fullAuto);

        try{
        SmartDashboard.putNumber("Reef", this.buttonBox.reefSide.selectedBit().id);
        SmartDashboard.putNumber("Elevator", this.buttonBox.elevatorLevel.selectedBit().id);
        SmartDashboard.putNumber("Source", this.buttonBox.sourceSide.selectedBit().id);
        SmartDashboard.putNumber("Score", this.buttonBox.scoreSide.selectedBit().id);
        } catch(Exception e)
        {
            e.printStackTrace();
        }
    }
}
