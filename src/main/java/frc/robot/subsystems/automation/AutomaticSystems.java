package frc.robot.subsystems.automation;

import edu.wpi.first.wpilibj.DriverStation.Alliance;

import java.lang.annotation.ElementType;

import com.fasterxml.jackson.databind.jsontype.impl.StdTypeResolverBuilder;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.commands.CoralIntake;
import frc.robot.commands.CoralManualControl;
import frc.robot.commands.CoralOutTake;
import frc.robot.commands.Drive;
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

public class AutomaticSystems extends SubsystemBase
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

    public AutomaticSystems(XboxController buttonBox, Drivebase drivebase, Command driveCommand, Coral coral, Elevator elevator, XboxController driveController, CommandXboxController c_driveyController)
    {
        this.buttonBox = new ButtonBox(buttonBox);
        this.pathplanning = new Pathplanning(this.buttonBox.reefSide, this.buttonBox.rightScore, this.buttonBox.leftScore, this.buttonBox.rightSource, this.buttonBox.leftSource);
        this.status = new Status();
        this.alliance = DriverStation.getAlliance().orElseThrow();

        this.drivebase = drivebase;
        this.coral = coral;
        this.elevator = elevator;

        this.goClick = false;
        this.beamBreakTriggered = false;
        this.driveController = driveController;

        this.coralIntakeCommand = new CoralIntake(coral);
        this.coralShootCommand = new CoralOutTake(coral);
        this.goPressedCommand = new goPressed(this);
        this.manualDriveCommand = driveCommand;
        this.currentDriveCommand = null;
        this.buttonBox.go.onTrue(this.goPressedCommand);
        this.manualElevatorCommand = new ElevatorManualControl(elevator, () -> c_driveyController.povUp().getAsBoolean(), () -> c_driveyController.povDown().getAsBoolean());
        this.semiAutomaticElevatorCommand = new ElevatorAutomaticControl(elevator, () -> c_driveyController.povUp().getAsBoolean(), () -> c_driveyController.povDown().getAsBoolean());
        this.manualCoralCommand = new CoralManualControl(coral, driveController);
    }

    public void automaticDriving()
    {
        switch (status.driveGoal) {
            case "source":
                if (currentDriveCommand.isFinished() && status.currentDriveAction == "source")
                {
                    status.currentDriveAction = "finished";
                    status.currentDriveLocation = "source";
                } else if ("source" != status.currentDriveAction)
                {
                    try
                    {
                        currentDriveCommand = new goToLocation(drivebase, pathplanning.getSourceLocation(alliance));
                        currentDriveCommand.schedule();
                        status.currentDriveAction = "source";
                        status.currentDriveLocation = "moving";
                    } catch(Exception e)
                    {
                        e.printStackTrace();
                    }
                }
                break;
            case "reef":
            if (currentDriveCommand.isFinished() && status.currentDriveAction == "reef")
            {
                status.currentDriveAction = "finished";
                status.currentDriveLocation = "reef";
            } else if ("source" != status.currentDriveAction)
            {
                try
                {
                    currentDriveCommand = new goToLocation(drivebase, pathplanning.getReefLocation(alliance));
                    currentDriveCommand.schedule();
                    status.currentDriveAction = "reef";
                    status.currentDriveLocation = "moving";
                } catch(Exception e)
                {
                    e.printStackTrace();
                }
            }
            break;
            case "processor":
            if (currentDriveCommand.isFinished() && status.currentDriveAction == "processor")
            {
                status.currentDriveAction = "finished";
                status.currentDriveLocation = "processor";
            } else if ("source" != status.currentDriveAction)
            {
                try
                {
                    currentDriveCommand = new goToLocation(drivebase, pathplanning.getProcessorLocation(alliance));
                    currentDriveCommand.schedule();
                    status.currentDriveAction = "processor";
                    status.currentDriveLocation = "moving";
                } catch(Exception e)
                {
                    e.printStackTrace();
                }
            }
            break;
        }
        if (status.currentDriveAction == "finished")
        {
            try
            {
            nextAction(false);
            } catch(elevatorNotAtTarget e)
            {
                status.callNextAction = true;
            } catch(Exception e)
            {
                e.printStackTrace();
            }
            status.currentDriveAction = "";
        }
    }

    public void automaticSubsystems()
    {
        switch (status.elevatorGoal) {
            case "elevatorSource":
            elevator.goToStateCommand(ElevatorState.SOURCE);
            try
            {
                if (elevator.elevatorAtTarget())
                {
                    status.elevatorAction = "finished";
                } else
                {
                    status.elevatorAction = "inProgress";
                }
            } catch(Exception e)
            {
                e.printStackTrace();
            }
            if (status.coralIndexGoal == "index")
            {
                if (status.coralIndexAction == "hold")
                {
                    coralIntakeCommand.schedule();
                    status.coralIndexAction = "index";
                }
                if (coral.BeamBrake1() && !beamBreakTriggered)
                {
                    //run once
                    beamBreakTriggered = true;
                    try
                    {
                    nextAction(true);
                    } catch(elevatorNotAtTarget e)
                    {
                    status.callNextAction = true;
                    } catch(Exception e)
                    {
                    e.printStackTrace();
                    }
                }
                if (!coral.BeamBrake1() && beamBreakTriggered)
                {
                    //when intake coral has stops triggering first beam break
                    beamBreakTriggered = false;
                    status.coralIndexAction = "finished";
                    coralIntakeCommand.cancel();
                    try
                    {
                    nextAction(false);
                    } catch(Exception e)
                    {
                    e.printStackTrace();
                    }
                }
            }
            break;
            case "elevatorLevel1":
            elevator.goToStateCommand(ElevatorState.L1);
            checkSubsystemStatus();
            break; 
            case "elevatorLevel2":
            elevator.goToStateCommand(ElevatorState.L2);
            checkSubsystemStatus();
            break; 
            case "elevatorLevel3":
            elevator.goToStateCommand(ElevatorState.L3);
            checkSubsystemStatus();
            break; 
            case "elevatorLevel4":
            elevator.goToStateCommand(ElevatorState.L4);
            checkSubsystemStatus();
            break;            
        }
    }

    public void checkSubsystemStatus()
    {
        try
        {
        if (elevator.elevatorAtTarget())
        {
            status.elevatorAction = "finished";
        } else
        {
            status.elevatorAction = "inProgress";
        }
        } catch(Exception e)
        {
            e.printStackTrace();
        }
        if (status.coralIndexGoal == "shoot")
        {
            if (status.coralIndexAction == "hold")
            {
                coralShootCommand.schedule();
                status.coralIndexAction = "shoot";
            }
            if (coralShootCommand.isFinished())
            {
                status.coralIndexAction = "finished";
                try
                {
                nextAction(true);
                } catch(elevatorNotAtTarget e)
                {
                    status.callNextAction = true;
                } catch(Exception e)
                {
                    e.printStackTrace();
                }
            }
        }
    }

    public void nextAction(boolean driving) throws noNextAction, elevatorNotAtTarget
    {
        if (driving)
        {
            if (status.coralIndexAction == "finished")
            {
                if (status.coralIndexGoal == "shoot")
                {
                    status.coralIndexGoal = "index";
                    status.driveGoal = "source";
                } else if (status.coralIndexGoal == "index")
                {
                    status.coralIndexGoal = "hold";
                    status.driveGoal = "reef";
                } else 
                {
                    status.coralIndexAction = "";
                    throw new noNextAction("The coral index action was finished but neither index goal was selected (index and shoot)");
                }
            } else
            {
                ///throw new noNextAction("the next command was called while the coralIndexAction was not finished");
            }
        } else
        {
            if (status.currentDriveLocation == "reef")
            {
                if (status.elevatorAction == "finished")
                {
                    status.coralIndexGoal = "shoot";
                    status.callNextAction = false;
                } else
                {
                    throw new elevatorNotAtTarget("next command called while elevator not at target");
                }
            } else if (status.currentDriveLocation == "source")
            {
                if (status.elevatorAction == "finished")
                {
                    status.coralIndexGoal = "intake";
                    status.callNextAction = false;
                } else
                {
                    throw new elevatorNotAtTarget("next command called while elevator not at target");
                }
            } else 
            {
                //throw new noNextAction("current drive location not reef or source");
            }
        }
    }

    @Override
    public void periodic() {
        if (!buttonBox.autoDrive.Flipped())
        {
            manualDriveCommand.schedule();
            currentDriveCommand.cancel();
        } else
        {
            manualDriveCommand.cancel();
            automaticDriving();
        }

        if (!buttonBox.fullAutoElevator.Flipped())
        {
            manualCoralCommand.schedule();

            if (buttonBox.semiAutoElevator.Flipped())
            {
                manualElevatorCommand.cancel();
                semiAutomaticElevatorCommand.schedule();
                elevator.pidControl();
            } else
            {
                manualElevatorCommand.schedule();
                semiAutomaticElevatorCommand.cancel();
            }
        } else
        {
            manualCoralCommand.cancel();
            manualElevatorCommand.cancel();
            semiAutomaticElevatorCommand.cancel();
            automaticSubsystems();
            elevator.pidControl();
        }
        if (buttonBox.fullAutoCycles.Flipped())
        {
            if (status.callNextAction)
            {
                try
                {
                    nextAction(false);
                } catch(elevatorNotAtTarget e)
                {
                    status.callNextAction = true;
                } catch(Exception e)
                {
                    e.printStackTrace();
                }
            }        
        } else
        {
            if (goClick)
            {
                goClick = false;
                try{
                    nextAction(true);
                    nextAction(false);
                } catch(elevatorNotAtTarget e)
                {
                    status.callNextAction = true;
                } catch(Exception e)
                {
                    e.printStackTrace();
                }
            }
        }
    }
}
