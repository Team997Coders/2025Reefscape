package frc.robot.subsystems.automation;

import edu.wpi.first.wpilibj.DriverStation.Alliance;

import com.fasterxml.jackson.databind.jsontype.impl.StdTypeResolverBuilder;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.goToLocation;
import frc.robot.exceptions.elevatorNotAtTarget;
import frc.robot.exceptions.noNextAction;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.buttonBox.ButtonBox;

public class AutomaticSystems extends SubsystemBase
{
    private Pathplanning pathplanny;
    private ButtonBox buttonBox;
    private Status status;
    private Drivebase drivebase;
    private Alliance alliance;
    private Command currentDriveCommand;

    public AutomaticSystems(Joystick joystick, Drivebase _drivebase)
    {
        buttonBox = new ButtonBox(joystick);
        pathplanny = new Pathplanning(buttonBox.reefSide, buttonBox.scoreSide, buttonBox.rightSource, buttonBox.leftSource);
        status = new Status();
        drivebase = _drivebase;
        alliance = DriverStation.getAlliance().orElseThrow();
    }

    public void automaticDriving()
    {
        switch (status.driveGoal) {
            case "source":
                if ("source" == status.currentDriveLocation)
                {
                    if (currentDriveCommand.isFinished())
                    {
                        status.currentDriveAction = "finished";
                        status.currentDriveLocation = "source";
                    }
                } else
                {
                    if ("source" != status.currentDriveAction)
                    {
                        try
                        {
                            currentDriveCommand = new goToLocation(drivebase, pathplanny.getSourceLocation(alliance));
                            status.currentDriveAction = "source";
                        } catch(Exception e)
                        {
                            e.printStackTrace();
                        }
                    }
                }
                break;
            case "reef":
                if ("reef" == status.currentDriveLocation)
                {
                    if (currentDriveCommand.isFinished())
                    {
                        status.currentDriveAction = "finished";
                        status.currentDriveLocation = "reef";
                    }
                } else
                {
                    if ("reef" != status.currentDriveAction)
                    {
                        try
                        {
                            currentDriveCommand = new goToLocation(drivebase, pathplanny.getReefLocation(alliance));
                            status.currentDriveAction = "reef";
                        } catch(Exception e)
                        {
                            e.printStackTrace();
                        }
                    }
                }
                break;
            case "processor":
            if ("processor" == status.currentDriveLocation)
            {
                if (currentDriveCommand.isFinished())
                {
                        status.currentDriveAction = "finished";
                        status.currentDriveLocation = "processor";
                }
            } else
            {
                if ("processor" != status.currentDriveAction)
                {
                    try
                    {
                        currentDriveCommand = new goToLocation(drivebase, pathplanny.getProcessorLocation(alliance));
                        status.currentDriveAction = "processor";
                    } catch(Exception e)
                    {
                        e.printStackTrace();
                    }
                }
            }
            break;
        }
        if (status.currentDriveAction == "finished")
        {
            try
            {
            nextAction(false);
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
            //Set the target location of the elevator to source level
            //if the elevator is at location
            //elevator action = "finished"
            //else
            //elevator action = "inProgress"
            if (status.coralIndexGoal == "index")
            {
                //intake coral command
                //when intake coral command has triggered first beam break (ie: coral in robot)
                    //run once
                    //nextAction(true);
                //when intake coral has stops triggering first beam break
                    //nextAction(false)
            }
            case "elevatorLevel1":
            //Set the target location of the elevator to level 1
            //Set the target location of the elevator to source level
            //if the elevator is at location
            //elevator action = "finished"
            //else
            //elevator action = "inProgress"
            if (status.coralIndexGoal == "shoot")
            {
                //shoot the coral command
                //when the coral command is finished
                //set coralIndexAction to finished
            }
            break;
            case "elevatorLevel2":
            //Set the target location of the elevator to level 2
            //Set the target location of the elevator to source level
            //if the elevator is at location
            //elevator action = "finished"
            //else
            //elevator action = "inProgress"
            if (status.coralIndexGoal == "shoot")
            {
                //shoot the coral command
                //when the coral command is finished
                //set coralIndexAction to finished
            }
            break;
            case "elevatorLevel3":
            //Set the target location of the elevator to level 3
            //Set the target location of the elevator to source level
            //if the elevator is at location
            //elevator action = "finished"
            //else
            //elevator action = "inProgress"
            if (status.coralIndexGoal == "shoot")
            {
                //shoot the coral command
                //when the coral command is finished
                //set coralIndexAction to finished
            }
            break;
            case "elevatorLevel4":
            //Set the target location of the elevator to level 4
            //Set the target location of the elevator to source level
            //if the elevator is at location
            //elevator action = "finished"
            //else
            //elevator action = "inProgress"
            if (status.coralIndexGoal == "shoot")
            {
                //shoot the coral command
                //when the coral command is finished
                //set coralIndexAction to finished
            }
            break;
        }
        if (status.coralIndexAction == "finished")
        {
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

    public void nextAction(boolean driving) throws noNextAction, elevatorNotAtTarget
    {
        if (driving)
        {
            if (status.coralIndexAction == "finished")
            {
                if (status.coralIndexGoal == "shoot")
                {
                    status.coralIndexAction = "index";
                    try
                    {
                    currentDriveCommand = new goToLocation(drivebase, pathplanny.getSourceLocation(alliance));
                    } catch(Exception e)
                    {
                        e.printStackTrace();
                    }
                } else if (status.coralIndexGoal == "index")
                {
                    status.coralIndexAction = "hold";
                    try
                    {
                    currentDriveCommand = new goToLocation(drivebase, pathplanny.getReefLocation(alliance));
                    } catch(Exception e)
                    {
                        e.printStackTrace();
                    }
                } else 
                {
                    status.coralIndexAction = "";
                    throw new noNextAction("The coral index action was finished but neither index goal was selected (index and shoot)");
                }
            } else
            {
                throw new noNextAction("the next command was called while the coralIndexAction was not finished");
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
            }
        }
    }

    @Override
    public void periodic() {
        automaticDriving();
        automaticSubsystems();
    }
}
