// package frc.robot.subsystems.automation;

// import edu.wpi.first.wpilibj.DriverStation.Alliance;

// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// import frc.robot.commands.CoralIntake;
// import frc.robot.commands.CoralManualControl;
// import frc.robot.commands.CoralOutTake;
// import frc.robot.commands.ElevatorAutomaticControl;
// import frc.robot.commands.ElevatorManualControl;
// import frc.robot.commands.goToLocation;
// import frc.robot.exceptions.elevatorNotAtTarget;
// import frc.robot.exceptions.noNextAction;
// import frc.robot.subsystems.Coral;
// import frc.robot.subsystems.Drivebase;
// import frc.robot.subsystems.Elevator;
// import frc.robot.subsystems.Elevator.ElevatorState;
// import frc.robot.subsystems.buttonBox.ButtonBox;
// import frc.robot.subsystems.buttonBox.buttonCommands.goPressed;

// public class AutomaticSystems extends SubsystemBase
// {
//     private Pathplanning pathplanning;
//     private ButtonBox buttonBox;
//     private Status status;
//     private Alliance alliance;

//     private Drivebase drivebase;
//     private Coral coral;
//     private Elevator elevator;
    
//     private boolean beamBreakTriggered;
//     public boolean goClick;
    
//     private XboxController driveController;

//     private Command coralShootCommand;
//     private Command coralIntakeCommand;
//     private Command goPressedCommand;
//     private Command currentDriveCommand;
//     private Command manualDriveCommand;
//     private Command manualElevatorCommand;
//     private Command semiAutomaticElevatorCommand;
//     private Command manualCoralCommand;

//     public AutomaticSystems(XboxController buttonBox, Drivebase drivebase, Command driveCommand, Coral coral, Elevator elevator, XboxController driveController, CommandXboxController c_driveyController)
//     {
//         this.buttonBox = new ButtonBox(buttonBox);
//         this.pathplanning = new Pathplanning(this.buttonBox.reefSide, this.buttonBox.rightScore, this.buttonBox.leftScore, this.buttonBox.rightSource, this.buttonBox.leftSource);
//         this.status = new Status();
//         this.alliance = alliance.Red;

//         this.drivebase = drivebase;
//         this.coral = coral;
//         this.elevator = elevator;

//         this.goClick = false;
//         this.beamBreakTriggered = false;
//         this.driveController = driveController;

//         this.coralIntakeCommand = new CoralIntake(coral);
//         this.coralShootCommand = new CoralOutTake(coral);
//         this.goPressedCommand = new goPressed(this);
//         this.manualDriveCommand = driveCommand;
//         this.currentDriveCommand = null;
//         this.buttonBox.go.onTrue(this.goPressedCommand);
//       //  this.manualElevatorCommand = new ElevatorManualControl(elevator, () -> c_driveyController.povUp().getAsBoolean(), () -> c_driveyController.povDown().getAsBoolean());
//         this.semiAutomaticElevatorCommand = new ElevatorAutomaticControl(elevator, () -> c_driveyController.povUp().getAsBoolean(), () -> c_driveyController.povDown().getAsBoolean());
//         this.manualCoralCommand = new CoralManualControl(coral, driveController);
//     }

//     public void automaticDriving()
//     {
//         /*
//         This will be called periodically while the automaticDriving flippySwitch is on
//         It checks the driveGoal: 
//             First time            -> Schedule a new path on the fly to target location
//             Every subsequent call -> Checks if the path is finished
//                 When the path is finished -> calls next action (for subsystems) -> generally shoots or intakes coral
//          */
//         switch (status.driveGoal) {
//             case "source":
//                 if (currentDriveCommand.isFinished() && status.currentDriveAction == "source")
//                 {
//                     //This happens when the source path drive action is finished
//                     status.currentDriveAction = "finished";
//                     status.currentDriveLocation = "source";
//                 } else if ("source" != status.currentDriveAction)
//                 {
//                     //This happens the first run through where the source is the goal but not the action
//                     try
//                     {
//                         //Makes new goTo command for the target location based off of button box inputs
//                         currentDriveCommand = new goToLocation(drivebase, pathplanning.getSourceLocation(alliance));
//                         currentDriveCommand.schedule();
//                         status.currentDriveAction = "source";
//                         status.currentDriveLocation = "moving";
//                     } catch(Exception e)
//                     {
//                         //This will fail if not all the buttons are selected (such as no side of reef has been pressed)
//                         //Here there is going to be the buzzing button box to let the drivers know the new location was not set
//                         //So it will try again next periodic until all buttons are selected
//                         e.printStackTrace();
//                     }
//                 }
//                 break;
//             case "reef":
//             //Next two Identical to the source drive command except with the reef and processor
//             if (currentDriveCommand.isFinished() && status.currentDriveAction == "reef")
//             {
//                 status.currentDriveAction = "finished";
//                 status.currentDriveLocation = "reef";
//             } else if ("source" != status.currentDriveAction)
//             {
//                 try
//                 {
//                     currentDriveCommand = new goToLocation(drivebase, pathplanning.getReefLocation(alliance));
//                     currentDriveCommand.schedule();
//                     status.currentDriveAction = "reef";
//                     status.currentDriveLocation = "moving";
//                 } catch(Exception e)
//                 {
//                     e.printStackTrace();
//                 }
//             }
//             break;
//             case "processor":
//             if (currentDriveCommand.isFinished() && status.currentDriveAction == "processor")
//             {
//                 status.currentDriveAction = "finished";
//                 status.currentDriveLocation = "processor";
//             } else if ("source" != status.currentDriveAction)
//             {
//                 try
//                 {
//                     currentDriveCommand = new goToLocation(drivebase, pathplanning.getProcessorLocation(alliance));
//                     currentDriveCommand.schedule();
//                     status.currentDriveAction = "processor";
//                     status.currentDriveLocation = "moving";
//                 } catch(Exception e)
//                 {
//                     e.printStackTrace();
//                 }
//             }
//             break;
//         }
//         //This is called when the drive action finishes
//         if (status.currentDriveAction == "finished")
//         {
//             try
//             {
//             //This will try to call either shoot or intake of coral depending on the location
//             nextAction(false);
//             } catch(elevatorNotAtTarget e)
//             {
//                 //This will fail if the elevator is not yet at the target (the drivebase reaches the target before the elevator)
//                 //So callNextAction will try to call nextAction every periodic until the elevator reaches the target location
//                 status.callNextAction = true;
//             } catch(Exception e)
//             {
//                 //Otherwise the no next action was called which is really bad and the robot will have so be switched into manual or semi-Auto
//                 //Printing the stacktrace so we can see after the match what caused the error and hopefully fix it
//                 e.printStackTrace();
//             }
//         }
//     }

//     public void automaticSubsystems()
//     {
//         /*
//         This will be called periodically while the automaticSubsystem flippySwitch is on
//         It checks the elevatorGoal: 
//             Tells the elevator to go the selected state (Source-L4)
//             When the elevator is at the target it sets the elevatorAction to finished
//                 If coral indexing -> it will schedule the index command
//                     Once the coral touches the first beam break the robot will start driving
//                     Once the coral stops touching the first beam break it will hold and move the elevator
//                 If coral shooting -> it will schedule shoot command
//                     Once the coral exits the robot it will start driving and move the elevator back down to the source level
//          */
//         switch (status.elevatorGoal) {
//             case "elevatorSource":
//             elevator.goToStateCommand(ElevatorState.SOURCE);
//             try
//             {
//                 if (elevator.elevatorAtTarget())
//                 {
//                     status.elevatorAction = "finished";
//                 } else
//                 {
//                     status.elevatorAction = "inProgress";
//                 }
//             } catch(Exception e)
//             {
//                 //This will only happen if the distance for the elevator to be finished is 0
//                 //This should be tuned; once it is this will never be called
//                 e.printStackTrace();
//             }
//             if (status.coralIndexGoal == "index")
//             {
//                 if (status.coralIndexAction == "hold")
//                 {
//                     //This will happen only the first time the goal switches to index
//                     coralIntakeCommand.schedule();
//                     status.coralIndexAction = "index";
//                 }
//                 if (coral.BeamBrake1() && !beamBreakTriggered)
//                 {
//                     //This will run once; when the coral triggers the first beam break
//                     beamBreakTriggered = true;
//                     try
//                     {
//                     //This will start the robot driving to the reef
//                     nextAction(true);
//                     } catch(Exception e)
//                     {
//                         //If it doesnt work something has gone seriously wrong and driving will have to go manual
//                         e.printStackTrace();
//                     }
//                 }
//                 if (!coral.BeamBrake1() && beamBreakTriggered)
//                 {
//                     //ran once when intake coral has stops triggering first beam break
//                     beamBreakTriggered = false;
//                     status.coralIndexAction = "finished";
//                     coralIntakeCommand.cancel();
//                     try
//                     {
//                     //This should now have the coral secured in the elevator and past the point where it could hit the top bar
//                     //This will then send the elevator to the target level set on the button box
//                     nextAction(false);
//                     } catch(Exception e)
//                     {
//                     //No next action was called which is really bad and the robot will have so be switched into manual or semi-Auto
//                     e.printStackTrace();
//                     }
//                 }
//             }
//             break;
//             case "elevatorLevel1":
//             elevator.goToStateCommand(ElevatorState.L1);
//             checkSubsystemStatus();
//             break; 
//             case "elevatorLevel2":
//             elevator.goToStateCommand(ElevatorState.L2);
//             checkSubsystemStatus();
//             break; 
//             case "elevatorLevel3":
//             elevator.goToStateCommand(ElevatorState.L3);
//             checkSubsystemStatus();
//             break; 
//             case "elevatorLevel4":
//             elevator.goToStateCommand(ElevatorState.L4);
//             checkSubsystemStatus();
//             break;            
//         }
//     }

//     public void checkSubsystemStatus()
//     {
//         //
//         try
//         {
//         if (elevator.elevatorAtTarget())
//         {
//             status.elevatorAction = "finished";
//         } else
//         {
//             status.elevatorAction = "inProgress";
//         }
//         } catch(Exception e)
//         {
//             //This will only happen if the distance for the elevator to be finished is 0
//             //This should be tuned; once it is this will never be called
//             e.printStackTrace();
//         }
//         if (status.coralIndexGoal == "shoot")
//         {
//             //This will happen only the first time the goal switches to shoot
//             if (status.coralIndexAction == "hold")
//             {
//                 coralShootCommand.schedule();
//                 status.coralIndexAction = "shoot";
//             }
//             if (coralShootCommand.isFinished() && status.coralIndexAction == "shoot")
//             {
//                 //run once when Command finishes when the second beam break stops being triggered
//                 status.coralIndexAction = "finished";
//                 try
//                 {
//                 //Will start the robot driving to the source selected on button box
//                 nextAction(true);
//                 } catch(Exception e)
//                 {
//                     //If this doesn't work we switch to manual driving because something has gone horribly wrong
//                     e.printStackTrace();
//                 }
//             }
//         }
//     }

//     public void nextAction(boolean driving) throws noNextAction, elevatorNotAtTarget
//     {
//         //This will be called every time we need to change the current action of either elevator, coral indexer, or drivebase
//         if (driving)
//         {
//             //Have driving be true If the action makes the robot drive (otherwise will involve subsystems)
//             if (status.coralIndexAction == "finished")
//             {
//                 if (status.coralIndexGoal == "shoot")
//                 {
//                     //Finished shoot command means start index command 
//                     status.coralIndexGoal = "index";
//                     status.driveGoal = "source";
//                 } else if (status.coralIndexGoal == "index")
//                 {
//                     //Finished index command means hold coral in robot
//                     status.coralIndexGoal = "hold";
//                     status.driveGoal = "reef";
//                 } else 
//                 {
//                     //Something has gone horribly wrong
//                     status.coralIndexAction = "";
//                     throw new noNextAction("The coral index action was finished but neither index goal was selected (index and shoot)");
//                 }
//             } else
//             {
//                 //If the coral index action is not finished we should not be driving anywhere
//                 //throw new noNextAction("the next command was called while the coralIndexAction was not finished");
//             }
//         } else
//         {
//             if (status.currentDriveLocation == "reef")
//             {
//                 //The robot is at the reef
//                 if (status.elevatorAction == "finished")
//                 {
//                     //Shoot the coral onto the reef
//                     status.coralIndexGoal = "shoot";
//                     //Stop the repeated trying to shoot because the elevator is now at target
//                     status.callNextAction = false;
//                 } else
//                 {
//                     //If the elevator is not at the target (or within range to still work) 
//                     //It will then try again until the elevator gets to the target
//                     throw new elevatorNotAtTarget("next command called while elevator not at target");
//                 }
//             } else if (status.currentDriveLocation == "source")
//             {
//                 //robot is at the source
//                 if (status.elevatorAction == "finished")
//                 {
//                     //Intake the coral (this wont do anything until the first beam break is triggered)
//                     status.coralIndexGoal = "intake";
//                     //Stop the repeated trying to shoot because the elevator is now at target
//                     status.callNextAction = false;
//                 } else
//                 {
//                     //If the elevator is not at the target (or within range to still work) 
//                     //It will then try again until the elevator gets to the target
//                     throw new elevatorNotAtTarget("next command called while elevator not at target");
//                 }
//             } else 
//             {
//                 //If the robot is not at the reef or the source it wont do anything automatically
//                 //throw new noNextAction("current drive location not reef or source");
//             }
//         }
//     }

//     @Override
//     public void periodic() {
//         //This is called every 20ms
//         if (!buttonBox.autoDrive.Flipped())
//         {
//             //The robot will be driving through joystick (no automation (sad))
//             manualDriveCommand.schedule();
//             if (currentDriveCommand != null)
//             {   
//                 currentDriveCommand.cancel();
//             } 
//         } else
//         {
//             //The robot will be driving automatically through pathplanner
//             manualDriveCommand.cancel();
//             automaticDriving();
//         }

//         if (!buttonBox.fullAutoElevator.Flipped())
//         {
//             //The subsystems will not be fully automatic
//             //The coral will be full manual: A/B/X control: intake/shoot/reverse
//             manualCoralCommand.schedule();

//             if (buttonBox.semiAutoElevator.Flipped())
//             {
//                 //The elevator will be able to just go to specific levels with dpad up/down
//                 manualElevatorCommand.cancel();
//                 semiAutomaticElevatorCommand.schedule();
//                 elevator.pidControl();
//             } else
//             {
//                 //The elevator will just spin the motor; very basic but can be used for minor adjustments
//                 manualElevatorCommand.schedule();
//                 semiAutomaticElevatorCommand.cancel();
//             }
//         } else if (buttonBox.fullAutoCycles.Flipped())
//         {
//             //Elevator and coral indexer are entirely automatic and require no human control whatsoever
//             manualCoralCommand.cancel();
//             manualElevatorCommand.cancel();
//             semiAutomaticElevatorCommand.cancel();
//             automaticSubsystems();
//             elevator.pidControl();

//             if (status.callNextAction)
//             {
//                 try
//                 {
//                 //This will happen if robot is waiting on elevator to get to the correct level
//                 nextAction(false);
//                 } catch(elevatorNotAtTarget e)
//                 {
//                 //Elevator not yet at level so try again next loop
//                 status.callNextAction = true;
//                 } catch(Exception e)
//                 {
//                 //something else has gone horribly wrong and you should probably switch to manual
//                 e.printStackTrace();
//                 }
//             }        
//         } else
//         {
//             //Semi automatic commands (will only run the commands on clicking go)
//             //Example: once the robot picks up coral you click go to make it go to reef and move elevator 
//             //  Once it reaches the reef it will wait for you to click go again to shoot coral
//             if (status.callNextAction)
//             {
//                 try
//                 {
//                 //This will happen if robot is waiting on elevator to get to the correct level
//                 nextAction(false);
//                 } catch(elevatorNotAtTarget e)
//                 {
//                 //Elevator not yet at level so try again next loop
//                 status.callNextAction = true;
//                 } catch(Exception e)
//                 {
//                 //something else has gone horribly wrong and you should probably switch to manual
//                 e.printStackTrace();
//                 }
//             }     
            
//             if (goClick)
//             {
//                 goClick = false;
//                 try{
//                 //Calls both the drive and subsystem command to check if they have available actions
//                 nextAction(true);
//                 nextAction(false);
//                 } catch(elevatorNotAtTarget e)
//                 {
//                 status.callNextAction = true;
//                 } catch(Exception e)
//                 {
//                 e.printStackTrace();
//                 }
//             }
//         }
//     }
// }
