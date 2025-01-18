package frc.robot.subsystems.automation;

public class Status 
{
    public String coralIndexAction;
    public String coralIndexGoal;
    public String elevatorAction;
    public String elevatorGoal;
    public String driveGoal;
    public String currentDriveLocation;
    public String currentDriveAction;
    public boolean callNextAction;

    public Status()
    {
        driveGoal = "";
        currentDriveAction = "";
        currentDriveLocation = "";
        elevatorAction = "";
        elevatorGoal = "";
        coralIndexAction = "";
        coralIndexGoal = "";
        currentDriveAction = "";
        callNextAction = false;
    }    
}
