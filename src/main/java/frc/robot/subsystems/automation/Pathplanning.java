package frc.robot.subsystems.automation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.exceptions.allianceNotInitialized;
import frc.robot.exceptions.noSelectedButton;
import frc.robot.subsystems.buttonBox.ButtonyBit;
import frc.robot.subsystems.buttonBox.ButtonyList;
import frc.robot.subsystems.buttonBox.FlippySwitch;

public class Pathplanning 
{
    private ButtonyList reefy;
    private FlippySwitch scoreySide;
    private ButtonyBit rightSourcey;
    private ButtonyBit leftSourcey;

    public Pathplanning(ButtonyList reef, FlippySwitch scoreSide, ButtonyBit rightSource, ButtonyBit leftSource)
    {
        reefy = reef;
        scoreySide = scoreSide;
        rightSourcey = rightSource;
        leftSourcey = leftSource;
    }    

    //TODO: NEED TO SET BUTTON NUMBERS TO BE CORRECT, THEY ARE NOT
    public Pose2d getSourceLocation(Alliance alliance) throws allianceNotInitialized, noSelectedButton
    {
        //TODO: Find the positions of all the sources
        if (rightSourcey.pressed())
        {
            if (alliance == Alliance.Blue)
            {
                return new Pose2d();
            } else if (alliance == Alliance.Red)
            {
                return new Pose2d();
            } else 
            {
                throw new allianceNotInitialized("The alliance is neither blue nor red");
            }
        }
        if (leftSourcey.pressed())
        {
            if (alliance == Alliance.Blue)
            {
                return new Pose2d();
            } else if (alliance == Alliance.Red)
            {
                return new Pose2d();
            } else 
            {
                throw new allianceNotInitialized("The alliance is neither blue nor red");
            }
        }
        throw new noSelectedButton("None of the source buttons are selected");
    }

    public Pose2d getProcessorLocation(Alliance alliance) throws allianceNotInitialized
    {
        //TODO: Find the positions of all the processors
    
        if (alliance == Alliance.Blue)
        {
            return new Pose2d();
        } else if (alliance == Alliance.Red)
        {
            return new Pose2d();
        } else 
        {
            throw new allianceNotInitialized("The alliance is neither blue nor red");
        }
    }
        
    //TODO: NEED TO SET BUTTON NUMBERS TO BE CORRECT, THEY ARE NOT
    public Pose2d getReefLocation(Alliance alliance) throws allianceNotInitialized, noSelectedButton
    {
        //TODO: Find the positions of all the reef locations
        int selectedSide = 0;
        try
        {
        selectedSide = reefy.selectedBit().id;
        } catch(noSelectedButton e)
        {
            e.printStackTrace();
        }

        if (alliance == Alliance.Blue)
        {
            if (scoreySide.Flipped())
            {
                switch (selectedSide)
                {
                    case 7:
                    //side 1
                    return new Pose2d();
                    case 8:
                    //side 2
                    return new Pose2d();
                    case 9:
                    //side 3
                    return new Pose2d();
                    case 10:
                    //side 4
                    return new Pose2d();
                    case 11:
                    //side 5
                    return new Pose2d();
                    case 12:
                    //side 6
                    return new Pose2d();
                    default:
                    throw new noSelectedButton("None of the sides of the reef are selected");
                }
            } else
            {
                switch (selectedSide)
                {
                    case 7:
                    //side 1
                    return new Pose2d();
                    case 8:
                    //side 2
                    return new Pose2d();
                    case 9:
                    //side 3
                    return new Pose2d();
                    case 10:
                    //side 4
                    return new Pose2d();
                    case 11:
                    //side 5
                    return new Pose2d();
                    case 12:
                    //side 6
                    return new Pose2d();
                    default:
                    throw new noSelectedButton("None of the sides of the reef are selected");
                }
            }
        } else if (alliance == Alliance.Red)   
        {
            if (scoreySide.Flipped())
            {
                switch (selectedSide)
                {
                    case 7:
                    //side 1
                    return new Pose2d();
                    case 8:
                    //side 2
                    return new Pose2d();
                    case 9:
                    //side 3
                    return new Pose2d();
                    case 10:
                    //side 4
                    return new Pose2d();
                    case 11:
                    //side 5
                    return new Pose2d();
                    case 12:
                    //side 6
                    return new Pose2d();
                    default:
                    throw new noSelectedButton("None of the sides of the reef are selected");
                }
            } else
            {
                switch (selectedSide)
                {
                    case 7:
                    //side 1
                    return new Pose2d();
                    case 8:
                    //side 2
                    return new Pose2d();
                    case 9:
                    //side 3
                    return new Pose2d();
                    case 10:
                    //side 4
                    return new Pose2d();
                    case 11:
                    //side 5
                    return new Pose2d();
                    case 12:
                    //side 6
                    return new Pose2d();
                    default:
                    throw new noSelectedButton("None of the sides of the reef are selected");
                }
            }
        } else 
        {
            throw new allianceNotInitialized("The alliance is neither blue nor red");
        }
    }
}
