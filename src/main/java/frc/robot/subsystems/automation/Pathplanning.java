package frc.robot.subsystems.automation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.exceptions.allianceNotInitialized;
import frc.robot.exceptions.noSelectedButton;
import frc.robot.subsystems.buttonBox.ButtonyBit;
import frc.robot.subsystems.buttonBox.ButtonyList;
import frc.robot.subsystems.buttonBox.FlippySwitch;

public class Pathplanning 
{
    private ButtonyList reefy;
    private ButtonyBit rightScorey;
    private ButtonyBit leftScorey;
    private ButtonyBit rightSourcey;
    private ButtonyBit leftSourcey;

    public Pathplanning(ButtonyList reef, ButtonyBit rightScore, ButtonyBit leftScore, ButtonyBit rightSource, ButtonyBit leftSource)
    {
        reefy = reef;
        rightScorey = rightScore;
        leftScorey = leftScore;
        rightSourcey = rightSource;
        leftSourcey = leftSource;
    }    

    public Pose2d getSourceLocation(Alliance alliance, int sourceId) throws allianceNotInitialized, noSelectedButton
    {
        //TODO: Find the positions of all the sources
        if (sourceId == 13)
        {
            if (alliance == Alliance.Blue)
            {
                return new Pose2d(1.083,.974, new Rotation2d(.9425));
            } else if (alliance == Alliance.Red)
            {
                return new Pose2d(16.379,7.078, new Rotation2d(4.084));
            } else 
            {
                throw new allianceNotInitialized("The alliance is neither blue nor red");
            }
        }
        if (sourceId == 12){
            if (alliance == Alliance.Blue)
            {
                return new Pose2d(1.083,7.078, new Rotation2d(5.341));
            } else if (alliance == Alliance.Red)
            {
                return new Pose2d(16.379,0.974, new Rotation2d(2.199));
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
            return new Pose2d(5.988,0.39,new Rotation2d(4.712));
        } else if (alliance == Alliance.Red)
        {
            return new Pose2d(11.56,7.662,new Rotation2d(1.571));
        } else 
        {
            throw new allianceNotInitialized("The alliance is neither blue nor red");
        }
    }
        
    //TODO: NEED TO SET BUTTON NUMBERS TO BE CORRECT, THEY ARE NOT
    public Pose2d getReefLocation(Alliance alliance, int selectedSide, int scoreSide) throws allianceNotInitialized, noSelectedButton
    {
        //TODO: Find the positions of all the reef locations

        if (alliance == Alliance.Blue)
        {
            if (scoreSide == 11)
            {
                switch (selectedSide)
                {
                    case 0:
                    //side 1
                    return new Pose2d(3.264,3.861,new Rotation2d(0));
                    case 1:
                    //side 2
                    return new Pose2d(3.734,5.004,new Rotation2d(5.236));
                    case 2:
                    //side 3
                    return new Pose2d(4.851,5.169,new Rotation2d(4.189));
                    case 3:
                    //side 4
                    return new Pose2d(5.714,4.191,new Rotation2d(3.142));
                    case 4:
                    //side 5
                    return new Pose2d(5.245,3.048,new Rotation2d(2.094));
                    case 5:
                    //side 6
                    return new Pose2d(4.128,2.883,new Rotation2d(1.047));
                    default:
                    throw new noSelectedButton("None of the sides of the reef are selected");
                }
            }
            if (scoreSide == 10)
            {
                switch (selectedSide)
                {
                    case 0:
                    //side 1
                    return new Pose2d(3.264,4.191,new Rotation2d(0));
                    case 1:
                    //side 2
                    return new Pose2d(4.128,5.169,new Rotation2d(5.236));
                    case 2:
                    //side 3
                    return new Pose2d(5.245,5.004,new Rotation2d(4.189));
                    case 3:
                    //side 4
                    return new Pose2d(5.714,3.861,new Rotation2d(3.142));
                    case 4:
                    //side 5
                    return new Pose2d(4.851,2.883,new Rotation2d(2.094));
                    case 5:
                    //side 6
                    return new Pose2d(3.734,3.048,new Rotation2d(1.047));
                    default:
                    throw new noSelectedButton("None of the sides of the reef are selected");
                }
            } else
            {
                throw new noSelectedButton("Right or left score side has not been selected");
            }
        } else if (alliance == Alliance.Red)   
        {
            if (rightScorey.pressed())
            {
                switch (selectedSide)
                {
                    case 0:
                    //side 1
                    return new Pose2d(14.284,4.191,new Rotation2d(3.142));
                    case 1:
                    //side 2
                    return new Pose2d(13.815,3.048,new Rotation2d(2.094));
                    case 2:
                    //side 3
                    return new Pose2d(12.698,2.883,new Rotation2d(1.047));
                    case 3:
                    //side 4
                    return new Pose2d(11.834,3.861,new Rotation2d(0));
                    case 4:
                    //side 5
                    return new Pose2d(12.304,5.004,new Rotation2d(5.236));
                    case 5:
                    //side 6
                    return new Pose2d(13.421,5.169,new Rotation2d(4.189));
                    default:
                    throw new noSelectedButton("None of the sides of the reef are selected");
                }
            } else if (leftScorey.pressed())
            {
                switch (selectedSide)
                {
                    case 0:
                    //side 1
                    return new Pose2d(14.284,3.861,new Rotation2d(3.142));
                    case 1:
                    //side 2
                    return new Pose2d(13.442,2.883,new Rotation2d(2.094));
                    case 2:
                    //side 3
                    return new Pose2d(12.304,3.048,new Rotation2d(1.047));
                    case 3:
                    //side 4
                    return new Pose2d(11.834,4.191,new Rotation2d(0));
                    case 4:
                    //side 5
                    return new Pose2d(12.698,5.169,new Rotation2d(5.236));
                    case 5:
                    //side 6
                    return new Pose2d(13.815,5.004,new Rotation2d(4.189));
                    default:
                    throw new noSelectedButton("None of the sides of the reef are selected");
                }
            }else
            {
                throw new noSelectedButton("Right or left score side has not been selected");
            }
        } else 
        {
            throw new allianceNotInitialized("The alliance is neither blue nor red");
        }
    }
}
