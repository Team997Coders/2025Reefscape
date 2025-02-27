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
                return new Pose2d(1.13,1.05, new Rotation2d(.9425));
            } else if (alliance == Alliance.Red)
            {
                return new Pose2d(16.413,7.01, new Rotation2d(4.084));
            } else 
            {
                throw new allianceNotInitialized("The alliance is neither blue nor red");
            }
        }
        if (sourceId == 12){
            if (alliance == Alliance.Blue)
            {
                return new Pose2d(1.13,7.01, new Rotation2d(5.341));
            } else if (alliance == Alliance.Red)
            {
                return new Pose2d(16.413,1.05, new Rotation2d(2.199));
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
            return new Pose2d(5.988,0.479,new Rotation2d(4.712));
        } else if (alliance == Alliance.Red)
        {
            return new Pose2d(11.561,7.573,new Rotation2d(1.571));
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
                    return new Pose2d(3.175,3.861,new Rotation2d(0));
                    case 1:
                    //side 2
                    return new Pose2d(3.175,3.861,new Rotation2d(0));
                    case 2:
                    //side 3
                    return new Pose2d(5.003,5.246,new Rotation2d(4.189));
                    case 3:
                    //side 4
                    return new Pose2d(5.803,4.191,new Rotation2d(3.142));
                    case 4:
                    //side 5
                    return new Pose2d(5.29,2.971,new Rotation2d(2.094));
                    case 5:
                    //side 6
                    return new Pose2d(3.976,2.971,new Rotation2d(1.047));
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
                    return new Pose2d(3.175,4.191,new Rotation2d(0));
                    case 1:
                    //side 2
                    return new Pose2d(3.976,5.246,new Rotation2d(5.236));
                    case 2:
                    //side 3
                    return new Pose2d(5.29,5.081,new Rotation2d(4.189));
                    case 3:
                    //side 4
                    return new Pose2d(5.803,3.861,new Rotation2d(3.142));
                    case 4:
                    //side 5
                    return new Pose2d(5.003,2.805,new Rotation2d(2.094));
                    case 5:
                    //side 6
                    return new Pose2d(3.69,2.971,new Rotation2d(1.047));
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
                    return new Pose2d(14.373,4.191,new Rotation2d(3.142));
                    case 1:
                    //side 2
                    return new Pose2d(13.86,2.971,new Rotation2d(2.094));
                    case 2:
                    //side 3
                    return new Pose2d(12.546,2.805,new Rotation2d(1.047));
                    case 3:
                    //side 4
                    return new Pose2d(11.745,3.861,new Rotation2d(0));
                    case 4:
                    //side 5
                    return new Pose2d(12.26,5.081,new Rotation2d(5.236));
                    case 5:
                    //side 6
                    return new Pose2d(13.573,5.246,new Rotation2d(4.189));
                    default:
                    throw new noSelectedButton("None of the sides of the reef are selected");
                }
            } else if (leftScorey.pressed())
            {
                switch (selectedSide)
                {
                    case 0:
                    //side 1
                    return new Pose2d(14.373,3.861,new Rotation2d(3.142));
                    case 1:
                    //side 2
                    return new Pose2d(13.573,2.805,new Rotation2d(2.094));
                    case 2:
                    //side 3
                    return new Pose2d(12.26,2.971,new Rotation2d(1.047));
                    case 3:
                    //side 4
                    return new Pose2d(11.745,4.191,new Rotation2d(0));
                    case 4:
                    //side 5
                    return new Pose2d(12.546,5.246,new Rotation2d(5.236));
                    case 5:
                    //side 6
                    return new Pose2d(13.86,5.081,new Rotation2d(4.189));
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
