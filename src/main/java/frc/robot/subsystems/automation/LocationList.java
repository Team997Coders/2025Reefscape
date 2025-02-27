package frc.robot.subsystems.automation;

import java.util.Arrays;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LocationList {
    private List<Pose2d> allLocations = Arrays.asList(
        new Pose2d(1.13,1.05, new Rotation2d(.9425)),
        new Pose2d(16.413,7.01, new Rotation2d(4.084)),
        new Pose2d(1.13,7.01, new Rotation2d(5.341)),
        new Pose2d(16.413,1.05, new Rotation2d(2.199)),
        new Pose2d(5.988,0.479,new Rotation2d(4.712)),
        new Pose2d(11.561,7.573,new Rotation2d(1.571)),
        new Pose2d(3.175,3.861,new Rotation2d(0)),
        new Pose2d(3.175,3.861,new Rotation2d(0)),
        new Pose2d(5.003,5.246,new Rotation2d(4.189)),
        new Pose2d(5.803,4.191,new Rotation2d(3.142)),
        new Pose2d(5.29,2.971,new Rotation2d(2.094)),
        new Pose2d(3.976,2.971,new Rotation2d(1.047)),
        new Pose2d(3.175,4.191,new Rotation2d(0)),
        new Pose2d(3.976,5.246,new Rotation2d(5.236)),
        new Pose2d(5.29,5.081,new Rotation2d(4.189)),
        new Pose2d(5.803,3.861,new Rotation2d(3.142)),
        new Pose2d(5.003,2.805,new Rotation2d(2.094)),
        new Pose2d(3.69,2.971,new Rotation2d(1.047)),
        new Pose2d(14.373,4.191,new Rotation2d(3.142)),
        new Pose2d(13.86,2.971,new Rotation2d(2.094)),
        new Pose2d(12.546,2.805,new Rotation2d(1.047)),
        new Pose2d(11.745,3.861,new Rotation2d(0)),
        new Pose2d(12.26,5.081,new Rotation2d(5.236)),
        new Pose2d(13.573,5.246,new Rotation2d(4.189)),
        new Pose2d(14.373,3.861,new Rotation2d(3.142)),
        new Pose2d(13.573,2.805,new Rotation2d(2.094)),
        new Pose2d(12.26,2.971,new Rotation2d(1.047)),
        new Pose2d(11.745,4.191,new Rotation2d(0)),
        new Pose2d(12.546,5.246,new Rotation2d(5.236)),
        new Pose2d(13.86,5.081,new Rotation2d(4.189))
        );
    
    private int index;
    
    public LocationList()
    {
        this.index = -1;
    }

    public Pose2d getNextLocation()
    {
        this.index++;
        SmartDashboard.putNumber("position index", this.index);
        return allLocations.get(this.index);
    }
}
