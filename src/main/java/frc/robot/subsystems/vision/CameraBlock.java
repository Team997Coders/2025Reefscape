package frc.robot.subsystems.vision;

import java.util.List;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CameraBlock 
{
    List<Camera> cameraList;
    public int TargetId = -1;
    public double goodness = -1;

    public CameraBlock(List<Camera> cameraList)
    {
        this.cameraList = cameraList;
    }

    public void update(SwerveDrivePoseEstimator poseEstimator)
    {
        for (Camera camera: this.cameraList){
            List<PhotonPipelineResult> result = camera.getResults();
            if (!result.isEmpty()) {
                if (result.get(0).getBestTarget() != null)
                {
                    PhotonTrackedTarget result0 = result.get(0).getBestTarget();

                    if (TargetId != result0.getFiducialId())
                    {
                        goodness = -1;
                        TargetId = -1;
                    }

                    if (goodness == -1 && TargetId != -1) {
                        TargetId = result0.getFiducialId();
                        goodness = result0.getPoseAmbiguity();
                    } else if (result0.getPoseAmbiguity() < goodness) {
                        TargetId = result0.getFiducialId();
                        goodness = result0.getPoseAmbiguity();
                    } 
                }
            }

            SmartDashboard.putNumber("Best Target ID", TargetId);

            {
                camera.update(poseEstimator, result);
            }
        }
    }
}

