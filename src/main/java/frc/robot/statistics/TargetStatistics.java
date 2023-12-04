package frc.robot.statistics;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

public class TargetStatistics extends Statistics<PhotonTrackedTarget> {
    private final PoseStatistics poseStats;
    private final CornerStatistics cornerStats;

    public TargetStatistics(String tableName, double bufferSeconds) {
        super(tableName, bufferSeconds);
        poseStats = new PoseStatistics(tableName, bufferSeconds);
        cornerStats = new CornerStatistics(tableName, bufferSeconds);
    }

    @Override
    public void setBufferSeconds(double bufferSeconds) {
        super.setBufferSeconds(bufferSeconds);
        if (poseStats != null) poseStats.setBufferSeconds(bufferSeconds);
        if (cornerStats != null) cornerStats.setBufferSeconds(bufferSeconds);
    }

    @Override
    public void update(PhotonTrackedTarget target, double timestampSeconds) {
        update(new Pose3d().plus(target.getBestCameraToTarget()), target.getDetectedCorners(), timestampSeconds);
    }

    public void update(Pose3d pose, double[] corners, double timestampSeconds) {
        poseStats.update(pose, timestampSeconds);
        cornerStats.update(corners, timestampSeconds);
    }

    public void update(Pose3d pose, List<TargetCorner> corners, double timestampSeconds) {
        poseStats.update(pose, timestampSeconds);
        cornerStats.update(corners, timestampSeconds);
    }

    public Translation3d getTranslationAvg() {return poseStats.getTranslationAvg();}
    public Translation3d getTranslationStdDev() {return poseStats.getTranslationStdDev();}
    public Rotation3d getRotationAvg() {return poseStats.getRotationAvg();}
    public double[] getRotationStdDev() {return poseStats.getRotationStdDev();}
    public List<TargetCorner> getCornerAvgs() {return cornerStats.getCornerAvgs();}
    public List<TargetCorner> getCornerStdDevs() {return cornerStats.getCornerStdDevs();}
    public TargetCorner getTotalCornerStdDev() {return cornerStats.getTotalCornerStdDev();}
    public TargetCorner getTotalCornerSqError() {return cornerStats.getTotalCornerSqError();}
}
