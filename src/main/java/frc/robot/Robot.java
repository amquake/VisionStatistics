// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.util.HashMap;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.statistics.CornerStatistics;
import frc.robot.statistics.TargetStatistics;

public class Robot extends TimedRobot {
    //#################################
    // Leave names blank "" to disable
    String photonCameraName = "camera";
    String limelightCameraName = ""; // e.g. "limelight"
    String customTagLayoutPath = ""; // e.g. "layout.json"
    private final double kBufferLengthSeconds = 20;
    //#################################

    PhotonCamera photonCamera;
    PhotonPoseEstimator photonEstimator;

    AprilTagFieldLayout tagLayout;

    HashMap<Integer, TargetStatistics> photonTargetStatsMap;
    TargetStatistics photonMultitagStats;
    TargetStatistics limelightMegatagStats;

    NetworkTable photonCameraTable;
    DoubleArrayPublisher photonEstPoseArrayPublisher;

    {
        if (customTagLayoutPath != "") {
            try {
                tagLayout = new AprilTagFieldLayout(customTagLayoutPath);
            }
            catch (IOException e) {
                e.printStackTrace();
            }
        }
        else {
            tagLayout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
        }

        if (photonCameraName != "") {
            photonCamera = new PhotonCamera(photonCameraName);
            photonEstimator = new PhotonPoseEstimator(tagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, photonCamera, new Transform3d());
            photonTargetStatsMap = new HashMap<Integer, TargetStatistics>();
            photonMultitagStats = new TargetStatistics("Photon Multitag Stats", kBufferLengthSeconds);

            photonCameraTable = NetworkTableInstance.getDefault().getTable("photonvision").getSubTable(photonCameraName);
            photonEstPoseArrayPublisher = photonCameraTable.getDoubleArrayTopic("EstPoseArray").publish();
        }

        if (limelightCameraName != "") {
            limelightMegatagStats = new TargetStatistics("Limelight Megatag Stats", kBufferLengthSeconds);
        }
    }

    @Override
    public void robotInit() {
        // DataLogManager.start();

        SmartDashboard.putData("Start Log", Commands.runOnce(()->DataLogManager.start()));
        SmartDashboard.putData("Stop Log", Commands.runOnce(()->DataLogManager.stop()));
    }
    
    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

        if (photonCameraName != "") {
            updatePhotonStats();
        }

        if (limelightCameraName != "") {
            updateLimelightStats();
        }
    }
        
    @Override
    public void teleopInit() {}
    
    @Override
    public void teleopPeriodic() {}
    
    @Override
    public void simulationInit() {}
    
    @Override
    public void simulationPeriodic() {}



    private void updatePhotonStats() {
        var result = photonCamera.getLatestResult();
        double timestamp = result.getTimestampSeconds();

        // single-tag stats
        for (var target : result.getTargets()) {
            String name = "Photon Target("+target.getFiducialId()+") Stats";
            if (!photonTargetStatsMap.containsKey(target.getFiducialId())) {
                photonTargetStatsMap.put(target.getFiducialId(), new TargetStatistics(name, kBufferLengthSeconds));
            }
            var stats = photonTargetStatsMap.get(target.getFiducialId());
            stats.update(target, timestamp);
        }

        // estimated pose stats (needs tag layout!)
        var estimation = photonEstimator.update(result);
        if (estimation.isEmpty()) return;
        var estimatedPose = estimation.get().estimatedPose;
        photonMultitagStats.update(estimatedPose, CornerStatistics.allTargetCorners(estimation.get().targetsUsed), timestamp);

        double[] estPoseArray = {
            estimatedPose.getX(),
            estimatedPose.getY(),
            estimatedPose.getZ(),
            Math.toDegrees(estimatedPose.getRotation().getX()),
            Math.toDegrees(estimatedPose.getRotation().getY()),
            Math.toDegrees(estimatedPose.getRotation().getZ())
        };
        photonEstPoseArrayPublisher.set(estPoseArray);
    }

    private void updateLimelightStats() {
        var result = LimelightHelpers.getLatestResults(limelightCameraName);
        var pose = result.targetingResults.getBotPose3d_wpiBlue();
        double[] tcornxy = LimelightHelpers.getLimelightNTDoubleArray(limelightCameraName, "tcornxy");
        double timestamp = result.targetingResults.timestamp_LIMELIGHT_publish;

        limelightMegatagStats.update(pose, tcornxy, timestamp);
    }
}
