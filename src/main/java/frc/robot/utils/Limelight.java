// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.util.Optional;
import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/** Add your docs here. */
public class Limelight extends VisionSystem {
    int targetPipeline = 0;

    private final double TELEOP_MT2_ROTATION_THRESHOLD = 0.5;
    private final double AUTON_MT2_ROTATION_THRESHOLD = 0.1;

    private final double TELEOP_MT2_DISTANCE_THRESHOLD = 1.0;
    private final double AUTON_MT2_DISTANCE_THRESHOLD = 0.25;

    private double angularVelocityThreshold = TELEOP_MT2_ROTATION_THRESHOLD; // how many rad/sec to update MT2 pose
    private double poseDifferenceThreshold = TELEOP_MT2_DISTANCE_THRESHOLD;

    public Limelight(String cameraName, Transform3d robotToCamera) {
        super(cameraName, robotToCamera);
    }

    public boolean getHasTarget() {
        return LimelightHelpers.getTV(cameraName);
    }

    public Optional<Rotation2d> getTargetX() {
        if (LimelightHelpers.getTV(cameraName))
            return Optional.of(Rotation2d.fromDegrees(-LimelightHelpers.getLimelightNTDouble(cameraName, "txnc")));

        return Optional.empty();
    }

    public Optional<Rotation2d> getTagYaw(int tagID) {
        return getTargetX();
    }

    public Optional<Double> getTagDistance(int tagID) {
        var dist = super.getTagDistance(tagID);
        var tx = getTargetX();
        if (dist.isEmpty() || tx.isEmpty())
            return dist;

        // var compensatedDistance = dist.get() / Math.cos(-tx.get().getRadians());

        // System.out.println("tx: " + tx.get());
        // System.out.println("Compensated distance: " + compensatedDistance);
        // System.out.println("======================================================================");
        return Optional.of(/* compensatedDistance */dist.get() / Math.cos(-tx.get().getRadians()));
    }

    public Optional<Rotation2d> getTargetY() {
        if (LimelightHelpers.getTV(cameraName))
            return Optional.of(Rotation2d.fromDegrees(LimelightHelpers.getLimelightNTDouble(cameraName, "tync")));

        return Optional.empty();
    }

    public Optional<Rotation2d> getTagPitch(int tagID) {
        return getTargetY();
    }

    public int getTV() {
        return LimelightHelpers.getTV(cameraName) ? 1 : 0;
    }

    public void setRobotRotationMT2(double degrees) {
        LimelightHelpers.SetRobotOrientation(cameraName, degrees, 0, 0, 0, 0, 0);
    }

    public LimelightHelpers.PoseEstimate getBotposeEstimateMT2() {
        return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(cameraName);
    }

    public Optional<Double[]> getSTDevsXY(CommandSwerveDrivetrain drivetrain) {
        var visionResults = getBotposeEstimateMT2();
        boolean validPose = drivetrain.getTranslation().getDistance(visionResults.pose.getTranslation()) <= poseDifferenceThreshold;

        if (Math.abs(drivetrain.getState().speeds.omegaRadiansPerSecond) > angularVelocityThreshold
                || Math.sqrt(Math.pow(drivetrain.getState().speeds.vxMetersPerSecond, 2)
                        + Math.pow(drivetrain.getState().speeds.vyMetersPerSecond, 2)) > 5){
            return Optional.empty();

        } else if (visionResults.tagCount >= 2) {
            return Optional.of(new Double[] { 0.1, 0.1 });
        } else if (visionResults.tagCount == 1 && visionResults.avgTagArea >= 0.4
                && validPose) {
            return Optional.of(new Double[] { 1.0, 1.0 });
        } else {
            return Optional.empty();
        }
    }

    public void setPipeline(int pipeline) {
        LimelightHelpers.setPipelineIndex(cameraName, pipeline);
        targetPipeline = pipeline;
    }

    public BooleanSupplier getPipelineReady() {
        return () -> targetPipeline == getPipeline();
    }

    public int getPipeline() {
        return (int) LimelightHelpers.getCurrentPipelineIndex(cameraName);
    }

    public void setTeleopMT2Threshold() {
        angularVelocityThreshold = TELEOP_MT2_ROTATION_THRESHOLD;
        poseDifferenceThreshold = TELEOP_MT2_DISTANCE_THRESHOLD;
    }

    
    public void setAutonMT2Threshold() {
        angularVelocityThreshold = AUTON_MT2_ROTATION_THRESHOLD;
        poseDifferenceThreshold = AUTON_MT2_DISTANCE_THRESHOLD;
    }
}
