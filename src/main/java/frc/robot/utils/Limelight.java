// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.util.Optional;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;

/** Add your docs here. */
public class Limelight extends VisionSystem {
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
        return Optional.of(/*compensatedDistance*/dist.get() / Math.cos(-tx.get().getRadians()));
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

    public void setPipeline(int pipeline) {
        LimelightHelpers.setPipelineIndex(cameraName, pipeline);
    }

    public int getPipeline() {
        return (int) LimelightHelpers.getCurrentPipelineIndex(cameraName);
    }
}
