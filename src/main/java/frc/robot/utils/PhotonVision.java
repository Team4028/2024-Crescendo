// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class PhotonVision extends VisionSystem {
    private final PhotonCamera camera;

    private final PhotonPoseEstimator estimator;

    public PhotonVision(String cameraName, Transform3d robotToCamera) {
        super(cameraName, robotToCamera);

        camera = new PhotonCamera(cameraName);

        estimator = new PhotonPoseEstimator(layout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera,
                robotToCamera);
        estimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_REFERENCE_POSE);

    }

    public Optional<PhotonTrackedTarget> getTag(int tagID) {
        for (PhotonTrackedTarget target : camera.getLatestResult().getTargets()) {
            if (target.getFiducialId() == tagID) {
                return Optional.of(target);
            }
        }

        return Optional.empty();
    }

    public Optional<Double> getTagYaw(int tagID) {
        Optional<PhotonTrackedTarget> target = getTag(tagID);
        if (target.isEmpty())
            return Optional.empty();

        double yaw = Units.degreesToRadians(target.get().getYaw());

        return Optional.of(yaw);
    }

    public Optional<Double> getTagPitch(int tagID) {
        Optional<PhotonTrackedTarget> target = getTag(tagID);
        if (target.isEmpty())
            return Optional.empty();

        double pitch = target.get().getPitch();

        return Optional.of(pitch);
    }

    public Optional<EstimatedRobotPose> getCameraResult(Pose2d prevPose) {
        estimator.setReferencePose(prevPose);
        Optional<EstimatedRobotPose> pose = estimator.update();
        if (pose.isPresent() && !pose.get().strategy.equals(PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR)) {
            int cameraTargetID = pose.get().targetsUsed.get(0).getFiducialId();
            if (cameraTargetID != 7 && cameraTargetID != 8)
                return Optional.empty();
        }
        return pose;
    }

    public Optional<Translation2d> getBestTranslationToTarget(int tagID) {
        var target = this.getTag(tagID);

        // cam: 9'11.5" real: 9'1"
        //

        if (target.isEmpty()) {
            return Optional.empty();
        }

        return Optional.of(target.get().getBestCameraToTarget().getTranslation().toTranslation2d());
    }

    public PhotonTrackedTarget getBestTarget() {
        var result = camera.getLatestResult();
        if (!result.hasTargets())
            return null;
        return result.getBestTarget();
    }

    public PhotonTrackedTarget getTargetById(int id) {
        var result = camera.getLatestResult();

        for (var target : result.getTargets()) {
            if (target.getFiducialId() == id)
                return target;
        }

        return null;
    }

    public void setPipeline(int pipelineIndex) {
        camera.setPipelineIndex(pipelineIndex);
    }
}
