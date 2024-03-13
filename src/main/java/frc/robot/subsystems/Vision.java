package frc.robot.subsystems;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

public class Vision extends SubsystemBase {
    private final PhotonCamera camera;
    private AprilTagFieldLayout layout;
    private final Transform3d offset;

    private PhotonPoseEstimator estimator;

    public static final Transform3d LEFT_ROBOT_TO_CAMERA = new Transform3d(Units.inchesToMeters(0.),
            Units.inchesToMeters(+12.5), Units.inchesToMeters(0.),
            new Rotation3d(0., Units.degreesToRadians(28.125), Units.degreesToRadians(+150.)));

    public static final Transform3d RIGHT_ROBOT_TO_CAMERA = new Transform3d(Units.inchesToMeters(0.),
            Units.inchesToMeters(-12.5), Units.inchesToMeters(0.),
            new Rotation3d(0., Units.degreesToRadians(28.125),
                    Units.degreesToRadians(-150.)));

    public static final Transform3d SHOOTER_ROBOT_TO_CAMERA = new Transform3d(
            0.,
            0.,
            Units.inchesToMeters(18.75),
            new Rotation3d(0., Units.degreesToRadians(18.), 0.));

    public static final int TRAP_PIPELINE_INDEX = 1;
    public static final int SHOOTER_PIPELINE_INDEX = 0;

    // "2.5": 44 in (14 in)
    // "5.5": 80 in (14 in)
    // "7.5": 104 in (14 in)
    // "10.5": 140 in (14 in diff)
    // "14.5": 188 in (14 in diff)

    /**
     * Subsystem that handles a PhotonVision attached camera.
     * 
     * @param cameraName    The name of the camera, in the UI.
     * @param robotToCamera The transformation from the center of the robot to the
     *                      center of the camera lens.
     */
    public Vision(String cameraName, Transform3d robotToCamera) {
        camera = new PhotonCamera(cameraName);
        offset = robotToCamera;

        try {
            layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
            estimator = new PhotonPoseEstimator(layout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera,
                    robotToCamera);
            estimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_REFERENCE_POSE);
        } catch (IOException e) {
            System.err.println(e.getMessage());
            System.exit(1);
        }
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

        double yaw = target.get().getYaw();
        yaw += offset.getRotation().toRotation2d().getDegrees();

        return Optional.of(yaw);
    }

    public Optional<Double> getTagDistance(int tagID) {
        Optional<PhotonTrackedTarget> target = getTag(tagID);

        Optional<Double> yaw = getTagYaw(tagID);
        if (yaw.isEmpty())
            return Optional.empty();

        double distance = PhotonUtils.calculateDistanceToTargetMeters(
                offset.getZ(), layout.getTagPose(tagID).get().getZ(), offset.getRotation().getY(),
                Units.degreesToRadians(target.get().getPitch()));

        return Optional.of(distance);
    }

    public Optional<EstimatedRobotPose> getCameraResult(Pose2d prevPose) {
        estimator.setReferencePose(prevPose);
        Optional<EstimatedRobotPose> pose = estimator.update();
        return pose;
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

    public Command setPipelineCommand(int pipelineIndex) {
        return runOnce(() -> setPipeline(pipelineIndex));
    }

}
