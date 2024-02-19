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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

public class Vision extends SubsystemBase {
    private final PhotonCamera m_camera;
    private final AprilTagFieldLayout m_layout;
    private final Transform3d m_offset;

    private final PhotonPoseEstimator m_estimator;
    private Field2d m_field = new Field2d();

    public static final Transform3d leftCameraToRobot = new Transform3d(Units.inchesToMeters(10.5),
            Units.inchesToMeters(10.0), Units.inchesToMeters(10.0),
            new Rotation3d(0., Units.degreesToRadians(28.125), Units.degreesToRadians(-22.)));

    public static final Transform3d rightCameraToRobot = new Transform3d(Units.inchesToMeters(10.5),
            Units.inchesToMeters(-10.0), Units.inchesToMeters(10.0),
            new Rotation3d(0., Units.degreesToRadians(28.125),
                    Units.degreesToRadians(22.)));

    /**
     * Subsystem that handles a PhotonVision attached camera.
     * 
     * @param cameraName    The name of the camera, in the UI.
     * @param cameraToRobot The transformation from the camera to the center of the
     *                      robot.
     */
    public Vision(String cameraName, Transform3d cameraToRobot) throws IOException {
        m_camera = new PhotonCamera(cameraName);
        m_offset = cameraToRobot;

        m_layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);

        m_estimator = new PhotonPoseEstimator(m_layout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, m_camera,
                cameraToRobot);
        m_estimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_REFERENCE_POSE);
    }

    public Optional<PhotonTrackedTarget> getTag(int tagID) {
        for (PhotonTrackedTarget target : m_camera.getLatestResult().getTargets()) {
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
        yaw += m_offset.getRotation().toRotation2d().getDegrees();

        return Optional.of(yaw);
    }

    public Optional<Double> getTagDistance(int tagID) {
        Optional<PhotonTrackedTarget> target = getTag(tagID);

        Optional<Double> yaw = getTagYaw(tagID);
        if (yaw.isEmpty())
            return Optional.empty();

        double distance = PhotonUtils.calculateDistanceToTargetMeters(
                m_offset.getZ(), m_layout.getTagPose(tagID).get().getZ(), m_offset.getRotation().getY(),
                target.get().getPitch());

        return Optional.of(distance);
    }

    public Optional<EstimatedRobotPose> getCameraResult(Pose2d prevPose) {
        m_estimator.setReferencePose(prevPose);
        Optional<EstimatedRobotPose> pose = m_estimator.update();
        if (pose.isPresent()) {
            return pose;
        } else {
            return Optional.empty();
        }
    }

    public PhotonTrackedTarget getBestTarget() {
        var result = m_camera.getLatestResult();
        if (!result.hasTargets())
            return null;
        return result.getBestTarget();
    }

    public PhotonTrackedTarget getTargetById(int id) {
        var result = m_camera.getLatestResult();

        for (var target : result.getTargets()) {
            if (target.getFiducialId() == id)
                return target;
        }

        return null;
    }

    public void publishResults() {
        PhotonTrackedTarget target = getBestTarget();
        if (target == null)
            return;

        double yaw = target.getYaw();
        double pitch = target.getPitch();
        double Area = target.getArea();
        double Skew = target.getSkew();

        Transform3d threeDt = target.getBestCameraToTarget();
        Transform2d pose = new Transform2d(new Translation2d(threeDt.getX(), threeDt.getY()), new Rotation2d());

        SmartDashboard.putNumber(m_camera.getName() + " X distance", pose.getX());
        SmartDashboard.putNumber(m_camera.getName() + " Y distance", pose.getY());
        SmartDashboard.putNumber(m_camera.getName() + " yaw", yaw);
        SmartDashboard.putNumber(m_camera.getName() + " pitch", pitch);
        SmartDashboard.putNumber(m_camera.getName() + " area", Area);
        SmartDashboard.putNumber(m_camera.getName() + " skew", Skew);
        m_field.setRobotPose(getCameraResult(new Pose2d()).get().estimatedPose.toPose2d());
    }
}
