package frc.robot.utils;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.PhotonUtils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;

public class VisionSystem {
    protected final String cameraName;
    protected final Transform3d offset;
    public static final Odometry None = new Odometry(TunerConstants.DriveTrain);

    protected static AprilTagFieldLayout layout = null;

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
            Units.inchesToMeters(16.75),
            new Rotation3d(0., Units.degreesToRadians(28.), 0.));

    public static final Transform3d STATIONARY_ROBOT_TO_CAMERA = new Transform3d(
            0.,
            0.,
            Units.inchesToMeters(7.75),
            new Rotation3d(0., Units.degreesToRadians(19), 0.));

    public static final int TRAP_PIPELINE_INDEX = 1;
    public static final int SHOOTER_PIPELINE_INDEX = 0;

    private static final class Odometry extends VisionSystem {
        private final CommandSwerveDrivetrain drivetrain;

        private Odometry(CommandSwerveDrivetrain drivetrain) {
            super("", new Transform3d());
            this.drivetrain = drivetrain;
        }

        public Optional<Double> getTagYaw(int tagID) {
            var tagPose = layout.getTagPose(tagID);
            if (tagPose.isEmpty())
                return Optional.empty();
            Pose2d tagPose2d = tagPose.get().toPose2d();
            var rPose = drivetrain.getState().Pose;
            var rawYaw = Math.atan2(rPose.getY() - tagPose2d.getY(), rPose.getX() - tagPose2d.getX());
            return Optional.of(rawYaw - rPose.getRotation().getRadians());
        }

        public Optional<Double> getTagPitch(int tagID) {
            var tagPose = layout.getTagPose(tagID);
            if (tagPose.isEmpty())
                return Optional.empty();
            Pose3d tagPose3d = tagPose.get();
            Pose3d rPose = new Pose3d(drivetrain.getState().Pose);
            var dist2d = Math
                    .sqrt(Math.pow(rPose.getX() - tagPose3d.getX(), 2) + Math.pow(rPose.getY() - tagPose3d.getY(), 2));
            return Optional.of(Math.atan2(tagPose3d.getZ() - rPose.getZ(), dist2d));
        }

        public Optional<Double> getTagDistance(int tagID) {
            var tagPose = layout.getTagPose(tagID);
            if (tagPose.isEmpty())
                return Optional.empty();
            Pose3d tagPose3d = tagPose.get();
            Pose3d rPose = new Pose3d(drivetrain.getState().Pose);
            var dist2d = Math
                    .sqrt(Math.pow(rPose.getX() - tagPose3d.getX(), 2) + Math.pow(rPose.getY() - tagPose3d.getY(), 2));
            return Optional.of(Math.sqrt(Math.pow(tagPose3d.getZ() - rPose.getZ(), 2) + dist2d * dist2d));
        }
    }

    // "2.5": 44 in (14 in)
    // "5.5": 80 in (14 in)
    // "7.5": 104 in (14 in)
    // "10.5": 140 in (14 in diff)
    // "14.5": 188 in (14 in diff)

    /**
     * Subsystem that handles an attached camera.
     * 
     * @param cameraName    The name of the camera, in the UI.
     * @param robotToCamera The transformation from the center of the robot to the
     *                      center of the camera lens.
     */
    public VisionSystem(String cameraName, Transform3d robotToCamera) {
        this.cameraName = cameraName;

        offset = robotToCamera;

        if (layout == null) {
            try {
                layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
            } catch (IOException e) {
                System.err.println(e.getMessage());
                System.exit(1);
            }
        }
    }

    public static AprilTagFieldLayout layout() {
        return layout;
    }

    public void configFieldOrigin() {
        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
            layout.setOrigin(OriginPosition.kRedAllianceWallRightSide);
        } else {
            layout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
        }
    }

    public Optional<Double> getTagYaw(int tagID) {
        return Optional.empty();
    }

    public Optional<Double> getTagPitch(int tagID) {
        return Optional.empty();
    }

    public Optional<Double> getTagDistance(int tagID) {
        Optional<Double> pitch = getTagPitch(tagID);

        if (pitch.isEmpty()) {
            return Optional.empty();
        }

        double distance = PhotonUtils.calculateDistanceToTargetMeters(
                offset.getZ(), layout.getTagPose(tagID).get().getZ(), offset.getRotation().getY(),
                Units.degreesToRadians(pitch.get()));

        return Optional.of(distance);
    }

    public void setPipeline(int pipelineIndex) {
    }

    public Command setPipelineCommand(int pipelineIndex) {
        return Commands.runOnce(() -> setPipeline(pipelineIndex));
    }
}
