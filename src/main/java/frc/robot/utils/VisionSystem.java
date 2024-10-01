package frc.robot.utils;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.PhotonUtils;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;

public class VisionSystem {
	protected final String cameraName;
	protected final Transform3d offset;

	protected static AprilTagFieldLayout layout = null;

	public static final Transform3d LEFT_ROBOT_TO_CAMERA = new Transform3d(Units.inchesToMeters(0.),
			Units.inchesToMeters(+12.5), Units.inchesToMeters(0.),
			new Rotation3d(0., Units.degreesToRadians(28.125), Units.degreesToRadians(+150.)));

	public static final Transform3d RIGHT_ROBOT_TO_CAMERA = new Transform3d(Units.inchesToMeters(0.),
			Units.inchesToMeters(-12.5), Units.inchesToMeters(0.),
			new Rotation3d(0., Units.degreesToRadians(28.125), Units.degreesToRadians(-150.)));

	public static final Transform3d SHOOTER_ROBOT_TO_CAMERA = new Transform3d(0., 0., Units.inchesToMeters(16.75),
			new Rotation3d(0., Units.degreesToRadians(28.), 0.));

	public static final Transform3d STATIONARY_ROBOT_TO_CAMERA = new Transform3d(0., 0., Units.inchesToMeters(7.75),
			new Rotation3d(0., Units.degreesToRadians(19), 0.));

	/**
	 * Subsystem that handles an attached camera.
	 * 
	 * @param cameraName
	 *            The name of the camera, in the UI.
	 * @param robotToCamera
	 *            The transformation from the center of the robot to the center of
	 *            the camera lens.
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

	public boolean getHasTarget() {
		return false;
	}

	public Optional<Rotation2d> getTargetX() {
		return Optional.empty();
	}

	public Optional<Rotation2d> getTargetY() {
		return Optional.empty();
	}

	public Optional<Rotation2d> getTagYaw(int tagID) {
		return Optional.empty();
	}

	public Optional<Rotation2d> getTagPitch(int tagID) {
		return Optional.empty();
	}

	public Optional<Double> getTagDistance(int tagID) {
		Optional<Rotation2d> pitch = getTagPitch(tagID);

		if (pitch.isEmpty()) {
			return Optional.empty();
		}

		double distance = PhotonUtils.calculateDistanceToTargetMeters(offset.getZ(),
				layout.getTagPose(tagID).get().getZ(), offset.getRotation().getY(), pitch.get().getRadians());
		// System.out.println("====================================================");
		// System.out.println("offset.getZ: " + offset.getZ());
		// System.out.println("layout.getTagPose: " +
		// layout.getTagPose(tagID).get().getZ());
		// System.out.println("camera pitch: " + offset.getRotation().getY());
		// System.out.println("Tag pitch: " + pitch.get().getRadians());
		// System.out.println("DISTANCE CALCULATION: " + distance);

		return Optional.of(distance);
	}

	public void setPipeline(int pipelineIndex) {
	}

	public int getPipeline() {
		return 0;
	}

	public Command setPipelineCommand(int pipelineIndex) {
		return Commands.runOnce(() -> setPipeline(pipelineIndex));
	}
}
