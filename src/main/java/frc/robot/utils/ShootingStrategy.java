// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Meters;

import java.util.Optional;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utils.ShooterTable.ShooterTableEntry;
import frc.robot.utils.ShooterTable.VisionTableEntry.CameraLerpStrat;

/** Add your docs here. */
public class ShootingStrategy {
    private final CommandSwerveDrivetrain drivetrain;

    private final VisionSystem vision;
    private final CameraLerpStrat visionStrategy;

    private static final Rotation2d OFFSET = Rotation2d.fromDegrees(-3.0);

    /** Seconds */
    private static final double FORWARD_LOOK_TIME = 0.1;

    public ShootingStrategy(VisionSystem vision, CameraLerpStrat visionStrategy) {
        this.vision = vision;
        this.visionStrategy = visionStrategy;

        drivetrain = null;
    }

    public ShootingStrategy(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;

        vision = null;
        visionStrategy = null;
    }

    public Translation2d getDrivetrainTranslation(boolean moving) {
        Translation2d pose = drivetrain.getTranslation();

        if (moving) {
            ChassisSpeeds speeds = drivetrain.getFieldRelativeChassisSpeeds();
            pose = new Translation2d(
                pose.getX() + speeds.vxMetersPerSecond * FORWARD_LOOK_TIME,
                pose.getY() + speeds.vyMetersPerSecond * FORWARD_LOOK_TIME
            );
        }

        return BeakUtils.goalTranslation(pose);
    }

    public Rotation2d getTargetOffset(boolean moving) {
        if (vision == null) {
            Translation2d translation = getDrivetrainTranslation(moving);

            Rotation2d totalAngle = translation.getAngle();
            return totalAngle.minus(drivetrain.getRotation()).plus(OFFSET);
        } else if (drivetrain == null) {
            Optional<Rotation2d> angle = vision.getTagYaw(BeakUtils.speakerTagID());

            if (angle.isEmpty()) {
                return new Rotation2d();
            }

            return angle.get().plus(OFFSET);
        } else {
            return new Rotation2d();
        }
    }

    public Rotation2d getTargetOffset() {
        return getTargetOffset(false);
    }

    public ShooterTableEntry getTargetEntry(boolean moving) {
        if (vision == null) {
            Translation2d translation = getDrivetrainTranslation(moving);

            return ShooterTable.calcShooterTableEntry(Meters.of(translation.getNorm()));
        } else if (drivetrain == null) {
            Optional<Rotation2d> pitch = vision.getTagPitch(BeakUtils.speakerTagID());

            if (pitch.isEmpty()) {
                return ShooterTable.calcShooterTableEntry(Feet.of(20.0));
            }

            return ShooterTable.calcShooterTableEntryCamera(pitch.get().getDegrees(), visionStrategy);
        } else {
            return ShooterTable.calcShooterTableEntry(Feet.of(20.0));
        }
    }

    public ShooterTableEntry getTargetEntry() {
        return getTargetEntry(false);
    }
}
