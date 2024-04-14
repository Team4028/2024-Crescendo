// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Meters;

import java.util.Optional;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utils.ShooterTable.ShooterTableEntry;
import frc.robot.utils.ShooterTable.VisionTableEntry.CameraLerpStrat;

/** Add your docs here. */
public class ShootingStrategy {
    private final CommandSwerveDrivetrain drivetrain;
    private final VisionSystem vision;
    private final CameraLerpStrat visionStrategy;

    private static final Rotation2d OFFSET = Rotation2d.fromDegrees(-1.5);

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

    public Rotation2d getTargetOffset() {
        if (vision == null) {
            Translation2d translation = BeakUtils.goalTranslation(drivetrain.getState().Pose);

            Rotation2d totalAngle = translation.getAngle();
            return totalAngle.minus(drivetrain.getState().Pose.getRotation());
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

    public ShooterTableEntry getTargetEntry() {
        if (vision == null) {
            Translation2d translation = BeakUtils.goalTranslation(drivetrain.getState().Pose);

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
}
