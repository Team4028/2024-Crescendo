// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.util.Optional;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class Limelight extends VisionSystem {
    public Limelight(String cameraName, Transform3d robotToCamera) {
        super(cameraName, robotToCamera);
    }

    public Optional<Double> getTagYaw(int tagID) {
        var fiducials = LimelightHelpers
                .getLatestResults(cameraName).targetingResults.targets_Fiducials;

        for (var fiducial : fiducials) {
            if (fiducial.fiducialID == tagID) {
                return Optional.of(Units.degreesToRadians(fiducial.tx));
            }
        }

        return Optional.empty();
    }

    public Optional<Double> getTagPitch(int tagID) {
        var fiducials = LimelightHelpers
                .getLatestResults(cameraName).targetingResults.targets_Fiducials;

        for (var fiducial : fiducials) {
            if (fiducial.fiducialID == tagID) {
                return Optional.of(Units.degreesToRadians(fiducial.ty));
            }
        }

        return Optional.empty();
    }
}
