// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;

/** Add your docs here. */
public final class BeakUtils {
    /**
     * Returns true if the alliance is blue.
     */
    public static boolean allianceIsBlue() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        return alliance.isPresent() && alliance.get() == Alliance.Blue;
    }

    /** Get the central speaker AprilTag ID based on alliance. */
    public static int speakerTagID() {
        return BeakUtils.allianceIsBlue() ? 7 : 4;
    }

    /** Get the translation to a goal from a pose. */
    public static Translation2d goalTranslation(Pose2d pose) {
        Pose2d target = BeakUtils.allianceIsBlue() ? Constants.SPEAKER_DISTANCE_TARGET
                : Constants.SPEAKER_DISTANCE_TARGET_RED;

        return new Translation2d(pose.getX() - target.getX(), pose.getY() - target.getY());
    }
}
