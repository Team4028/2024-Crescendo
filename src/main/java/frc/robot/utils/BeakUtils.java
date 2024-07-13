// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.util.Optional;

import edu.wpi.first.math.geometry.Rotation2d;
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
    public static Translation2d goalTranslation(Translation2d pose) {
        Translation2d target = speakerTarget();

        return translationToTarget(pose, target);
    }

    /** Get the translation to the proper passing target */
    public static Translation2d passingTranslation(Translation2d pose) {
        return translationToTarget(pose, passingTarget());
    }

    public static Translation2d translationToTarget(Translation2d currentPose, Translation2d targetPose) {
        return new Translation2d(currentPose.getX() - targetPose.getX(), currentPose.getY() - targetPose.getY());
    }

    public static Translation2d speakerTarget() {
        return allianceIsBlue() ? Constants.SPEAKER_BLUE : Constants.SPEAKER_RED;
    }

    public static Translation2d passingTarget() {
        return allianceIsBlue() ? Constants.SHUTTLE_BLUE : Constants.SPEAKER_RED;
    }

    public static Rotation2d getShuttleOffset(boolean shortShuttle) {
        if (!shortShuttle)
            return Rotation2d.fromDegrees(0);
        return allianceIsBlue() ? Constants.SHUTTLE_SHORT_OFFSET_BLUE : Constants.SHUTTLE_SHORT_OFFSET_RED;
    }
}
