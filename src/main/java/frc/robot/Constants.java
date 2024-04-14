// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final Pose2d SPEAKER_TARGET = new Pose2d(1.37, 5.8, new Rotation2d(Math.PI));
    public static final Pose2d SPEAKER_DISTANCE_TARGET = new Pose2d(0.0, 5.65, new Rotation2d(Math.PI));
    public static final Pose2d SPEAKER_DISTANCE_TARGET_RED = new Pose2d(0.0, 5.25, new Rotation2d(Math.PI));

    public static final Pose2d AMP_TARGET = new Pose2d(1.83, 7.27, new Rotation2d(Math.PI / 2.));

    public static final Pose2d SHUTTLING_TARGET = new Pose2d(10.0, 0.78, Rotation2d.fromDegrees(-40.73));

    public static final Pose2d RIGHT_SHOT = new Pose2d(5.1, 1.9, Rotation2d.fromDegrees(-38.));
    public static final Pose2d CENTER_SHOT = new Pose2d(4.5, 4.7, Rotation2d.fromDegrees(-15.5));
    public static final Pose2d LEFT_SHOT = new Pose2d(5.3, 6.5, Rotation2d.fromDegrees(14.));

    public static final Pose2d NOTE_C_SHOT = new Pose2d(2.75, 4.15, Rotation2d.fromDegrees(-34.));
}

// RED: CCW: RIGHT
// BLUE: CCW: LEFT
// 0.3 u pivot
// 0.5 u others
// 2 deg right