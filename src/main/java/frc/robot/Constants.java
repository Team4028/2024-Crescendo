// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

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
    public static final Pose2d SPEAKER_TARGET = new Pose2d(1.37, 5.56, new Rotation2d(Math.PI));
    public static final Pose2d SPEAKER_DISTANCE_TARGET = new Pose2d(0.2, 5.52, new Rotation2d(Math.PI));
    public static final Pose2d AMP_TARGET = new Pose2d(1.83, 7.27, new Rotation2d(Math.PI / 2.));
    public static final Pose2d LEFT_TRAP_Target = new Pose2d(4.4, 4.62  , new Rotation2d(Units.degreesToRadians(-60)));
    public static final Pose2d MIDDLE_TRAP_Target = new Pose2d(4.4, 3.24, new Rotation2d(Units.degreesToRadians(60)));
    public static final Pose2d RIGHT_TRAP_Target = new Pose2d(5.83, 4.11, new Rotation2d(Units.degreesToRadians(180)));
}
