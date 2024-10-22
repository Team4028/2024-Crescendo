// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.generated.TunerConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final Translation2d SPEAKER_BLUE = new Translation2d(0.0, 5.5);
    public static final Translation2d SPEAKER_RED = new Translation2d(16.52, 5.5);

    public static final Rotation2d SHUTTLE_SHORT_OFFSET_BLUE = Rotation2d.fromDegrees(5);
    public static final Rotation2d SHUTTLE_SHORT_OFFSET_RED = Rotation2d.fromDegrees(-5);
    public static final double SHUTTLE_BEAN_MODIFIER = -0.01;

    public static final Pose2d AMP_TARGET = new Pose2d(1.83, 7.27, new Rotation2d(Math.PI / 2.));

    public static final Translation2d SHUTTLE_BLUE = new Translation2d(2.5, 8.22);

    public static final Pose2d RIGHT_SHOT = new Pose2d(5.1, 1.9, Rotation2d.fromDegrees(-38.));
    public static final Pose2d CENTER_SHOT = new Pose2d(4.5, 4.7, Rotation2d.fromDegrees(-15.5));
    public static final Pose2d LEFT_SHOT = new Pose2d(5.3, 6.5, Rotation2d.fromDegrees(14.));

    public static final Pose2d NOTE_C_SHOT = new Pose2d(2.75, 4.15, Rotation2d.fromDegrees(-34.));

    public static final double AUTON_SECONDS_LINE_DELAY = 0.5;

    public static double MAX_INDEX = 27.;
    public static double MIN_INDEX = 4.2;

    public static final class VBusConstants {
        public static final double CLIMBER_VBUS = 0.75;
        public static final double FAST_CLIMBER_VBUS = 0.95;

        public static final double INFEED_VBUS = 0.8;
        public static final double SLOW_INFEED_VBUS = 0.5;

        public static final double SLOW_CONVEYOR_VBUS = 0.5;
        public static final double FAST_CONVEYOR_VBUS = 0.85;

        public static final double FAN_VBUS = 1.;

        public static final double SHOOTER_BACKOUT_VBUS = -0.2;
        public static final double WHIPPY_VBUS = 0.2;
    }

    public static final class VisionConstants {
        public static final double CAMERA_SWITCH_TIMEOUT = 1.5;

        /** Pipeline for 3G MegaTag2 */
        public static final int MEGATAG_PIPELINE = 0;

        /** Pipeline for 3G 2D TY */
        public static final int TY_PIPELINE = 1;

        public static final String SHOOTER_LIMELIGHT = "limelight-ii";
        public static final String CHASSIS_LIMELIGHT = "limelight-gii";
        public static final String INFEED_LIMELIGHT_3G = "limelight-gi";

        public static final Transform3d SHOOTER_LIMELIGHT_TRANSFORM = new Transform3d();
        public static final Transform3d CHASSIS_LIMELIGHT_TRANSFORM = new Transform3d(-0.278, 0.17, 0.203,
                new Rotation3d(0, Units.degreesToRadians(33), 0));
        public static final Transform3d INFEED_LIMELIGHT_3G_TRANSFORM = new Transform3d();
    }

    public static final class DrivetrainConstants {
        public static final double MAX_SPEED = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top
                                                                                    // speed
        public static final double MAX_ANGULAR_SPEED = 4 * Math.PI; // 2rps

        public static final double BASE_SPEED = 0.25;
        public static final double SLOW_SPEED = 0.07;

        public double currentSpeed = BASE_SPEED;
    }

    public static enum AutoPoses {
        AMP_SHOT(new Translation2d(4.73, 6.95)), MID_SHOT(new Translation2d(2.89, 5.54)), SOURCE_SHOT(
                new Translation2d(3.87, 2.56)), NOTE1(new Translation2d(8.4, 7.29)), NOTE2(
                        new Translation2d(8.4, 5.65)), NOTE3(new Translation2d(8.4, 3.95)), NOTE4(
                                new Translation2d(8.4, 2.45)), NOTE5(new Translation2d(8.4, 0.71));

        public static Rotation2d UPWARD_ROTATION = Rotation2d.fromDegrees(42.5);
        public static Rotation2d DOWNWARD_ROTATION = Rotation2d.fromDegrees(-42.5);
        public static Rotation2d AMP_SHOT_ROTATION = Rotation2d.fromDegrees(30.05);
        public static Rotation2d MID_SHOT_ROTATION = Rotation2d.fromDegrees(0);
        public static Rotation2d SOURCE_SHOT_ROTATION = Rotation2d.fromDegrees(-36);
        public Translation2d pose;

        AutoPoses(Translation2d pose) {
            this.pose = pose;
        }
    }
}

// RED: CCW: RIGHT
// BLUE: CCW: LEFT
// 0.3 u pivot
// 0.5 u others
// 2 deg right