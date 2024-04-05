// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RotateToSpeaker extends ProfiledPIDCommand {
    private static final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric();
    private final CommandSwerveDrivetrain drivetrain;
    Field2d field = new Field2d();
    Field2d bruh = new Field2d();

    private static Rotation2d target = new Rotation2d();

    /** Creates a new RotateToSpeaker. */
    public RotateToSpeaker(CommandSwerveDrivetrain drivetrain) {
        super(
                // The ProfiledPIDController used by the command
                new ProfiledPIDController(
                        // The PID gains
                        10.,
                        0,
                        0,
                        // The motion profile constraints
                        new TrapezoidProfile.Constraints(2.0, 4.0)),
                // This should return the measurement
                () -> drivetrain.getState().Pose.getRotation().getRadians(),
                // This should return the goal (can also be a constant)
                () -> new TrapezoidProfile.State(target.getRadians(), 0.),
                // This uses the output
                (output, setpoint) -> {
                    // Use the output (and setpoint, if desired) here
                    drivetrain.setControl(drive.withRotationalRate(output + setpoint.velocity));
                });
        // Use addRequirements() here to declare subsystem dependencies.
        // Configure additional PID options by calling `getController` here.
        addRequirements(drivetrain);
        getController().enableContinuousInput(-Math.PI, Math.PI);

        this.drivetrain = drivetrain;
    }

    /* Check if we're blue */
    private boolean allianceIsBlue() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        return alliance.isPresent() && alliance.get() == Alliance.Blue;
    }

    @Override
    public void initialize() {
        super.initialize();

        Pose2d pose = drivetrain.getState().Pose;
        Pose2d target = allianceIsBlue() ? Constants.SPEAKER_DISTANCE_TARGET : Constants.SPEAKER_DISTANCE_TARGET_RED;

        Translation2d distance = new Translation2d(
                pose.getX() - target.getX(),
                pose.getY() - target.getY());

        Rotation2d angle = distance.getAngle();
        RotateToSpeaker.target = angle;
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
        // return Math.abs(
        // target.getDegrees() - drivetrain.getState().Pose.getRotation().getDegrees())
        // < 0.8;
    }
}
