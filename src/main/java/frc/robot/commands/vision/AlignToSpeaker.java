// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utils.ShootingStrategy;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AlignToSpeaker extends ProfiledPIDCommand {
    private static final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric();

    private final CommandSwerveDrivetrain drivetrain;
    private final ShootingStrategy strategy;

    private static Rotation2d target;

    /** Creates a new epicness. */
    public AlignToSpeaker(CommandSwerveDrivetrain drivetrain, ShootingStrategy strategy) {
        super(
                // The ProfiledPIDController used by the command
                new ProfiledPIDController(
                        // The PID gains
                        10.0,
                        0.0,
                        0.0,
                        // The motion profile constraints
                        new TrapezoidProfile.Constraints(6.0, 12.0)),
                // This should return the measurement
                () -> drivetrain.getState().Pose.getRotation().getRadians(),
                // This should return the goal (can also be a constant)
                () -> AlignToSpeaker.target.getRadians(),
                // This uses the output
                (output, setpoint) -> {
                    drivetrain.setControl(drive.withRotationalRate(output + setpoint.velocity));
                });
        // Use addRequirements() here to declare subsystem dependencies.
        // Configure additional PID options by calling `getController` here.
        addRequirements(drivetrain);

        this.drivetrain = drivetrain;
        this.strategy = strategy;

        getController().enableContinuousInput(-Math.PI, Math.PI);
        // getController().setTolerance(Units.degreesToRadians(1.5));
    }

    @Override
    public void initialize() {
        super.initialize();

        AlignToSpeaker.target = drivetrain.getState().Pose.getRotation().plus(strategy.getTargetOffset());
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // return getController().atGoal();
        return false;
    }
}
