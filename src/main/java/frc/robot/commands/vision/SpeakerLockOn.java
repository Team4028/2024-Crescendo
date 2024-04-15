// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utils.ShootingStrategy;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SpeakerLockOn extends PIDCommand {
    private static final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric();

    /** Creates a new WillRueter. */
    public SpeakerLockOn(CommandSwerveDrivetrain drivetrain, DoubleSupplier xSpeed, DoubleSupplier ySpeed,
            ShootingStrategy strategy) {
        super(
                // The ProfiledPIDController used by the command
                new PIDController(
                        // The PID gains
                        10.0,
                        0.0,
                        0.0),
                () -> drivetrain.getState().Pose.getRotation().getRadians(),
                // This should return the goal (can also be a constant)
                () -> drivetrain.getState().Pose.getRotation().plus(strategy.getTargetOffset()).getRadians(),
                // This uses the output
                (output) -> {
                    drivetrain.setControl(drive.withRotationalRate(output)
                            .withVelocityX(xSpeed.getAsDouble()).withVelocityY(ySpeed.getAsDouble()));
                });
        // Use addRequirements() here to declare subsystem dependencies.
        // Configure additional PID options by calling `getController` here.
        addRequirements(drivetrain);

        getController().enableContinuousInput(-Math.PI, Math.PI);
        // getController().setTolerance(Units.degreesToRadians(1.5));
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // return getController().atGoal();
        return false;
    }
}
