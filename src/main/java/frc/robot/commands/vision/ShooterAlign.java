// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utils.ShootingStrategy;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShooterAlign extends ProfiledPIDCommand {
    private static final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric();

    /** Creates a new WillRueter. */
    public ShooterAlign(CommandSwerveDrivetrain drivetrain, ShootingStrategy strategy) {
        super(
                // The ProfiledPIDController used by the command
                new ProfiledPIDController(
                        // The PID gains
                        6.0,
                        0.0,
                        0.0,
                        // The motion profile constraints
                        new TrapezoidProfile.Constraints(2.0, 3.0)),
                // This should return the measurement
                () -> strategy.getTargetOffset().getRadians(),
                // This should return the goal (can also be a constant)
                () -> 0.0,
                // This uses the output
                (output, setpoint) -> {
                    drivetrain.setControl(drive.withRotationalRate(output + setpoint.velocity));
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
