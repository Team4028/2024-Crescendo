// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AlignDrivetrain extends ProfiledPIDCommand {

    private static final SwerveRequest.FieldCentric m_drive = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.Velocity);

    /** Creates a new AlignDrivetrain. */
    public AlignDrivetrain(CommandSwerveDrivetrain drivetrain, DoubleSupplier target, DoubleSupplier measurement, boolean slow) {
        super(
                // The controller that the command will use
                new ProfiledPIDController(7.5, 0, 0, new TrapezoidProfile.Constraints(slow ? 1.5 : 5., slow ? 2.: 5.)),
                // This should return the measurement
                measurement,
                // This should return the setpoint (can also be a constant)
                target,
                // This uses the output
                (output, setpoint) -> {
                    drivetrain.setControl(m_drive.withRotationalRate(output + setpoint.velocity));
                });
        // Use addRequirements() here to declare subsystem dependencies.
        // Configure additional PID options by calling `getController` here.
        addRequirements(drivetrain);
        getController().enableContinuousInput(-Math.PI, Math.PI);
        getController().setTolerance(Units.degreesToRadians(1.5));
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return getController().atGoal();
    }
}
