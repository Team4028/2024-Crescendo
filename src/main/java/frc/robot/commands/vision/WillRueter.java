// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import java.util.Optional;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class WillRueter extends ProfiledPIDCommand {
    private static final double OFFSET = -2.0;

    private static final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric();

    /** Creates a new WillRueter. */
    public WillRueter(CommandSwerveDrivetrain drivetrain, Vision left, Vision right) {
        super(
                // The ProfiledPIDController used by the command
                new ProfiledPIDController(
                        // The PID gains
                        20.,
                        0,
                        0,
                        // The motion profile constraints
                        new TrapezoidProfile.Constraints(2, 2)),
                // This should return the measurement
                () -> {
                    int tagID = DriverStation.getAlliance().isPresent()
                            && DriverStation.getAlliance().get() == Alliance.Red ? 4 : 7;

                    Optional<Double> leftYaw = left.getTagYaw(tagID);
                    Optional<Double> rightYaw = right.getTagYaw(tagID);

                    int numPresent = leftYaw.isPresent() ? 1 : 0;
                    numPresent += rightYaw.isPresent() ? 1 : 0;

                    double theta = 0;

                    switch (numPresent) {
                        // both are present: relatively aligned & good
                        case 2: {
                            theta = leftYaw.get() + rightYaw.get();
                            break;
                        }
                        case 1: {
                            // only left is present; this means the robot is aligned too far to the right
                            // so we force the robot to the left
                            if (leftYaw.isPresent()) {
                                theta = -10.;
                            } else {
                                // same but for right; force the robot to the right
                                theta = 10.;
                            }
                        }
                        // bad things have happened
                        // in this case we don't know if we're aligned to the right or left
                        // so we ignore it and pray that it eventually finds it
                        case 0:
                        default: {
                            theta = 10.;
                        }
                    }

                    return Units.degreesToRadians(theta);
                },
                // This should return the goal (can also be a constant)
                () -> OFFSET,
                // This uses the output
                (output, setpoint) -> {
                    drivetrain.setControl(drive.withRotationalRate(output + setpoint.velocity));
                });
        // Use addRequirements() here to declare subsystem dependencies.
        // Configure additional PID options by calling `getController` here.
        addRequirements(drivetrain, left, right);

        getController().enableContinuousInput(-Math.PI, Math.PI);
        getController().setTolerance(Units.degreesToRadians(0.5));
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return getController().atGoal();
    }
}
