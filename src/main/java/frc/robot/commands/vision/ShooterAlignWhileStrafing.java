// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision;
import frc.robot.utils.LimelightHelpers;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShooterAlignWhileStrafing extends ProfiledPIDCommand {
    private static final double OFFSET = -3.0;

    private static final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric();

    /** Creates a new WillRueter. */
    public ShooterAlignWhileStrafing(DoubleSupplier ySpeed, CommandSwerveDrivetrain drivetrain,
            Vision vision) {
        super(
                // The ProfiledPIDController used by the command
                new ProfiledPIDController(
                        // The PID gains
                        10.5,
                        0.0,
                        0.0,
                        // The motion profile constraints
                        new TrapezoidProfile.Constraints(1, 1)),
                // This should return the measurement
                () -> {
                    // int tagID = DriverStation.getAlliance().isPresent()
                    // && DriverStation.getAlliance().get() == Alliance.Red ? 4 : 7;

                    // Optional<Double> yaw = vision.getTagYaw(tagID);

                    // if (yaw.isPresent()) {
                    // return Units.degreesToRadians(yaw.get());
                    // }

                    // return 0.;
                    int tagID = DriverStation.getAlliance().isPresent()
                            && DriverStation.getAlliance().get() == Alliance.Red ? 4 : 7;

                    var fiducials = LimelightHelpers
                            .getLatestResults("limelight-shooter").targetingResults.targets_Fiducials;

                    for (var fiducial : fiducials) {
                        if (fiducial.fiducialID == tagID) {
                            return Units.degreesToRadians(fiducial.tx);
                        }
                    }

                    return 0.;

                },
                // This should return the goal (can also be a constant)
                () -> Units.degreesToRadians(OFFSET),
                // This uses the output
                (output, setpoint) -> {
                    drivetrain.setControl(
                            drive.withRotationalRate(output + setpoint.velocity).withVelocityY(ySpeed.getAsDouble()));
                });
        // Use addRequirements() here to declare subsystem dependencies.
        // Configure additional PID options by calling `getController` here.
        addRequirements(drivetrain, vision);

        getController().enableContinuousInput(-Math.PI, Math.PI);
        getController().setTolerance(Units.degreesToRadians(0.5));
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}