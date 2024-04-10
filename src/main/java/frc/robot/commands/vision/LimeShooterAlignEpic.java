// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utils.LimelightHelpers;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LimeShooterAlignEpic extends ProfiledPIDCommand {
    private static final double OFFSET = -3.0;

    private static final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric();

    private final CommandSwerveDrivetrain drivetrain;

    private static Rotation2d target;

    /** Creates a new epicness. */
    public LimeShooterAlignEpic(CommandSwerveDrivetrain drivetrain) {
        super(
                // The ProfiledPIDController used by the command
                new ProfiledPIDController(
                        // The PID gains
                        6.0,
                        0.0,
                        0.0,
                        // The motion profile constraints
                        new TrapezoidProfile.Constraints(2.0, 4.0)),
                // This should return the measurement
                () -> drivetrain.getState().Pose.getRotation().getRadians(),
                // This should return the goal (can also be a constant)
                () -> LimeShooterAlignEpic.target.getRadians(),
                // This uses the output
                (output, setpoint) -> {
                    // System.out.println("The thing is doing >:D");
                    SmartDashboard.putNumber("output", output + setpoint.velocity);
                    drivetrain.setControl(drive.withRotationalRate(output + setpoint.velocity));
                });
        // Use addRequirements() here to declare subsystem dependencies.
        // Configure additional PID options by calling `getController` here.
        addRequirements(drivetrain);

        this.drivetrain = drivetrain;

        getController().enableContinuousInput(-Math.PI, Math.PI);
        // getController().setTolerance(Units.degreesToRadians(1.5));
    }

    @Override
    public void initialize() {
        super.initialize();

        double offset = 0.0;

        int tagID = DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Red ? 4 : 7;

        var fiducials = LimelightHelpers
                .getLatestResults("limelight-shooter").targetingResults.targets_Fiducials;

        for (var fiducial : fiducials) {
            if (fiducial.fiducialID == tagID) {
                offset = -Units.degreesToRadians(fiducial.tx);
            }
        }

        LimeShooterAlignEpic.target = drivetrain.getState().Pose.getRotation()
                .plus(new Rotation2d(offset + Units.degreesToRadians(OFFSET)));
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // return getController().atGoal();
        return false;
    }
}
