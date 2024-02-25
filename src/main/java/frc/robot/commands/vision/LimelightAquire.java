// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utils.LimelightHelpers;

public class LimelightAquire extends Command {
    private PIDController controller;

    private CommandSwerveDrivetrain drivetrain;

    private final SwerveRequest.ApplyChassisSpeeds drive = new SwerveRequest.ApplyChassisSpeeds();

    private final DoubleSupplier xSpeed;

    private static final double NOTE_PICKUP_THRESH = -18.5;
    // private static final double DRIVE_TO_NOTE_THRESH = 5;
    private static final double NOTE_ANGLE_THRESH = 1.;

    /** Creates a new LimelightDrive. */
    public LimelightAquire(DoubleSupplier xSpeed, CommandSwerveDrivetrain drivetrain) {
        this.xSpeed = xSpeed;
        addRequirements(this.drivetrain = drivetrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        controller = new PIDController(6., 0, 0.5);
        controller.reset();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        drivetrain.setControl(
                drive.withSpeeds(new ChassisSpeeds(
                        /* Math.abs(LimelightHelpers.getTX("")) <= DRIVE_TO_NOTE_THRESH ? */xSpeed.getAsDouble()/*
                                                                                                                 * : 0
                                                                                                                 */,
                        0,
                        Math.abs(LimelightHelpers.getTX("")) > NOTE_ANGLE_THRESH
                                ? controller.calculate(Units.degreesToRadians(LimelightHelpers.getTX("")))
                                : 0.)));
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return (LimelightHelpers.getTY("") < NOTE_PICKUP_THRESH) || !LimelightHelpers.getTV("");
    }
}
