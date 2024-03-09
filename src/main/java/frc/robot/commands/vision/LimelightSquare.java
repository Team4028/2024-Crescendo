package frc.robot.commands.vision;

import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utils.LimelightHelpers;

public class LimelightSquare extends Command {
    private PIDController m_controller;
    private boolean m_continuous;
    private boolean m_hasReachedThreshold = false;

    private CommandSwerveDrivetrain drivetrain;

    private final DoubleSupplier m_xSupplier, m_ySupplier;

    private final SwerveRequest.ApplyChassisSpeeds drive = new SwerveRequest.ApplyChassisSpeeds();

    private static final double NOTE_PICKUP_THRESH = 5;
    private static final double NOTE_ANGLE_THRESH = 1;

    public LimelightSquare(boolean continuous, DoubleSupplier xSupplier, DoubleSupplier ySupplier,
            CommandSwerveDrivetrain drivetrain) {
        m_continuous = continuous;
        this.drivetrain = drivetrain;
        m_xSupplier = xSupplier;
        m_ySupplier = ySupplier;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        m_controller = new PIDController(1.7, 0., 0.2);
        m_controller.reset();
    }

    @Override
    public void execute() {
        drivetrain.setControl(
                drive.withSpeeds(
                        new ChassisSpeeds(
                                m_xSupplier.getAsDouble(),
                                m_ySupplier.getAsDouble(),
                                (m_continuous || !m_hasReachedThreshold)
                                        && Math.abs(LimelightHelpers.getTX("")) > NOTE_ANGLE_THRESH
                                                ? m_controller
                                                        .calculate(Units.degreesToRadians(LimelightHelpers.getTX("")))
                                                : 0)));
    }

    @Override
    public boolean isFinished() {
        return m_continuous ? false
                : (LimelightHelpers
                        .getTY("") < NOTE_PICKUP_THRESH);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(drive.withSpeeds(new ChassisSpeeds()));
    }
}
