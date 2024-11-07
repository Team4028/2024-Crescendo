package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utils.LimelightHelpers;

public class LimelightsquareTest extends Command {
    // find tx < 0, then xsupplier - 0.1;
    // if tx<0, xsupplier +0.1
    private static final double testThreshold = -2;
    private boolean m_hasReachedThreshold = false;
    private boolean forever;

    private DoubleSupplier xSupplier = () -> (LimelightHelpers.getTX("") < 0) ? -0.1 : 0.1;
    private DoubleSupplier ySupplier;

    private CommandSwerveDrivetrain drivetrain;

    private PIDController controller;

    private LimelightsquareTest(boolean forever, DoubleSupplier xSupplier, DoubleSupplier ySupplier,
            CommandSwerveDrivetrain drivetrain) {
        this.forever = forever;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.drivetrain = drivetrain;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        controller = new PIDController(DriverStation.isAutonomousEnabled() ? 7.5 : 5.0, 0., 0.);
    }

    @Override
    public void execute() {
        if (!m_hasReachedThreshold)
            m_hasReachedThreshold = (LimelightHelpers
                    .getTY("") < (testThreshold));

        double rotationOutput = controller.calculate(Units.degreesToRadians(LimelightHelpers.getTX("")));

        drivetrain.applyRequest(() -> new SwerveRequest.ApplyChassisSpeeds().withSpeeds(new ChassisSpeeds(
                xSupplier.getAsDouble(),
                ySupplier.getAsDouble(),
                forever || !m_hasReachedThreshold ? rotationOutput : 0.)));
    }

}
