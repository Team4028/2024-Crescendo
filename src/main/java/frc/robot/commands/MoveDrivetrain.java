package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class MoveDrivetrain extends ProfiledPIDCommand {
    private static final SwerveRequest.ApplyChassisSpeeds drive = new SwerveRequest.ApplyChassisSpeeds()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private static final LinearFilter filter = LinearFilter.movingAverage(5);

    private static int taps = 5;

    public MoveDrivetrain(CommandSwerveDrivetrain drivetrain, DoubleSupplier target,
            DoubleSupplier measurement, boolean slow) {
        super(
                new ProfiledPIDController(slow ? 0.2 : 1.0, 0, 0, new TrapezoidProfile.Constraints(slow ? 1.0: 5.0, slow ? 1.0 : 5.0)),
                measurement,
                target,
                (output, setpoint) -> {
                    filter.calculate(measurement.getAsDouble() - target.getAsDouble());
                    taps++;
                    drivetrain.setControl(drive.withSpeeds(
                            new ChassisSpeeds(output + setpoint.velocity, 0, 0)));
                });

        addRequirements(drivetrain);
        getController().enableContinuousInput(-1., 1.);
    }

    @Override
    public void initialize() {
        super.initialize();
        filter.reset();
        taps = 0;
    }

    @Override
    public boolean isFinished() {
        return taps > 5 && Math.abs(filter.calculate(2)) < 0.1;
    }
}
