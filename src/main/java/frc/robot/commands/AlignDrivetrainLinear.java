package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AlignDrivetrainLinear extends ProfiledPIDCommand {
    private static final SwerveRequest.ApplyChassisSpeeds drive = new SwerveRequest.ApplyChassisSpeeds()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private static final LinearFilter filter = LinearFilter.movingAverage(5);

    private static int taps = 5;

    public AlignDrivetrainLinear(CommandSwerveDrivetrain drivetrain, DoubleSupplier target,
            DoubleSupplier measurement) {
        super(
                new ProfiledPIDController(1, 0, 0, new TrapezoidProfile.Constraints(5, 5)),
                measurement,
                target,
                (output, setpoint) -> {
                    SmartDashboard.putNumber("dr pos", measurement.getAsDouble());
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
