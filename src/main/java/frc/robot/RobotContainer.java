// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Infeed;
import frc.robot.subsystems.Shooter;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
    private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain;
    private final Infeed infeed = Infeed.getInstance();
    public final Shooter shooter = new Shooter();
    private final Conveyor conveyor = new Conveyor();
    private final Climber m_climber = new Climber();

    private final Command smartInfeedCommand;
    private SendableChooser<Command> autonChooser;

    private final SlewRateLimiter xLimiter = new SlewRateLimiter(4.);
    private final SlewRateLimiter yLimiter = new SlewRateLimiter(4.);
    private final SlewRateLimiter thetaLimiter = new SlewRateLimiter(4.);
    private final SlewRateLimiter xLimeAquireLimiter = new SlewRateLimiter(4.);

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                                     // driving in open loop
    private final Telemetry logger = new Telemetry(MaxSpeed);

    private void initAutonChooser() {
        autonChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autonChooser);
    }

    private void initNamedCommands() {
        NamedCommands.registerCommand("startShooter",
                shooter.setSlot(1).andThen(() -> shooter.runPivotToPosition(shooter.getPivotPosition()))
                        .andThen(shooter.runVelocityCommand()));
        NamedCommands.registerCommand("4pinfeed", infeed.runInfeedMotorCommand(0.8)
                .alongWith(conveyor.runMotorCommand(0.85)).repeatedly());// .withTimeout(1.5));
        NamedCommands.registerCommand("farShot", Commands.runOnce(() -> shooter.runPivotToPosition(14.25)));
        // NamedCommands.registerCommand("shoot", feeder.runXRotations(10));
    }

    private void configureBindings() {
        drivetrain.setDefaultCommand(
                drivetrain.applyRequest(() -> drive
                        .withVelocityX(scaleDriverController(-driverController.getLeftY(),
                                xLimiter,
                                .25) * MaxSpeed)
                        .withVelocityY(scaleDriverController(-driverController.getLeftX(),
                                yLimiter,
                                .25) * MaxSpeed)
                        .withRotationalRate(
                                scaleDriverController(-driverController.getRightX(),
                                        thetaLimiter, .25) *
                                        MaxSpeed)));

        conveyor.setDefaultCommand(conveyor.runMotorCommand(0.));
        infeed.setDefaultCommand(infeed.runInfeedMotorCommand(0.));

        driverController.leftTrigger().onTrue(
                infeed.runInfeedMotorCommand(.8).alongWith(
                        conveyor.runMotorCommand(0.5)).repeatedly())
                .onFalse(infeed.runInfeedMotorCommand(0.).alongWith(
                        conveyor.runMotorCommand(0.)));

        driverController.a().onTrue(conveyor.runXRotations(10).alongWith(infeed.runInfeedMotorCommand(0.5)))
                .onFalse(infeed.runInfeedMotorCommand(0.5));

        driverController.b().toggleOnTrue(smartInfeedCommand);

        driverController.x().and(driverController.povCenter())
                .toggleOnTrue(shooter.runVelocityCommand());
        driverController.x().and(driverController.povDown())
                .onTrue(shooter.cycleDownCommand());
        driverController.x().and(driverController.povUp())
                .onTrue(shooter.cycleUpCommand());

        driverController.y().onTrue(shooter.pivotZeroCommand());

        // reset the field-centric heading on start
        driverController.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative(new Pose2d())));

        driverController.rightBumper().onTrue(shooter.runPivotCommand(0.3))
                .onFalse(shooter.runPivotCommand(0.0));
        driverController.leftBumper().onTrue(shooter.runPivotCommand(-0.3))
                .onFalse(shooter.runPivotCommand(0.0));

        driverController.povRight()
                .onTrue(shooter.run(() -> shooter.runPivotToPosition(shooter.getPivotPosition())));

        if (Utils.isSimulation()) {
            drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
        }

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    private double scaleDriverController(double controllerInput, SlewRateLimiter limiter, double baseSpeedPercent) {
        return limiter.calculate(
                controllerInput * (baseSpeedPercent
                        + driverController.getRightTriggerAxis() * (1 - baseSpeedPercent)));
    }

    public RobotContainer() {
        smartInfeedCommand = infeed.runInfeedMotorCommand(0.8).alongWith(conveyor.runMotorCommand(0.5))
                .repeatedly().until(conveyor.hasInfedSupplier())
                .andThen(infeed.runInfeedMotorCommand(0.).alongWith(conveyor.runMotorCommand(0.))
                        .repeatedly().withTimeout(0.1))
                .andThen(shooter.spinMotorRightCommand(-0.4).repeatedly()
                        .raceWith(conveyor.runXRotations(-.5).withTimeout(0.25)
                                .alongWith(infeed.runInfeedMotorCommand(0.))))
                .andThen(shooter.spinMotorRightCommand(0.));
        // .until(() -> !conveyor.hasGamePiece()))
        // .andThen(conveyor.runXRotations(0.).alongWith(infeed.runInfeedMotorCommand(0.)));

        initNamedCommands();
        initAutonChooser();
        configureBindings();
    }

    public Command getAutonomousCommand() {
        return new InstantCommand(() -> drivetrain.seedFieldRelative(new Pose2d()))
                .alongWith(shooter.pivotZeroCommand()).andThen(autonChooser.getSelected());
    }

    public void logDrivetrainValues() {
        drivetrain.logValues();
        conveyor.logValues();
        infeed.logValues();
    }
}
