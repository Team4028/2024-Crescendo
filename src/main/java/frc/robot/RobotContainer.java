// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Infeed;

public class RobotContainer {
  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain;
  private final Infeed infeed = Infeed.getInstance();
  SendableChooser<Command> autonChooser;

  SlewRateLimiter xLimiter = new SlewRateLimiter(4.);
  SlewRateLimiter yLimiter = new SlewRateLimiter(4.);
  SlewRateLimiter thetaLimiter = new SlewRateLimiter(4.);
  SlewRateLimiter xLAquireLimiter = new SlewRateLimiter(4.);

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final Telemetry logger = new Telemetry(MaxSpeed);

  private void initAutonChooser() {
    autonChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autonChooser);
  }

  private void configureBindings() {
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive
            .withVelocityX(scaleDriverController(-driverController.getLeftY(), xLimiter, .4) * MaxSpeed)
            .withVelocityY(scaleDriverController(-driverController.getLeftX(), yLimiter, .4) * MaxSpeed)
            .withRotationalRate(
                scaleDriverController(-driverController.getRightX(), thetaLimiter, .4) * MaxSpeed)));

    driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
    driverController.b().whileTrue(drivetrain
        .applyRequest(() -> point
            .withModuleDirection(new Rotation2d(-driverController.getLeftY(), -driverController.getLeftX()))));

    driverController.x().onTrue(infeed.runInfeedMotorCommand(.9)).onFalse(infeed.runInfeedMotorCommand(0.));
    driverController.y().onTrue(infeed.runInfeedMotorCommand(-.9)).onFalse(infeed.runInfeedMotorCommand(0.));

    // reset the field-centric heading on left bumper press
    driverController.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative(new Pose2d())));
    driverController.back().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative(new Pose2d(0., 0., Rotation2d.fromDegrees(180)))));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);
  }

  private double scaleDriverController(double controllerInput, SlewRateLimiter limiter, double baseSpeedPercent) {
    return limiter.calculate(
        controllerInput * (baseSpeedPercent + driverController.getRightTriggerAxis() * (1 - baseSpeedPercent)));
  }

  public RobotContainer() {
    initAutonChooser();
    configureBindings();
  }

  public Command getAutonomousCommand() {
    return new InstantCommand(drivetrain::seedFieldRelative).andThen(autonChooser.getSelected());
  }

  public void logDrivetrainValues() {
    drivetrain.logValues();
  }
}
