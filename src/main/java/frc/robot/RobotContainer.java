// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Conveyor;
// import frc.robot.subsystems.Fan;
import frc.robot.subsystems.Infeed;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;
import frc.robot.utils.ShooterTable;
import frc.robot.utils.ShooterTable.ShooterTableEntry;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
    private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandSwerveDrivetrain m_drivetrain = TunerConstants.DriveTrain;
    private final Infeed infeed = Infeed.getInstance();
    public final Shooter shooter = new Shooter();
    private final Conveyor conveyor = new Conveyor();
    private final Vision m_rightVision = new Vision("Right_AprilTag_Camera", Vision.rightCameraToRobot);
    private final Vision m_leftVision = new Vision("Left_AprilTag_Camera", Vision.leftCameraToRobot);
    // private final Fan m_fan = new Fan();

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
        m_drivetrain.setDefaultCommand(
                m_drivetrain.applyRequest(() -> drive
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

        // driverController.y().onTrue(shooter.pivotZeroCommand());
        // driverController.y().toggleOnTrue(m_fan.runMotorCommand(1.0));
        // driverController.y().onTrue(
        // Commands.runOnce(() -> SmartDashboard.putNumber("Distance",
        // Units.metersToFeet(getBestDistance(7)))));
        driverController.y().onTrue(Commands.runOnce(this::printSTVals));

        // reset the field-centric heading on start
        driverController.start().onTrue(m_drivetrain.runOnce(() -> m_drivetrain.seedFieldRelative(new Pose2d())));
        // driverController.back()
        // .toggleOnTrue(new AlignDrivetrain(drivetrain, () -> 0., () ->
        // Units.degreesToRadians(getBestYaw(7))));
        driverController.back().and(driverController.povCenter())
                .onTrue(m_drivetrain.addMeasurementCommand(() -> getBestPose()));
        driverController.back().and(driverController.povDown()).onTrue(Commands.runOnce(() -> {
            var pose = getBestPose();
            if (pose.isPresent())
                m_drivetrain.seedFieldRelative(pose.get().estimatedPose.toPose2d());
        }));

        driverController.rightBumper().onTrue(shooter.runPivotCommand(0.3))
                .onFalse(shooter.runPivotCommand(0.0));
        driverController.leftBumper().onTrue(shooter.runPivotCommand(-0.3))
                .onFalse(shooter.runPivotCommand(0.0));

        driverController.povRight()
                .onTrue(shooter.run(() -> shooter.runPivotToPosition(shooter.getPivotPosition())));

        if (Utils.isSimulation()) {
            m_drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
        }

        m_drivetrain.registerTelemetry(logger::telemeterize);
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
        return new InstantCommand(() -> m_drivetrain.seedFieldRelative(new Pose2d()))
                .alongWith(shooter.pivotZeroCommand()).andThen(autonChooser.getSelected());
    }

    public void logDrivetrainValues() {
        m_drivetrain.logValues();
        conveyor.logValues();
        infeed.logValues();
    }

    public double getBestYaw(int tagID) {
        Optional<Double> leftYaw = m_leftVision.getTagYaw(tagID);
        Optional<Double> rightYaw = m_rightVision.getTagYaw(tagID);

        int numYaws = 0;
        numYaws += leftYaw.isPresent() ? 1 : 0;
        numYaws += rightYaw.isPresent() ? 1 : 0;

        switch (numYaws) {
            case 1:
                return (leftYaw.isPresent() ? leftYaw : rightYaw).get();
            case 2:
                return (leftYaw.get() + rightYaw.get()) / 2.;
            case 0:
            default:
                return 0.;
        }
    }

    public double getBestDistance(int tagID) {
        Optional<Double> leftDistance = m_leftVision.getTagDistance(tagID);
        Optional<Double> rightDistance = m_rightVision.getTagDistance(tagID);

        int numDistances = 0;
        numDistances += leftDistance.isPresent() ? 1 : 0;
        numDistances += rightDistance.isPresent() ? 1 : 0;

        switch (numDistances) {
            case 1:
                return (leftDistance.isPresent() ? leftDistance : rightDistance).get();
            case 2:
                return (leftDistance.get() + rightDistance.get()) / 2.;
            case 0:
            default:
                return 0.;
        }
    }

    public Optional<EstimatedRobotPose> getBestPose() {
        Pose2d drivetrainPose = m_drivetrain.getState().Pose;

        Optional<EstimatedRobotPose> front = m_rightVision.getCameraResult(drivetrainPose);
        Optional<EstimatedRobotPose> back = m_leftVision.getCameraResult(drivetrainPose);

        int numPoses = 0;

        numPoses += front.isPresent() ? 1 : 0;
        numPoses += back.isPresent() ? 1 : 0;

        Optional<Pose2d> pose = Optional.empty();
        SmartDashboard.putNumber("nuMPoses", numPoses);

        if (numPoses == 1) {
            pose = Optional
                    .of(new Pose2d((front.isEmpty() ? back : front).get().estimatedPose.toPose2d().getTranslation(),
                            drivetrainPose.getRotation()));
        } else if (numPoses == 2) {
            // average the poses
            Pose3d frontP = front.get().estimatedPose;
            Pose3d backP = back.get().estimatedPose;

            Translation3d frontT = frontP.getTranslation();
            Translation3d backT = backP.getTranslation();

            Pose3d dPose = new Pose3d(drivetrainPose);
            Rotation3d bruh = (dPose.getRotation());

            pose = Optional.of(
                    new Pose3d(frontT.plus(backT),
                            bruh.times(2.)).toPose2d().div(2.));
        }

        if (pose.isPresent()) {
            return Optional.of(new EstimatedRobotPose(
                    new Pose3d(pose.get()),
                    (front.isEmpty() ? back : front).get().timestampSeconds,
                    null, null));
        }

        return Optional.empty();
    }

    public void printSTVals() {
        Optional<EstimatedRobotPose> pose = getBestPose();
        if (pose.isEmpty())
            return;

        Transform2d dist = pose.get().estimatedPose.toPose2d().minus(Constants.SPEAKER_TARGET);

        ShooterTableEntry entryPicked = ShooterTable.calcShooterTableEntry(dist.getTranslation().getNorm());

        SmartDashboard.putNumber("Distance", dist.getTranslation().getNorm());
        SmartDashboard.putNumberArray("Shooter Table recommended value",
                new Double[] {
                        entryPicked.angle,
                        entryPicked.leftSpeed,
                        entryPicked.rightSpeed
                });
    }
}
