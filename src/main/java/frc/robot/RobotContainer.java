// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.RotateToSpeaker;
import frc.robot.commands.Autons;
import frc.robot.commands.Autons.Notes;
import frc.robot.commands.Autons.StartPoses;
import frc.robot.commands.vision.LimelightAcquire;
import frc.robot.generated.TunerConstants;

import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Fan;
// import frc.robot.subsystems.Fan;
import frc.robot.subsystems.Infeed;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

import frc.robot.subsystems.Shooter.ShotSpeeds;
import frc.robot.utils.ShooterTable;
import frc.robot.utils.ShooterTable.ShooterTableEntry;

import static edu.wpi.first.units.Units.*;

public class RobotContainer {
    // =============================================== //
    /* Magic numbers, Vbus constants, and OI constants */
    // =============================================== //
    private static final double CLIMBER_VBUS = 0.05;
    private static final double INFEED_VBUS = 0.8;
    private static final double SLOW_INFEED_VBUS = 0.5;

    private static final double PIVOT_VBUS = 0.15;
    private static final double SLOW_CONVEYOR_VBUS = 0.5;
    private static final double FAST_CONVEYOR_VBUS = 0.85;

    private static final double FAN_VBUS = 1.d;
    private static final double SHOOTER_BACKOUT_VBUS = -0.4;

    private static final int OI_DRIVER_CONTROLLER = 0;
    private static final int OI_OPERATOR_CONTROLLER = 1;
    private static final int OI_EMERGENCY_CONTROLLER = 2;

    // ======================== //
    /* Controllers & Subsystems */
    // ======================== //
    private final CommandXboxController driverController = new CommandXboxController(OI_DRIVER_CONTROLLER);
    private final CommandXboxController operatorController = new CommandXboxController(OI_OPERATOR_CONTROLLER);
    private final CommandXboxController emergencyController = new CommandXboxController(OI_EMERGENCY_CONTROLLER);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain;
    private final Infeed infeed = new Infeed();
    private final Shooter shooter = new Shooter();
    private final Conveyor conveyor = new Conveyor();
    private final Climber climber = new Climber();
    private final Autons autons;
    private final Pivot pivot = new Pivot();
    private final Fan m_fan = new Fan();

    private final Vision rightVision = new Vision("Right_AprilTag_Camera", Vision.RIGHT_ROBOT_TO_CAMERA);
    private final Vision leftVision = new Vision("Left_AprilTag_Camera", Vision.LEFT_ROBOT_TO_CAMERA);

    // ====================== //
    /* Auton & Other Commands */
    // ====================== //
    private final Command smartInfeedCommand, magicShootCommand;
    private SendableChooser<Command> autonChooser;

    private boolean isOutfeedMode = false;

    // ====================================================== //
    /* Drivetrain Constants, Magic numbers, and Slew Limiters */
    // ====================================================== //
    private final SlewRateLimiter xLimiter = new SlewRateLimiter(4.);
    private final SlewRateLimiter yLimiter = new SlewRateLimiter(4.);
    private final SlewRateLimiter thetaLimiter = new SlewRateLimiter(4.);
    private final SlewRateLimiter xLimeAquireLimiter = new SlewRateLimiter(4.);

    private static final double MAX_SPEED = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top
                                                                               // speed
    private static final double MAX_ANGULAR_SPEED = 1.5 * Math.PI; // 3/4 of a rotation per second max angular
                                                                   // velocity
    private static final double BASE_SPEED = 0.25;

    // ======================== //
    /* Swerve Control & Logging */
    // ======================== //
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MAX_SPEED * 0.05).withRotationalDeadband(MAX_ANGULAR_SPEED * 0.05) // Add a 50%
                                                                                             // deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                                     // driving in open loop
    private final Telemetry logger = new Telemetry(MAX_SPEED);

    public RobotContainer() {

        // TODO: Failsafe timer based on Infeed ToF
        smartInfeedCommand = infeed.runMotorCommand(INFEED_VBUS)
                .alongWith(conveyor.runMotorCommand(SLOW_CONVEYOR_VBUS))
                .repeatedly().until(conveyor.hasInfedSupplier())
                .andThen(infeed.runMotorCommand(0.).alongWith(conveyor.runMotorCommand(0.))
                        .repeatedly().withTimeout(0.1))
                .andThen(shooter.spinMotorLeftCommand(SHOOTER_BACKOUT_VBUS).repeatedly()
                        .raceWith(conveyor.runXRotations(-4.0).withTimeout(0.5) // -1.5
                                .alongWith(infeed.runMotorCommand(0.))))
                .andThen(shooter.spinMotorLeftCommand(0.));// .withTimeout(3);

        initNamedCommands();

        autons = new Autons(drivetrain, shooter, conveyor, infeed, smartInfeedCommand);

        initAutonChooser();

        magicShootCommand = new RotateToSpeaker(drivetrain).andThen(Commands.runOnce(() -> {
            ShooterTableEntry entry = getBestSTEntry();
            shooter.runEntry(entry, ShotSpeeds.FAST);
            pivot.runToPosition(entry.angle);
        }, shooter, pivot)).andThen(Commands.waitUntil(shooter.isReady()))
                .andThen(Commands.waitSeconds(0.5))
                .andThen(conveyor.runXRotations(10)
                        .alongWith(infeed.runMotorCommand(SLOW_INFEED_VBUS)))
                .andThen(Commands.waitSeconds(0.2))
                .andThen(shooter.stopCommand())
                .andThen(pivot.runToPositionCommand(Pivot.HOLD_POSITION));

        configureBindings();
    }

    // ====================== //
    /* Auton & Named Commands */
    // ====================== //
    private void initAutonChooser() {
        autonChooser = AutoBuilder.buildAutoChooser();
        autonChooser.addOption("2pdyn", autons.twoPieceAutonDynamic(StartPoses.TOP, 1, Notes.ONE, Notes.TWO));
        SmartDashboard.putData("Auto Chooser", autonChooser);
    }

    private void initNamedCommands() {
        NamedCommands.registerCommand("pivotZero", pivot.zeroCommand());

        NamedCommands.registerCommand("zeroApril", new InstantCommand(() -> {
            Optional<EstimatedRobotPose> poseOpt = getBestPose();
            if (poseOpt.isEmpty())
                return;

            drivetrain.seedFieldRelative(poseOpt.get().estimatedPose.toPose2d());
        }, leftVision, rightVision));

        NamedCommands.registerCommand("llAquire",
                new LimelightAcquire(() -> xLimeAquireLimiter.calculate(0.8), drivetrain));

        // TODO: change this stuff for shootertable
        NamedCommands.registerCommand("runShooter", shooter.runVelocityCommand());
        NamedCommands.registerCommand("4pinfeed", infeed.runMotorCommand(INFEED_VBUS)
                .alongWith(conveyor.runMotorCommand(FAST_CONVEYOR_VBUS)).repeatedly());// .withTimeout(1.5));

        NamedCommands.registerCommand("runThru", infeed.runMotorCommand(INFEED_VBUS)
                .alongWith(conveyor.runMotorCommand(FAST_CONVEYOR_VBUS))
                .alongWith(shooter.spinBothCommand(0.15))
                .repeatedly());

        NamedCommands.registerCommand("LLAquire",
                new LimelightAcquire(() -> xLimeAquireLimiter.calculate(0.5),
                        drivetrain)
                        .alongWith(
                                infeed.runMotorCommand(INFEED_VBUS)
                                        .alongWith(conveyor.runMotorCommand(SLOW_CONVEYOR_VBUS))
                                        .repeatedly().until(conveyor.hasInfedSupplier())
                                        .andThen(
                                                infeed.runMotorCommand(0.).alongWith(conveyor.runMotorCommand(0.))
                                                        .repeatedly().withTimeout(0.1))
                                        .andThen(shooter.spinMotorLeftCommand(SHOOTER_BACKOUT_VBUS).repeatedly()
                                                .raceWith(conveyor.runXRotations(-4.0).withTimeout(0.5) // -1.5
                                                        .alongWith(infeed.runMotorCommand(0.))))
                                        .andThen(shooter.spinMotorLeftCommand(0.))// .withTimeout(3);
                        ));

        NamedCommands.registerCommand("smartInfeed", infeed.runMotorCommand(INFEED_VBUS)
                .alongWith(conveyor.runMotorCommand(SLOW_CONVEYOR_VBUS))
                .repeatedly().until(conveyor.hasInfedSupplier())
                .andThen(infeed.runMotorCommand(0.).alongWith(conveyor.runMotorCommand(0.))
                        .repeatedly().withTimeout(0.1))
                .andThen(shooter.spinMotorLeftCommand(SHOOTER_BACKOUT_VBUS).repeatedly()
                        .raceWith(conveyor.runXRotations(-4.0).withTimeout(0.5) // -1.5
                                .alongWith(infeed.runMotorCommand(0.))))
                .andThen(shooter.spinMotorLeftCommand(0.)));// .withTimeout(3);
        NamedCommands.registerCommand("farShot", Commands.runOnce(() -> pivot.runToPosition(1)));

        ShooterTableEntry aentry = new ShooterTableEntry(Feet.of(0),
                3, 1.0);
        NamedCommands.registerCommand("startShooter",
                shooter.runEntryCommand(() -> aentry, () -> ShotSpeeds.FAST)
                        .alongWith(pivot.runToPositionCommand(
                                aentry.angle)));

        NamedCommands.registerCommand("2.5StartShooter", shooter.runEntryCommand(() -> aentry, () -> ShotSpeeds.FAST));

        NamedCommands.registerCommand("stopShooter", shooter.stopCommand());

        NamedCommands.registerCommand("goTo2.5Shoot", drivetrain
                .pathFindCommand(new Pose2d(4.99, 6.66, new Rotation2d(Units.degreesToRadians(13.3))), 0.5, 0));

        NamedCommands.registerCommand("follow2pchoice",
                new ConditionalCommand(AutoBuilder.followPath(PathPlannerPath.fromPathFile("2pleft")),
                        AutoBuilder.followPath(PathPlannerPath.fromPathFile("2pright")),
                        () -> true));
    }

    // =========================== //
    /* Bindings & Default Commands */
    // =========================== //
    private void configureBindings() {

        // TODO: Add reverse infeed in case of jams so driver can spit out note and
        // retry
        // TODO: Buttons should NOT be toggles. Commands should only be running while
        // buttons are being held

        // ================ //
        /* Default Commands */
        // ================ //

        drivetrain.setDefaultCommand(
                drivetrain.applyRequest(() -> drive
                        .withVelocityX(scaleDriverController(-driverController.getLeftY(),
                                xLimiter,
                                BASE_SPEED) * MAX_SPEED)
                        .withVelocityY(scaleDriverController(-driverController.getLeftX(),
                                yLimiter,
                                BASE_SPEED) * MAX_SPEED)
                        .withRotationalRate(
                                scaleDriverController(-driverController.getRightX(),
                                        thetaLimiter, BASE_SPEED) *
                                        MAX_SPEED)));

        conveyor.setDefaultCommand(conveyor.runMotorCommand(0.));
        infeed.setDefaultCommand(infeed.runMotorCommand(0.));
        m_fan.setDefaultCommand(m_fan.stopCommand());

        // ================= //
        /* DRIVER CONTROLLER */
        // ================= //

        // ========================= //
        /* Infeed & Conveyor Control */
        // ========================= //

        /* Dumb Infeed */
        driverController.leftTrigger().onTrue(
                infeed.runMotorCommand(INFEED_VBUS).alongWith(
                        conveyor.runMotorCommand(SLOW_CONVEYOR_VBUS)).repeatedly())
                .onFalse(infeed.runMotorCommand(0.).alongWith(
                        conveyor.runMotorCommand(0.)));

        /* Smart Infeed */
        driverController.leftBumper().toggleOnTrue(smartInfeedCommand);

        // ========================== //
        /* Drivetain & Vision Control */
        // ========================== //

        /* Reset Field-Centric Heading */
        driverController.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative(new Pose2d())));

        /* Add Vision Measurement */
        driverController.back().and(driverController.povCenter())
                .onTrue(drivetrain.addMeasurementCommand(() -> getBestPose()));

        /* Reset Pose & Test ShooterTable */
        driverController.back().and(driverController.povDown()).onTrue(Commands.runOnce(() -> {
            var pose = getBestPose();
            if (pose.isPresent())
                drivetrain.seedFieldRelative(pose.get().estimatedPose.toPose2d());

            getBestSTEntry();
        }));

        // TODO: Limelight squaring
        // toggle that runs forever, goes to robot relative mode, no infeed/whatever
        // control
        driverController.leftStick().onTrue(Commands.none());

        // =================== //
        /* OPERATOR CONTROLLER */
        // =================== //

        // ======================= //
        /* Shooter Control */
        // ======================= //

        /* Spin Up Shooter */
        operatorController.leftBumper()
                .onTrue(Commands.runOnce(() -> {
                    ShooterTableEntry entry = getBestSTEntry();
                    shooter.runEntry(entry, ShotSpeeds.FAST);
                    pivot.runToPosition(entry.angle);
                }, shooter, pivot))
                .onFalse(shooter.stopCommand());

        /* Convey Note */
        operatorController.rightBumper()
                .whileTrue(runBoth(SLOW_CONVEYOR_VBUS, SLOW_INFEED_VBUS).repeatedly());

        /* Magic Shoot */
        operatorController.b().onTrue(magicShootCommand);

        // TODO: bind these to ST index up/down

        /* Shooter Table Index Down */
        operatorController.leftTrigger(0.5).onTrue(Commands.none());

        /* Shooter Table Index Up */
        operatorController.rightTrigger(0.5).onTrue(Commands.none());

        // ========================= //
        /* Climber & Zeroing Control */
        // ========================= //

        // TODO: get climber good

        /* Zero Climber & Pivot */
        operatorController.start().onTrue(pivot.zeroCommand());

        /* Run Climber to "Home" */
        operatorController.povDown().onTrue(climber.climbCommand());

        /* Run Climber to "Down One" */
        operatorController.povLeft().onTrue(climber.runToPositionCommand(Climber.ClimberPositions.DOWN_ONE));

        /* Run Climber to "Down One" */
        operatorController.povRight().onTrue(climber.runToPositionCommand(Climber.ClimberPositions.DOWN_TWO));

        /* Run Climber to "Ready" */
        operatorController.povUp().onTrue(pivot.runToClimbCommand().andThen(
                Commands.waitUntil(pivot.inPositionSupplier())));
        // climber.runToPositionCommand(Climber.ClimberPositions.READY)));

        // ================ //
        /* Amp & Trap Magic */
        // ================ //

        // TODO: make amp magic
        operatorController.b().toggleOnTrue(drivetrain.pathFindCommand(Constants.AMP_TARGET, .5, 0));

        operatorController.y().toggleOnTrue(drivetrain.pathFindCommand(Constants.LEFT_TRAP_TARGET, .2, 0)
                .andThen(shooter.setSlotCommand(Shooter.Slots.TRAP))
                .andThen(new WaitCommand(2))
                .andThen(pivot.runToTrapCommand())
                .andThen(m_fan.runMotorCommand(FAN_VBUS))
                .andThen(shooter.runShotCommand(Shooter.ShotSpeeds.TRAP).repeatedly())
                .until(shooter.isReady()).withTimeout(4)
                .andThen(shooter.stopCommand())
                .alongWith(new WaitCommand(2)
                        .andThen(conveyor.runXRotations(20)))
                .andThen(pivot.runToHomeCommand()));

        // ==================== //
        /* EMERGENCY CONTROLLER */
        // ==================== //

        // ==================== //
        /* Manual Pivot Control */
        // ==================== //

        /* Bump Pivot Up */
        emergencyController.rightBumper()
                .onTrue(pivot.runOnce(() -> pivot.runToPosition(pivot.getPosition() + 0.2)));

        /* Bump Pivot Down */
        emergencyController.leftBumper()
                .onTrue(pivot.runOnce(() -> pivot.runToPosition(pivot.getPosition() - 0.2)));

        // ============================== //
        /* Manual Climber/Outfeed Control */
        // ============================== //

        /* Climber Up or Full Outfeed */
        emergencyController.rightTrigger(0.2).whileTrue(
                climber.runMotorCommand(CLIMBER_VBUS))
                .onFalse(climber.runMotorCommand(0.0));

        /* Climber Down or Conveyor Outfeed */
        emergencyController.leftTrigger(0.2).whileTrue(
                climber.runMotorCommand(CLIMBER_VBUS))
                .onFalse(climber.runMotorCommand(0.0));

        // ==== //
        /* Misc */
        // ==== //

        /* TrapStar 5000 */
        emergencyController.y().onTrue(m_fan.runMotorCommand(FAN_VBUS));

        /* ST test */
        emergencyController.back().onTrue(Commands.runOnce(() -> getBestSTEntry()));

        /* Full Outfeed: left Y */
        emergencyController.axisGreaterThan(XboxController.Axis.kLeftY.value, 0.2)
                .or(emergencyController.axisLessThan(XboxController.Axis.kLeftY.value, -0.2))
                .whileTrue(
                        runThree(
                                () -> -emergencyController.getLeftY(),
                                () -> -emergencyController.getLeftY(),
                                () -> emergencyController.getLeftY() > 0. ? -emergencyController.getLeftY() : 0.))
                .onFalse(shooter.stopCommand());

        /* Cool Outfeed: right Y */
        emergencyController.axisLessThan(XboxController.Axis.kRightY.value, -0.2)
                .whileTrue(
                        runThree(
                                () -> emergencyController.getRightY(),
                                () -> -emergencyController.getRightY(),
                                () -> emergencyController.getRightY()))
                .onFalse(shooter.stopCommand());

        if (Utils.isSimulation()) {
            drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
        }

        drivetrain.registerTelemetry(logger::telemeterize);

    }

    // =========================================== //
    /* Additional Commands, Getters, and Utilities */
    // =========================================== //

    /* Run both Conveyor and Infeed */
    private Command runBoth(double conveyorVbus, double infeedVbus) {
        return infeed.runMotorCommand(infeedVbus).alongWith(conveyor.runMotorCommand(conveyorVbus));
    }

    /* Run both Conveyor and Infeed */
    private Command runThree(Supplier<Double> conveyorVbus, Supplier<Double> infeedVbus, Supplier<Double> shooterVbus) {
        return Commands.run(() -> {
            infeed.runMotor(infeedVbus.get());
            conveyor.runMotor(conveyorVbus.get());
            shooter.spinMotorRight(0.3 * shooterVbus.get());
            shooter.spinMotorLeft(0.3 * shooterVbus.get());
        }, infeed, conveyor);
    }

    /* Auton Command */
    public Command getAutonomousCommand() {
        return new InstantCommand(() -> drivetrain.seedFieldRelative(new Pose2d()))
                .andThen(NamedCommands.getCommand("zeroApril"))
                .andThen(autonChooser.getSelected());
    }

    /* Zeroing Command */
    public Command zeroCommand() {
        // return climber.zeroCommand().andThen(pivot.zeroCommand());
        return pivot.zeroCommand();
    }

    /* Asynchronous Zero */
    public void zero() {
        zeroCommand().schedule();
    }

    /* Joystick Scaling */
    private double scaleDriverController(double controllerInput, SlewRateLimiter limiter, double baseSpeedPercent) {
        return limiter.calculate(
                controllerInput * (baseSpeedPercent
                        + driverController.getRightTriggerAxis() * (1 - baseSpeedPercent)));
    }

    // ======= //
    /* Logging */
    // ======= //
    public void logValues() {
        drivetrain.logValues();
        conveyor.logValues();
        infeed.logValues();
        shooter.logValues();
        climber.logValues();
        pivot.logValues();
        // m_fan.logValues();
    }

    // ================ //
    /* Vision Utilities */
    // ================ //

    /* Return approx. 3d pose */
    public Optional<EstimatedRobotPose> getBestPose() {
        Pose2d drivetrainPose = drivetrain.getState().Pose;

        Optional<EstimatedRobotPose> front = rightVision.getCameraResult(drivetrainPose);
        Optional<EstimatedRobotPose> back = leftVision.getCameraResult(drivetrainPose);

        int numPoses = 0;

        numPoses += front.isPresent() ? 1 : 0;
        numPoses += back.isPresent() ? 1 : 0;

        Optional<Pose2d> pose = Optional.empty();
        SmartDashboard.putNumber("nuMPoses", numPoses);

        if (numPoses == 1) {
            pose = Optional
                    .of(new Pose2d((front.isEmpty() ? back : front).get().estimatedPose.toPose2d()
                            .getTranslation(),
                            drivetrainPose.getRotation()));
        } else if (numPoses == 2) {
            // average the poses
            Pose3d frontP = front.get().estimatedPose;
            Pose3d backP = back.get().estimatedPose;

            Translation2d frontT = frontP.getTranslation().toTranslation2d();
            Translation2d backT = backP.getTranslation().toTranslation2d();

            pose = Optional.of(
                    new Pose2d(frontT.plus(backT).div(2.),
                            drivetrainPose.getRotation()));
        }

        if (pose.isPresent()) {
            return Optional.of(new EstimatedRobotPose(
                    new Pose3d(pose.get()
                            .plus(new Transform2d(Units.inchesToMeters(9.), 0.,
                                    new Rotation2d()))),
                    (front.isEmpty() ? back : front).get().timestampSeconds,
                    null, null));
        }

        return Optional.empty();
    }

    /* Test Shooter Table */
    private ShooterTableEntry getBestSTEntry() {
        Pose2d pose = drivetrain.getState().Pose;

        Transform2d dist = pose.minus(Constants.SPEAKER_DISTANCE_TARGET);
        Translation2d translation = dist.getTranslation(); // get everything in feet

        ShooterTableEntry entryPicked = ShooterTable
                .calcShooterTableEntry(Meters.of(translation.getNorm()));

        SmartDashboard.putNumber("Distance", Units.metersToFeet(translation.getNorm()));

        SmartDashboard.putNumber("ST Angle", entryPicked.angle);
        SmartDashboard.putNumber("ST Left", entryPicked.percent);

        return entryPicked;
    }
}
