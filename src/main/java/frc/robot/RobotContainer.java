// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;
import java.util.function.BooleanSupplier;
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
// import frc.robot.subsystems.Fan;
import frc.robot.subsystems.Infeed;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Climber.ClimberPositions;
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

    private static final double FAN_VBUS = 1;
    private static final double SHOOTER_BACKOUT_VBUS = -0.4;

    private static final int OI_DRIVER_CONTROLLER = 0;
    private static final int OI_OPERATOR_CONTROLLER = 1;

    // ======================== //
    /* Controllers & Subsystems */
    // ======================== //
    private final CommandXboxController driverController = new CommandXboxController(OI_DRIVER_CONTROLLER);
    private final CommandXboxController operatorController = new CommandXboxController(OI_OPERATOR_CONTROLLER);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain;
    private final Infeed infeed = new Infeed();
    private final Shooter shooter = new Shooter();
    private final Conveyor conveyor = new Conveyor();
    private final Climber climber = new Climber();
    private final Autons autons;
    private final Pivot pivot = new Pivot();
    // private final Fan fan = new Fan();

    private final Vision rightVision = new Vision("Right_AprilTag_Camera", Vision.RIGHT_ROBOT_TO_CAMERA);
    private final Vision leftVision = new Vision("Left_AprilTag_Camera", Vision.LEFT_ROBOT_TO_CAMERA);

    // ====================== //
    /* Auton & Other Commands */
    // ====================== //
    private final Command magicShootCommand;
    private SendableChooser<Command> autonChooser;

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
        initNamedCommands();

        autons = new Autons(drivetrain, shooter, conveyor, infeed, smartInfeedCommand());

        initAutonChooser();

        magicShootCommand = new RotateToSpeaker(drivetrain).andThen(Commands.runOnce(() -> {
            ShooterTableEntry entry = getBestSTEntry();
            shooter.runEntry(entry, ShotSpeeds.FAST);
            pivot.runToPosition(entry.Angle);
        }, shooter, pivot)).andThen(Commands.waitUntil(shooter.isReadySupplier()))
                .andThen(Commands.waitSeconds(0.5))
                .andThen(conveyor.runXRotations(20)
                        .alongWith(infeed.runInfeedMotorCommand(SLOW_INFEED_VBUS)))
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

        // TODO: change this stuff for shooter table
        NamedCommands.registerCommand("4pinfeed", infeed.runInfeedMotorCommand(INFEED_VBUS)
                .alongWith(conveyor.runMotorCommand(FAST_CONVEYOR_VBUS)).repeatedly());// .withTimeout(1.5));

        NamedCommands.registerCommand("Shoot Note",
                conveyor.runXRotations(20.).alongWith(infeed.runInfeedMotorCommand(SLOW_INFEED_VBUS))
                        .andThen(shooter.stopCommand()));

        NamedCommands.registerCommand("Spit Note", infeed.runInfeedMotorCommand(INFEED_VBUS)
                .alongWith(conveyor.runMotorCommand(FAST_CONVEYOR_VBUS))
                .alongWith(shooter.spinBothCommand(0.15))
                .repeatedly());

        NamedCommands.registerCommand("Prepare Spit", shooter.spinBothCommand(0.15));

        // TODO: use the smart infeed command
        NamedCommands.registerCommand("Limelight Acquire",
                new LimelightAcquire(() -> xLimeAquireLimiter.calculate(0.5),
                        drivetrain)
                        .alongWith(smartInfeedCommand()));

        NamedCommands.registerCommand("Smart Infeed", smartInfeedCommand());

        NamedCommands.registerCommand("farShot", Commands.runOnce(() -> pivot.runToPosition(1)));

        ShooterTableEntry twoHalfEntry = new ShooterTableEntry(Feet.of(0),
                0.05, 1.0);

        // TODO: We may want a command that constantly updates the shooter table and
        // runs the shooter/pivot based on that
        // makes shooting on the move/faster autons much easier

        NamedCommands.registerCommand("Start Shooter",
                runEntryCommand(() -> twoHalfEntry, () -> ShotSpeeds.FAST));

        NamedCommands.registerCommand("Stop Shooter", shooter.stopCommand());
        NamedCommands.registerCommand("Stop Infeed", runBoth(0., 0.));

        NamedCommands.registerCommand("Center Pathfinding Shot", drivetrain
                .pathFindCommand(new Pose2d(4.99, 6.66, new Rotation2d(Units.degreesToRadians(13.3))), 0.75, 0)
                .alongWith(runEntryCommand(() -> twoHalfEntry, () -> ShotSpeeds.FAST).repeatedly()
                        .until(shooterAndPivotReady()))
                .andThen(conveyor.runXRotations(20.).alongWith(infeed.runInfeedMotorCommand(SLOW_INFEED_VBUS)).withTimeout(1.0))
                .andThen(shooter.stopCommand()));

        NamedCommands.registerCommand("Note 3",
                drivetrain.pathFindCommand(new Pose2d(4.67, 6.7, Rotation2d.fromDegrees(-18)), 0.75, 2.5));

        NamedCommands.registerCommand("Note 4",
                drivetrain.pathFindCommand(new Pose2d(3.66, 6.93, Rotation2d.fromDegrees(70.)), 0.75, 0));

        NamedCommands.registerCommand("follow2pchoice",
                new ConditionalCommand(AutoBuilder.followPath(PathPlannerPath.fromPathFile("2pleft")),
                        AutoBuilder.followPath(PathPlannerPath.fromPathFile("2pright")),
                        () -> true));

        NamedCommands.registerCommand("Wait For Shooter", Commands.waitUntil(shooterAndPivotReady()));

        /*
         * Spin up Shooter
         * When shooter ready, feed
         * Stop shooter
         */
        NamedCommands.registerCommand("2.5 Stationary Shot",
                runEntryCommand(() -> twoHalfEntry, () -> ShotSpeeds.FAST)
                        .repeatedly().until(shooterAndPivotReady())
                        .andThen(conveyor.runXRotations(20.).alongWith(infeed.runInfeedMotorCommand(SLOW_INFEED_VBUS)))
                        .andThen(shooter.stopCommand()));

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
        infeed.setDefaultCommand(infeed.runInfeedMotorCommand(0.));

        // ========================= //
        /* DRIVER CONTROLLER */
        // ========================= //

        // ========================= //
        /* Infeed & Conveyor Control */
        // ========================= //

        /* Dumb Infeed */
        driverController.leftTrigger().onTrue(
                infeed.runInfeedMotorCommand(INFEED_VBUS).alongWith(
                        conveyor.runMotorCommand(SLOW_CONVEYOR_VBUS)).repeatedly())
                .onFalse(infeed.runInfeedMotorCommand(0.).alongWith(
                        conveyor.runMotorCommand(0.)));

        /* Conveyance */
        driverController.a()
                .onTrue(conveyor.runXRotations(20.0)
                        .alongWith(infeed.runInfeedMotorCommand(SLOW_INFEED_VBUS)))
                .onFalse(infeed.runInfeedMotorCommand(SLOW_INFEED_VBUS));

        /* Smart Infeed */
        driverController.b().toggleOnTrue(smartInfeedCommand());

        // ======================= //
        /* Shooter Control */
        // ======================= //

        /* Run Shooter */
        driverController.x().and(driverController.povCenter())
                .toggleOnTrue(shooter.runVelocityCommand());

        /* Cycle Down (Trap -> Long) */
        driverController.x().and(driverController.povDown())
                .onTrue(shooter.cycleDownCommand());

        /* Cycle Up (Long -> Trap) */
        driverController.x().and(driverController.povUp())
                .onTrue(shooter.cycleUpCommand());

        // ========================= //
        /* Climber & Zeroing Control */
        // ========================= //

        /* Zero Climber & Pivot */
        // driverController.y().and(driverController.povCenter()).onTrue(pivot.zeroCommand());

        /* Run Climber to "Home" */
        driverController.y().and(driverController.povDown())
                .onTrue(climber.climbCommand());

        /* Run Climber to "Down One" */
        driverController.y().and(driverController.povLeft())
                .onTrue(climber.runToPositionCommand(Climber.ClimberPositions.DOWN_ONE));

        /* Run Climber to "Down One" */
        driverController.y().and(driverController.povRight())
                .onTrue(climber.runToPositionCommand(Climber.ClimberPositions.DOWN_TWO));

        // TODO: change this to a toggle that runs forever until you stop, and add a
        // robot-relative mode
        driverController.leftStick()
                .toggleOnTrue(new LimelightAcquire(() -> xLimeAquireLimiter.calculate(0.5),
                        drivetrain).alongWith(smartInfeedCommand()));

        // driverController.leftStick().toggleOnTrue(new LimelightSquare(true, () -> 0.,
        // () -> 0., drivetrain));
        // driverController.leftStick().toggleOnTrue(new LimelightAcquire(
        // () -> xLimeAquireLimiter.calculate(0.8), drivetrain));

        /* Run Climber to "Ready" */
        driverController.y().and(driverController.povUp())
                .onTrue(pivot.runToPositionCommand(Pivot.CLIMB_POSITION).andThen(
                        Commands.waitUntil(pivot.inPositionSupplier()))
                        .andThen(climber.runToPositionCommand(Climber.ClimberPositions.READY)));

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

        // ==================== //
        /* PIVOT MANUAL CONTROL */
        // ==================== //

        driverController.rightBumper()
                .onTrue(pivot.runOnce(() -> pivot.runToPosition(pivot.getPosition() + 0.2)));
        driverController.leftBumper()
                .onTrue(pivot.runOnce(() -> pivot.runToPosition(pivot.getPosition() - 0.2)));

        // =================== //
        /* OPERATOR CONTROLLER */
        // =================== //

        /* Manual Climber Control */
        operatorController.rightBumper().onTrue(climber.runMotorCommand(CLIMBER_VBUS))
                .onFalse(climber.runMotorCommand(0.0));
        operatorController.leftBumper().onTrue(climber.runMotorCommand(-CLIMBER_VBUS))
                .onFalse(climber.runMotorCommand(0.0));

        operatorController.a().toggleOnTrue(new RotateToSpeaker(drivetrain));

        operatorController.x()
                .toggleOnTrue(Commands.runOnce(() -> {
                    ShooterTableEntry entry = getBestSTEntry();
                    shooter.runEntry(entry, ShotSpeeds.FAST);
                    pivot.runToPosition(entry.Angle);
                }, shooter, pivot));

        operatorController.b().onTrue(magicShootCommand);

        operatorController.y().onTrue(pivot.runToPositionCommand(Pivot.TRAP_POSITION));
        // TODO: Go to trap also runs a shuffle routine in conveyor

        operatorController.leftStick().toggleOnTrue(drivetrain.pathFindCommand(Constants.AMP_TARGET, .5, 0));

        operatorController.rightStick().toggleOnTrue(drivetrain.pathFindCommand(Constants.LEFT_TRAP_Target, .2, 0)
                .andThen(new WaitCommand(2))
                .andThen(pivot.runToPositionCommand(Pivot.TRAP_POSITION))
                .alongWith(new WaitCommand(1))
                .andThen(shooter.run(() -> {
                    shooter.setLeftToVel(1300);
                    shooter.setRightToVel(1300);
                }).withTimeout(4)
                        .andThen(shooter.runOnce(() -> {
                            shooter.setLeftToVel(0);
                            shooter.setRightToVel(0);
                        }))
                        .alongWith(new WaitCommand(2)
                                .andThen(conveyor.runXRotations(20))))
                .andThen(pivot.runToPositionCommand(.5)));

        operatorController.back().onTrue(Commands.runOnce(() -> getBestSTEntry()));

        operatorController.start().onTrue(climber.runToPositionCommand(ClimberPositions.HOME));

        operatorController.povDown().whileTrue(
                shooter.startEnd(
                        () -> {
                            shooter.spinMotorRight(1.0);
                            shooter.spinMotorLeft(1.0);
                        },
                        shooter::stop));

        // operatorController.povDown().onTrue(fan.runMotorCommand(FAN_VBUS)).onFalse(fan.runMotorCommand(0.0));

        if (Utils.isSimulation()) {
            drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
        }

        drivetrain.registerTelemetry(logger::telemeterize);

    }

    // =========================================== //
    /* Additional Commands, Getters, and Utilities */
    // =========================================== //

    /* Smart Infeed Command Generator */
    private Command smartInfeedCommand() {
        return runBoth(SLOW_CONVEYOR_VBUS, INFEED_VBUS)
                .repeatedly().until(conveyor.hasInfedSupplier())
                .andThen(runBoth(0., 0.)
                        .repeatedly().withTimeout(0.1))
                .andThen(shooter.spinMotorLeftCommand(SHOOTER_BACKOUT_VBUS).repeatedly()
                        .raceWith(conveyor.runXRotations(-4.0).withTimeout(0.5) // -1.5
                                .alongWith(infeed.runInfeedMotorCommand(0.))))
                .andThen(shooter.spinMotorLeftCommand(0.));
    }

    /* Run a Shooter Table Entry */
    private Command runEntryCommand(Supplier<ShooterTableEntry> entry, Supplier<ShotSpeeds> speed) {
        return shooter.runEntryCommand(entry, speed)
                .alongWith(pivot.runToPositionCommand(entry.get().Angle));
    }

    /* Shooter & Pivot Both Ready */
    private BooleanSupplier shooterAndPivotReady() {
        return () -> shooter.isReady() && pivot.inPosition();
    }

    /* Run both Conveyor and Infeed */
    private Command runBoth(double conveyorVbus, double infeedVbus) {
        return infeed.runInfeedMotorCommand(infeedVbus).alongWith(conveyor.runMotorCommand(conveyorVbus));
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

        SmartDashboard.putNumber("ST Angle", entryPicked.Angle);
        SmartDashboard.putNumber("ST Left", entryPicked.percent);

        return entryPicked;
    }
}
