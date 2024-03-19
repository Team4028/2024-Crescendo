// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Meters;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.SwerveDriveBrake;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.RotateToSpeaker;
import frc.robot.commands.Autons;
import frc.robot.commands.Autons.Notes;
import frc.robot.commands.Autons.StartPoses;
import frc.robot.commands.vision.LimelightAcquire;
import frc.robot.commands.vision.LimelightSquare;
import frc.robot.commands.vision.ShooterAlign;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Fan;
import frc.robot.subsystems.Infeed;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShotSpeeds;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Whippy;
import frc.robot.subsystems.Climber.ClimberPositions;
import frc.robot.subsystems.Climber;
// import frc.robot.subsystems.Climber.ClimberPositions;
import frc.robot.utils.DashboardStore;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.ShooterTable;
import frc.robot.utils.ShooterTable.ShooterTableEntry;

public class RobotContainer {
    // =============================================== //
    /* Magic numbers, Vbus constants, and OI constants */
    // =============================================== //
    private static final double CLIMBER_VBUS = 0.2;
    private static final double INFEED_VBUS = 0.8;
    private static final double SLOW_INFEED_VBUS = 0.5;

    private static final double PIVOT_VBUS = 0.15;
    private static final double SLOW_CONVEYOR_VBUS = 0.5;
    private static final double FAST_CONVEYOR_VBUS = 0.85;

    private static final double FAN_VBUS = 1.;
    private static final double FAN_PIVOT_VBUS = 0.2;

    private static final double SHOOTER_BACKOUT_VBUS = -0.4;
    private static final double WHIPPY_VBUS = 0.2;

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
    private final Whippy whippy = new Whippy();

    private final Vision rightVision = new Vision("Right_AprilTag_Camera", Vision.RIGHT_ROBOT_TO_CAMERA);
    private final Vision leftVision = new Vision("Left_AprilTag_Camera", Vision.LEFT_ROBOT_TO_CAMERA);
    private final Vision trapVision = new Vision("Trap_AprilTag_Camera", Vision.SHOOTER_ROBOT_TO_CAMERA);

    // ====================== //
    /* Auton & Other Commands */
    // ====================== //
    private final Command magicShootCommand, ampPrep;
    private SendableChooser<Command> autonChooser;

    // ====================================================== //
    /* Drivetrain Constants, Magic numbers, and Slew Limiters */
    // ====================================================== //
    private final SlewRateLimiter xLimiter = new SlewRateLimiter(4.);
    private final SlewRateLimiter yLimiter = new SlewRateLimiter(4.);
    private final SlewRateLimiter thetaLimiter = new SlewRateLimiter(4.);
    private final SlewRateLimiter xLimeAquireLimiter = new SlewRateLimiter(4.);
    private final SlewRateLimiter yLimeAquireLimiter = new SlewRateLimiter(4.);

    private static final double MAX_SPEED = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top
                                                                               // speed
    private static final double MAX_ANGULAR_SPEED = 4 * Math.PI; // 2rps

    private static final double BASE_SPEED = 0.25;
    private static final double SLOW_SPEED = 0.07;

    private double currentSpeed = BASE_SPEED;

    private enum SnapDirection {
        None(Double.NaN),
        Forward(0),
        Left(90),
        Back(180),
        Right(270),
        LeftTrap(-60.),
        RightTrap(60.);

        public double Angle;

        private SnapDirection(double angle) {
            Angle = angle;
        }
    }

    private static double MAX_INDEX = 27.;
    private static double MIN_INDEX = 4.2;

    private final LinkedHashMap<Double, String> indexMap = new LinkedHashMap<>();

    private double currentIndex = MIN_INDEX;
    private double manualIndex = MIN_INDEX;
    private int presetIndex = 0;
    private boolean useManual = false;

    private boolean enableClimber = false;

    // ======================== //
    /* Swerve Control & Logging */
    // ======================== //
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MAX_SPEED * 0.02).withRotationalDeadband(MAX_ANGULAR_SPEED * 0.01)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.RobotCentric robotRelativeDrive = new SwerveRequest.RobotCentric()
            .withDeadband(MAX_SPEED * 0.02).withRotationalDeadband(MAX_ANGULAR_SPEED * 0.01)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MAX_SPEED);

    private final SwerveRequest.SwerveDriveBrake xDrive = new SwerveDriveBrake();
    private final SwerveRequest.FieldCentricFacingAngle snapDrive = new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(MAX_SPEED * 0.035)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    /* LL */
    private static final double SHOOTER_CAM_PITCH = Units.degreesToRadians(31 .0); // 32. //-4.65 ??
    private static final double SHOOTER_CAM_HEIGHT = Units.inchesToMeters(12.375); // 12.375

    public RobotContainer() {
        trapVision.setPipeline(Vision.SHOOTER_PIPELINE_INDEX);
                
        snapDrive.HeadingController = new PhoenixPIDController(3., 0., 0.);
        snapDrive.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

        /* Init Index Map */
        indexMap.put(4.2, "Speaker");
        indexMap.put(6.0, "Line");
        indexMap.put(10.0, "Protected");
        indexMap.put(12.0, "Midfield");
        indexMap.put(13.0, "Chain");
        indexMap.put(16.0, "Truss");
        indexMap.put(19.0, "Wing");
        // indexMap.put(22.0, "Neutral");

        /* Dashboard */
        DashboardStore.add("Shooter Table Index", () -> currentIndex);
        DashboardStore.add("Shooter Table Name",
                () -> indexMap.containsKey(currentIndex) ? indexMap.get(currentIndex) : "Manual");

        // this is weird
        DashboardStore.add("Snapped", () -> drivetrain.getCurrentRequest().getClass().equals(snapDrive.getClass()));
        DashboardStore.add("Robot Relative",
                () -> drivetrain.getCurrentRequest().getClass().equals(robotRelativeDrive.getClass()));

        DashboardStore.add("Manual Indexing", () -> useManual);
        DashboardStore.add("Climber Enabled", () -> enableClimber);

        // DashboardStore.add("Distance To Speaker", () -> {
        // int tagID = DriverStation.getAlliance().isPresent()
        // && DriverStation.getAlliance().get() == Alliance.Red ? 4 : 7;

        // Optional<Double> dist = trapVision.getTagDistance(tagID);

        // if (dist.isPresent()) {
        // return Units.metersToFeet(dist.get());
        // }

        // return 0.;
        // });

        // TODO: Failsafe timer based on Infeed ToF
        initNamedCommands();

        autons = new Autons(drivetrain, shooter, conveyor, infeed,
                smartInfeedCommand());

        initAutonChooser();

        magicShootCommand = Commands.runOnce(() -> {
            var pose = getBestPose();
            if (pose.isPresent())
                drivetrain.seedFieldRelative(pose.get().estimatedPose.toPose2d());

            getBestSTEntry();
        }).andThen(Commands.waitSeconds(0.1)).andThen(new RotateToSpeaker(drivetrain).andThen(Commands.runOnce(() -> {
            ShooterTableEntry entry = getBestSTEntry();
            shooter.runEntry(entry, ShotSpeeds.FAST);
            pivot.runToPosition(entry.Angle);
        }, shooter, pivot)).andThen(Commands.waitUntil(shooter.isReadySupplier()))
                .andThen(Commands.waitSeconds(0.5))
                .andThen(conveyCommand())
                .andThen(Commands.waitSeconds(0.2))
                .finallyDo(this::stopAll));

        ampPrep = pivot.runToClimbCommand()
                .alongWith(whippy.whippyWheelsCommand(WHIPPY_VBUS))
                .alongWith(shooter.setSlotCommand(Shooter.Slots.AMP)
                        .andThen(shooter.runShotCommand(ShotSpeeds.AMP)));

        configureBindings();
    }

    // ====================== //
    /* Auton & Named Commands */
    // ====================== //
    private void initAutonChooser() {
        autonChooser = AutoBuilder.buildAutoChooser();

        autonChooser.addOption("Zero", zeroCommand());

        autonChooser.addOption("Shoot Note", zeroCommand()
                .andThen(runEntryCommand(() -> ShooterTable.calcShooterTableEntry(Feet.of(4.2)),
                        () -> ShotSpeeds.FAST))
                .andThen(Commands.waitUntil(shooterAndPivotReady()))
                .andThen(conveyCommand())
                .andThen(Commands.waitSeconds(1.0))
                .andThen(pivot.runToHomeCommand())
                .andThen(shooter.stopCommand()));

        autonChooser.addOption("2pdyn", autons.twoPieceAutonDynamic(StartPoses.TOP,
                1, Notes.ONE, Notes.TWO));
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

        NamedCommands.registerCommand("Go to 4 piece path",
                drivetrain.pathFindCommand(PathPlannerAuto.getStaringPoseFromAutoFile("4 Piece Simple"), 1., 0.));

        // TODO: change this stuff for shooter table
        NamedCommands.registerCommand("4pinfeed", infeed.runMotorCommand(INFEED_VBUS)
                .alongWith(conveyor.runMotorCommand(FAST_CONVEYOR_VBUS)).repeatedly());// .withTimeout(1.5));

        NamedCommands.registerCommand("Shoot Note",
                conveyCommand().andThen(shooter.stopCommand()));

        NamedCommands.registerCommand("Spit Note", infeed.runMotorCommand(INFEED_VBUS)
                .alongWith(conveyor.runMotorCommand(FAST_CONVEYOR_VBUS))
                .alongWith(shooter.spinBothCommand(0.15))
                .repeatedly());

        NamedCommands.registerCommand("Run Pivot To Home", pivot.runToHomeCommand());

        NamedCommands.registerCommand("Prepare Spit", shooter.spinBothCommand(0.15));

        NamedCommands.registerCommand("Limelight Acquire",
                new LimelightAcquire(() -> xLimeAquireLimiter.calculate(0.5),
                        drivetrain)
                        .alongWith(smartInfeedCommand()));

        NamedCommands.registerCommand("Smart Infeed", smartInfeedCommand());

        NamedCommands.registerCommand("Fix Note",
                runBoth(FAST_CONVEYOR_VBUS, INFEED_VBUS).repeatedly().withTimeout(0.25).andThen(
                        shooter.spinMotorRightCommand(SHOOTER_BACKOUT_VBUS).alongWith(conveyor.runXRotations(-2.0)))
                        .andThen(Commands.waitSeconds(0.4)));

        NamedCommands.registerCommand("Dumb Infeed",
                runBoth(SLOW_CONVEYOR_VBUS, INFEED_VBUS).repeatedly().withTimeout(.25));

        NamedCommands.registerCommand("Run Back", shooter.spinMotorLeftCommand(SHOOTER_BACKOUT_VBUS).repeatedly()
                .raceWith(conveyor.runXRotations(-2.5).withTimeout(0.2) // -1.5
                        .alongWith(infeed.runMotorCommand(0.)))
                .andThen(shooter.stopCommand()));

        ShooterTableEntry twoHalfEntry = new ShooterTableEntry(Feet.of(0),
                10.2, 1.0); // TODO: fix

        // TODO: We may want a command that constantly updates the shooter table and
        // runs the shooter/pivot based on that
        // makes shooting on the move/faster autons much easier

        NamedCommands.registerCommand("Start Shooter",
                runEntryCommand(() -> twoHalfEntry, () -> ShotSpeeds.FAST));

        ShooterTableEntry fourP = new ShooterTableEntry(Feet.of(0), 0.0, 0.85);
        NamedCommands.registerCommand("Start Shooter No Pivot",
                shooter.runEntryCommand(() -> fourP, () -> ShotSpeeds.FAST));

        // ShooterTableEntry fourEntry = new ShooterTableEntry(Feet.of(0), 12, 1.0);
        NamedCommands.registerCommand("4 Piece Shooter Pivot", pivot.runToPositionCommand(15.));

        NamedCommands.registerCommand("4 Piece First Shooter Pivot", pivot.runToPositionCommand(11.6));

        NamedCommands.registerCommand("4 Piece Mid Shooter Pivot", pivot.runToPositionCommand(16.7));

        // ShooterTableEntry fourLastEntry = new ShooterTableEntry(Feet.of(0), 15, 1.0);
        NamedCommands.registerCommand("4 Piece Last Shooter Pivot", pivot.runToPositionCommand(13));

        NamedCommands.registerCommand("Stop Shooter", shooter.stopCommand());
        NamedCommands.registerCommand("Stop Infeed", runBoth(0., 0.));

        NamedCommands.registerCommand("Center Pathfinding Shot", drivetrain
                .pathFindCommand(new Pose2d(4.99, 6.66, new Rotation2d(Units.degreesToRadians(13.3))), 0.75, 0)
                .alongWith(runEntryCommand(() -> twoHalfEntry, () -> ShotSpeeds.FAST).repeatedly()
                        .until(shooterAndPivotReady()))
                .andThen(conveyCommand().withTimeout(1.0))
                .andThen(shooter.stopCommand()));

        ShooterTableEntry twoHalfRightEntry = new ShooterTableEntry(Feet.of(0), 4.0, 1.0);

        NamedCommands.registerCommand("Right Center Pathfinding Shot", drivetrain
                .mirrorablePathFindCommand(Constants.RIGHT_3_SHOOT_PATHFINDING_POSE, 0.75, 0)
                .alongWith(runEntryCommand(() -> twoHalfRightEntry, () -> ShotSpeeds.FAST).repeatedly()
                        .until(shooterAndPivotReady()))
                .andThen(conveyCommand().withTimeout(1.0))
                .andThen(shooter.stopCommand()));

        NamedCommands.registerCommand("Note 3",
                drivetrain.pathFindCommand(new Pose2d(4.67, 6.7, Rotation2d.fromDegrees(-18)), 0.75, 2.5));

        NamedCommands.registerCommand("Note 3 Right",
                drivetrain.pathFindCommand(new Pose2d(4.67, 1.5, Rotation2d.fromDegrees(18)), 0.75, 2.5));

        NamedCommands.registerCommand("Note 4",
                drivetrain.pathFindCommand(new Pose2d(3.66, 6.93, Rotation2d.fromDegrees(70.)), 0.75, 0));

        NamedCommands.registerCommand("Note 4 Right",
                drivetrain.pathFindCommand(new Pose2d(3.66, 1.2, Rotation2d.fromDegrees(-70.)), 0.75, 0));

        NamedCommands.registerCommand("follow2pchoice",
                new ConditionalCommand(AutoBuilder.followPath(PathPlannerPath.fromPathFile("2pleft")),
                        AutoBuilder.followPath(PathPlannerPath.fromPathFile("2pright")),
                        () -> true));

        NamedCommands.registerCommand("Wait For Shooter", Commands.waitUntil(shooterAndPivotReady()));

        NamedCommands.registerCommand("4 piece align pivot", pivot.runToPositionCommand(16.0));

        NamedCommands.registerCommand("4p 4th shoot", AutoBuilder.followPath(PathPlannerPath
                .fromPathFile(allianceIsBlue(DriverStation.getAlliance()) ? "4 piece 2-3" : "4 piece 2-3 red")));
        NamedCommands.registerCommand("4p 5th shoot", AutoBuilder.followPath(PathPlannerPath.fromPathFile(
                allianceIsBlue(DriverStation.getAlliance()) ? "4pend-5th-shoot" : "4pend-5th-shoot red")));

        /*
         * Spin up Shooter
         * When shooter ready, feed
         * Stop shooter
         */
        NamedCommands.registerCommand("2.5 Stationary Shot",
                runEntryCommand(() -> twoHalfEntry, () -> ShotSpeeds.FAST)
                        .repeatedly().until(shooterAndPivotReady())
                        .andThen(conveyCommand())
                        .andThen(shooter.stopCommand()));

        /* 2.5 but right side */
        NamedCommands.registerCommand("2.5 Right Stationary Shot",
                runEntryCommand(() -> twoHalfRightEntry, () -> ShotSpeeds.FAST)
                        .repeatedly().until(shooterAndPivotReady())
                        .andThen(conveyCommand())
                        .andThen(shooter.stopCommand()));

        NamedCommands.registerCommand("2.5 Final Note", drivetrain
                .pathFindCommand(new Pose2d(3.71, 6.51, new Rotation2d(Units.degreesToRadians(-9.1))), 0.75, 0));
        NamedCommands.registerCommand("2.5 Final Note Right", drivetrain
                .pathFindCommand(new Pose2d(3.71, 1.8, new Rotation2d(Units.degreesToRadians(-20))), 0.75, 0.));

    }

    private boolean allianceIsBlue(Optional<Alliance> alliance) {
        return alliance.isEmpty() || alliance.get() == Alliance.Blue;
    }

    // =========================== //
    /* Bindings & Default Commands */
    // =========================== //
    private void configureBindings() {
        // ================ //
        /* Default Commands */
        // ================ //

        drivetrain.setDefaultCommand(
                drivetrain.applyRequest(() -> drive
                        .withVelocityX(
                                (DriverStation.getAlliance().isPresent()
                                        && DriverStation.getAlliance().get() == Alliance.Red
                                                ? -1
                                                : 1)
                                        * scaleDriverController(-driverController.getLeftY(),
                                                xLimiter,
                                                currentSpeed)
                                        * MAX_SPEED)
                        .withVelocityY((DriverStation.getAlliance().isPresent()
                                && DriverStation.getAlliance().get() == Alliance.Red
                                        ? -1
                                        : 1)
                                * scaleDriverController(-driverController.getLeftX(),
                                        yLimiter,
                                        currentSpeed)
                                * MAX_SPEED)
                        .withRotationalRate(
                                scaleDriverController(-driverController.getRightX(),
                                        thetaLimiter, currentSpeed) *
                                        MAX_ANGULAR_SPEED)));

        conveyor.setDefaultCommand(conveyor.runMotorCommand(0.));
        infeed.setDefaultCommand(infeed.runMotorCommand(0.));

        // ================= //
        /* DRIVER CONTROLLER */
        // ================= //

        // ========================= //
        /* Infeed & Conveyor Control */
        // ========================= //

        /* Dumb Infeed */
        driverController.leftTrigger().onTrue(
                runBoth(SLOW_CONVEYOR_VBUS, INFEED_VBUS).repeatedly())
                .onFalse(shooter.spinMotorLeftCommand(SHOOTER_BACKOUT_VBUS).repeatedly()
                        .raceWith(conveyor.runXRotations(-2.5).withTimeout(0.2) // -1.5
                                .alongWith(infeed.runMotorCommand(0.)))
                        .andThen(shooter.stopCommand()));

        /* Smart Infeed */
        driverController.leftBumper().toggleOnTrue(smartInfeedCommand());

        // ========================== //
        /* Drivetain & Vision Control */
        // ========================== //

        /* Robot-Relative Drive */
        driverController.y().toggleOnTrue(drivetrain.applyRequest(() -> robotRelativeDrive
                .withVelocityX(
                        (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red
                                ? -1
                                : 1) * scaleDriverController(-driverController.getLeftY(),
                                        xLimiter,
                                        currentSpeed)
                                * MAX_SPEED)
                .withVelocityY(
                        (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red
                                ? -1
                                : 1) * scaleDriverController(-driverController.getLeftX(),
                                        yLimiter,
                                        currentSpeed)
                                * MAX_SPEED)
                .withRotationalRate(
                        scaleDriverController(-driverController.getRightX(),
                                thetaLimiter, currentSpeed) *
                                MAX_ANGULAR_SPEED)));

        /* X-Drive */
        driverController.x().whileTrue(drivetrain.applyRequest(() -> xDrive));

        /* Reset Field-Centric Heading */
        driverController.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative(new Pose2d())));

        /* Add Vision Measurement */
        driverController.back()
                .onTrue(Commands.runOnce(() -> {
                    var pose = getBestPose();
                    if (pose.isPresent())
                        drivetrain.seedFieldRelative(pose.get().estimatedPose.toPose2d());

                    getBestSTEntry();
                }));

        /* Toggle Chassis Mode */
        driverController.rightBumper().onTrue(Commands.runOnce(() -> currentSpeed = SLOW_SPEED))
                .onFalse(Commands.runOnce(() -> currentSpeed = BASE_SPEED));

        // ========================= //
        /* Misc */
        // ========================= //

        /* End snap, limelight & stop all motors */
        driverController.rightStick().onTrue(stopAllCommand().alongWith(drivetrain.runOnce(() -> {
        })));

        driverController.leftStick().whileTrue(new ShooterAlign(drivetrain, trapVision));

        // =================== //
        /* OPERATOR CONTROLLER */
        // =================== //

        // ========================= //
        /* Driver Help Control */
        // ========================= //

        /* Snap to Amp */
        operatorController.povUp().toggleOnTrue(Commands.either(
                snapCommand(SnapDirection.Left), snapCommand(SnapDirection.Right),
                () -> {
                    var alliance = DriverStation.getAlliance();
                    return alliance.isPresent() && alliance.get() == Alliance.Blue;
                }));

        /* Trap Snapping */
        operatorController.povRight().toggleOnTrue(snapCommand(SnapDirection.LeftTrap));
        operatorController.povLeft().toggleOnTrue(snapCommand(SnapDirection.RightTrap));
        operatorController.povDown().toggleOnTrue(snapCommand(SnapDirection.Back));

        /* Limelight Square */
        operatorController.leftStick().toggleOnTrue(new LimelightSquare(true,
                () -> scaleDriverController(-driverController.getLeftY(), xLimeAquireLimiter, currentSpeed) * MAX_SPEED,
                () -> scaleDriverController(-driverController.getLeftX(), yLimeAquireLimiter, currentSpeed) * MAX_SPEED,
                drivetrain));

        // ======================= //
        /* Shooter Control */
        // ======================= //

        /* Spin Up Shooter */
        operatorController.leftTrigger()
                .onTrue(Commands.runOnce(() -> {
                    ShooterTableEntry entry = ShooterTable
                            .calcShooterTableEntry(Feet.of(currentIndex));
                    shooter.runEntry(entry, ShotSpeeds.FAST);
                    pivot.runToPosition(entry.Angle);
                }, shooter, pivot))
                .onFalse(stopAllCommand());

        /* Convey Note */
        operatorController.rightTrigger()
                .whileTrue(runBoth(FAST_CONVEYOR_VBUS, SLOW_INFEED_VBUS).repeatedly());

        /* Magic Shoot */
        operatorController.x().toggleOnTrue(magicShootCommand);

        /* Manual/Preset Mode */
        operatorController.back().onTrue(Commands.runOnce(() -> useManual = !useManual).andThen(this::pushIndexData));

        /* Shooter Table Index Up */
        operatorController.rightBumper().onTrue(
                Commands.either(
                        Commands.runOnce(() -> manualIndex += 1.0),
                        Commands.runOnce(() -> presetIndex += 1),
                        () -> useManual).andThen(this::pushIndexData));

        /* Shooter Table Index Down */
        operatorController.leftBumper().onTrue(
                Commands.either(
                        Commands.runOnce(() -> manualIndex -= 1.0),
                        Commands.runOnce(() -> presetIndex -= 1),
                        () -> useManual).andThen(this::pushIndexData));

        // ========================= //
        /* Climber & Zeroing Control */
        // ========================= //

        /* Zero Pivot */
        operatorController.start().onTrue(zeroCommand());

        /* Run Pivot & Climber to Zero */
        operatorController.a().onTrue(pivot.runToHomeCommand()
                .alongWith(trapVision.setPipelineCommand(Vision.SHOOTER_PIPELINE_INDEX)));

        // ================ //
        /* Amp & Trap Magic */
        // ================ //

        operatorController.b().onTrue(ampPrep).onFalse(stopAllCommand());

        operatorController.y()
                .onTrue(pivot.runToTrapCommand().alongWith(trapVision.setPipelineCommand(Vision.TRAP_PIPELINE_INDEX)));

        // ==================== //
        /* EMERGENCY CONTROLLER */
        // ==================== //

        // ==================== //
        /* Manual Pivot Control */
        // ==================== //

        /* Bump Pivot Up */
        emergencyController.rightBumper()
                .onTrue(pivot.runOnce(() -> pivot.runToPosition(pivot.getPosition() + 2)));

        /* Bump Pivot Down */
        emergencyController.leftBumper()
                .onTrue(pivot.runOnce(() -> pivot.runToPosition(pivot.getPosition() - 2)));

        // ============================== //
        /* Manual Climber/Outfeed Control */
        // ============================== //

        /* Climber Up */
        emergencyController.rightTrigger(0.2).whileTrue(
                climber.runMotorCommand(CLIMBER_VBUS))
                .onFalse(climber.runMotorCommand(0.0));

        /* Climber Down */
        emergencyController.leftTrigger(0.2).whileTrue(
                climber.runMotorCommand(-CLIMBER_VBUS))
                .onFalse(climber.runMotorCommand(0.0));

        // ==== //
        /* Misc */
        // ==== //

        /* whippyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyy */
        emergencyController.a().onTrue(whippy.whippyWheelsCommand(WHIPPY_VBUS))
                .onFalse(whippy.stopCommand());

        /* TrapStar 5000 */
        // emergencyController.y().onTrue(m_fan.runMotorCommand(FAN_VBUS)).onFalse(m_fan.stopCommand());

        emergencyController.start().onTrue(m_fan.runToPositionCommand(0.));

        emergencyController.back().onTrue(Commands.runOnce(() -> enableClimber = !enableClimber));

        /* ST test */
        // emergencyController.back().onTrue(Commands.runOnce(() -> getBestSTEntry()));

        /* Full Outfeed: left Y */
        emergencyController.axisGreaterThan(XboxController.Axis.kLeftY.value, 0.2)
                .or(emergencyController.axisLessThan(XboxController.Axis.kLeftY.value, -0.2))
                .whileTrue(
                        runThree(
                                () -> -emergencyController.getLeftY(),
                                () -> -emergencyController.getLeftY(),
                                () -> -emergencyController.getLeftY()));

        /* Cool Outfeed: right Y */
        emergencyController.axisLessThan(XboxController.Axis.kRightY.value, -0.2)
                .whileTrue(
                        runThree(
                                () -> emergencyController.getRightY(),
                                () -> -emergencyController.getRightY(),
                                () -> emergencyController.getRightY()));

        /* TRAP STUFF */

        /* Prime Fan Pivot & Shooter Pivot */
        emergencyController.x().onTrue(m_fan.runToTrapCommand().alongWith(pivot.runToTrapCommand()));

        /* Run Shooter at Trap Speeds */
        emergencyController.y().onTrue(shooter.setSlotCommand(Shooter.Slots.TRAP).andThen(
                shooter.runShotCommand(ShotSpeeds.TRAP)
                        .alongWith(m_fan.runMotorCommand(FAN_VBUS))));

        /* Trap Shoot */
        emergencyController.b().onTrue(conveyCommand()).onFalse(stopAllCommand());

        // ======= //
        /* CLIMBER */
        // ======= //

        /* Prime Climber */
        emergencyController.povUp().onTrue(safeClimbCommand(Climber.ClimberPositions.READY));

        /* Tension Chain */
        emergencyController.povRight().onTrue(safeClimbCommand(Climber.ClimberPositions.TENSION));

        /* CLIMB */
        emergencyController.povDown().onTrue(safeClimbCommand(ClimberPositions.CLIMB));

        if (Utils.isSimulation()) {
            drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
        }

        drivetrain.registerTelemetry(logger::telemeterize);

    }

    // =========================================== //
    /* Additional Commands, Getters, and Utilities */
    // =========================================== //

    /* Check if Climber encoder is ready */
    public boolean climberReady() {
        return climber.encoderReady();
    }

    /* Rezero climber */
    public void rezeroClimber() {
        climber.reZero();
    }

    /* Safe Climb */
    private Command safeClimbCommand(ClimberPositions position) {
        Command climbCommand = position == ClimberPositions.CLIMB ? climber.climbCommand()
                : climber.runToPositionCommand(position);

        return Commands.either(
                climbCommand,
                Commands.none(),
                () -> pivot.getPosition() > 50. && enableClimber);
    }

    /* Convey the Note */
    private Command conveyCommand() {
        return conveyor.runXRotations(20.).alongWith(infeed.runMotorCommand(SLOW_INFEED_VBUS));
    }

    /* Stop all motors and zero everything */
    private void stopAll() {
        stopAllCommand().schedule();
    }

    private Command stopAllCommand() {
        return Commands.parallel(
                infeed.stopCommand(),
                conveyor.stopCommand(),
                shooter.stopCommand().andThen(shooter.setSlotCommand(Shooter.Slots.FAST)),
                pivot.runToHomeCommand()/* ) */,
                m_fan.stopCommand().andThen(
                        m_fan.runToPositionCommand(0.)),
                whippy.stopCommand());
    }

    /* Put Current ST Index Data to Dashboard */
    private void pushIndexData() {
        presetIndex = MathUtil.clamp(presetIndex, 0, indexMap.size() - 1);
        manualIndex = MathUtil.clamp(manualIndex, MIN_INDEX, MAX_INDEX);

        if (useManual) {
            currentIndex = manualIndex;
        } else {
            List<Double> keys = new ArrayList<>(indexMap.keySet());
            currentIndex = keys.get(presetIndex);
            manualIndex = currentIndex;
        }
    }

    /* Set Snap Direction Toggle */
    private Command snapCommand(SnapDirection direction) {
        return drivetrain.applyRequest(() -> snapDrive
                .withVelocityX(
                        (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red
                                ? -1
                                : 1) * scaleDriverController(-driverController.getLeftY(),
                                        xLimiter,
                                        currentSpeed)
                                * MAX_SPEED)
                .withVelocityY(
                        (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red
                                ? -1
                                : 1) * scaleDriverController(-driverController.getLeftX(),
                                        yLimiter,
                                        currentSpeed)
                                * MAX_SPEED)
                .withTargetDirection(Rotation2d.fromDegrees(direction.Angle)));
    }

    /* Smart Infeed Command Generator */
    private Command smartInfeedCommand() {
        return runBoth(SLOW_CONVEYOR_VBUS, INFEED_VBUS)
                .repeatedly().until(conveyor.hasInfedSupplier())
                .andThen(runBoth(0., 0.)
                        .repeatedly().withTimeout(0.1))
                .andThen(shooter.spinMotorLeftCommand(SHOOTER_BACKOUT_VBUS).repeatedly()
                        .raceWith(conveyor.runXRotations(-4.0).withTimeout(0.5) // -1.5
                                .alongWith(infeed.runMotorCommand(0.))))
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
        return infeed.runMotorCommand(infeedVbus).alongWith(conveyor.runMotorCommand(conveyorVbus));
    }

    /* Run Conveyor, Infeed, and shooter if backwards */
    private Command runThree(Supplier<Double> conveyorVbus, Supplier<Double> infeedVbus, Supplier<Double> shooterVbus) {
        return new FunctionalCommand(() -> {
        },
                () -> {
                    infeed.runMotor(infeedVbus.get());
                    conveyor.runMotor(conveyorVbus.get());
                    shooter.spinMotorRight(0.3 * shooterVbus.get());
                    shooter.spinMotorLeft(0.3 * shooterVbus.get());
                },
                (z) -> shooter.stop(),
                () -> false,
                infeed, conveyor);
    }

    /* Auton Command */
    public Command getAutonomousCommand() {
        return new InstantCommand(() -> drivetrain.seedFieldRelative(new Pose2d()))
                .andThen(NamedCommands.getCommand("zeroApril"))
                .andThen(autonChooser.getSelected());
    }

    /* Zeroing Command */
    public Command zeroCommand() {
        return pivot.zeroCommand().alongWith(m_fan.runToPositionCommand(0.));
    }

    /* Asynchronous Zero */
    public void zero() {
        zeroCommand().schedule();
    }

    /* Joystick Scaling */
    private double scaleDriverController(double controllerInput, SlewRateLimiter limiter, double baseSpeedPercent) {
        // controls are reversed for red
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
        // climber.logValues();
        pivot.logValues();
        m_fan.logValues();
    }

    // ================ //
    /* Vision Utilities */
    // ================ //

    /* Configure Field Origins */
    public void configVisionFieldOrigins() {
        trapVision.configFieldOrigin();
        leftVision.configFieldOrigin();
        rightVision.configFieldOrigin();
    }

    /* Return approx. 3d pose */
    public Optional<EstimatedRobotPose> getBestPose() {
        Pose2d drivetrainPose = drivetrain.getState().Pose;

        Optional<EstimatedRobotPose> front = rightVision.getCameraResult(drivetrainPose);
        Optional<EstimatedRobotPose> back = leftVision.getCameraResult(drivetrainPose);

        int numPoses = 0;

        numPoses += front.isPresent() ? 1 : 0;
        numPoses += back.isPresent() ? 1 : 0;

        Optional<Pose2d> pose = Optional.empty();

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
                            .plus(new Transform2d(Units.inchesToMeters(13.), 0.,
                                    new Rotation2d()))),
                    (front.isEmpty() ? back : front).get().timestampSeconds,
                    null, null));
        }

        return Optional.empty();
    }

    /* Get Shooter Table Entry */
    public ShooterTableEntry getBestSTEntry() {
        Pose2d pose = drivetrain.getState().Pose;

        Transform2d dist = pose.minus(Constants.SPEAKER_DISTANCE_TARGET);
        Translation2d translation = dist.getTranslation();

        ShooterTableEntry entryPicked = ShooterTable
                .calcShooterTableEntry(Meters.of(translation.getNorm()));

        SmartDashboard.putNumber("Distance", Units.metersToFeet(translation.getNorm()));

        SmartDashboard.putNumber("ST Angle", entryPicked.Angle);
        SmartDashboard.putNumber("ST Left", entryPicked.Percent);

        return entryPicked;
    }

    public void printDistanceValues() {
        int tagID = DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Red ? 4 : 7;

        var right = rightVision.getBestTranslationToTarget(tagID);
        var left = leftVision.getBestTranslationToTarget(tagID);

        Translation2d tr;

        if (right.isPresent() && left.isPresent()) {
            tr = right.get().plus(left.get()).div(2.);
        } else if (right.isPresent()) {
            tr = right.get();
        } else if (left.isPresent()) {
            tr = left.get();
        } else {
            tr = new Translation2d();
        }

        SmartDashboard.putNumber("Swerve Distance", Units.metersToFeet(tr.getNorm()));

        var rightYaw = rightVision.getTagYaw(tagID);
        var leftYaw = leftVision.getTagYaw(tagID);

        double yaw = 0.;

        if (rightYaw.isPresent() && leftYaw.isPresent()) {
            yaw = rightYaw.get() + leftYaw.get() / 2.;
        } else if (rightYaw.isPresent()) {
            yaw = rightYaw.get();
        } else if (leftYaw.isPresent()) {
            yaw = leftYaw.get();
        }

        SmartDashboard.putNumber("Swerve Yaw", yaw);

        var fiducials = LimelightHelpers.getLatestResults("limelight-shooter").targetingResults.targets_Fiducials;

        for (var fiducial : fiducials) {
            if (fiducial.fiducialID == tagID) {
                SmartDashboard.putNumber("Shooter Yaw", fiducial.tx);

                double tagZMeters = trapVision.layout().getTagPose(tagID).get().getZ();
                double angle = Units.degreesToRadians(fiducial.ty);

                double tangent = Math.tan(SHOOTER_CAM_PITCH + angle);
                double deltaHeight = Units.metersToFeet(tagZMeters - SHOOTER_CAM_HEIGHT);

                SmartDashboard.putNumber("Shooter ty", angle);
                SmartDashboard.putNumber("Shooter tangent", tangent);
                SmartDashboard.putNumber("Shooter dh", deltaHeight);

                SmartDashboard.putNumber("Shooter Distance",
                        deltaHeight / tangent);
            }
        }

        // Optional<Double> shooterYaw = trapVision.getTagYaw(tagID);

        // if (shooterYaw.isPresent()) {
        // SmartDashboard.putNumber("Shooter Yaw", shooterYaw.get());
        // } else {
        // SmartDashboard.putNumber("Shooter Yaw", 0.);
        // }

        // Optional<Double> shooterDist = trapVision.getTagDistance(tagID);

        // if (shooterDist.isPresent()) {
        // SmartDashboard.putNumber("Shooter Distance",
        // Units.metersToFeet(shooterDist.get()));
        // } else {
        // SmartDashboard.putNumber("Shooter Distance", 0.);
        // }

    }
}
