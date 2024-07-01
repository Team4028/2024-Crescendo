// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Meters;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.SwerveDriveBrake;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Candle;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Climber.ClimberPositions;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Fan;
import frc.robot.subsystems.FanPivot;
import frc.robot.subsystems.Infeed;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShotSpeeds;
import frc.robot.subsystems.Shooter.Slots;
import frc.robot.subsystems.Whippy;
import frc.robot.utils.BeakCommands;
import frc.robot.utils.BeakUtils;
import frc.robot.utils.DashboardStore;
import frc.robot.utils.DriverCamera;
import frc.robot.utils.Limelight;
import frc.robot.utils.LogStore;
import frc.robot.utils.NoteSensing;
import frc.robot.utils.ShooterTable;
import frc.robot.utils.ShooterTable.ShooterTableEntry;
import frc.robot.utils.ShooterTable.VisionTableEntry.CameraLerpStrat;
import frc.robot.utils.ShootingStrategy;
import frc.robot.utils.SubAutos;

public class RobotContainer {
    // =============================================== //
    /** Magic numbers, Vbus constants, and OI constants */
    // =============================================== //
    private static final double CLIMBER_VBUS = 0.75;
    private static final double FAST_CLIMBER_VBUS = 0.95;

    private static final double INFEED_VBUS = 0.8;
    private static final double SLOW_INFEED_VBUS = 0.5;

    private static final double SLOW_CONVEYOR_VBUS = 0.5;
    private static final double FAST_CONVEYOR_VBUS = 0.85;

    private static final double FAN_VBUS = 1.;

    private static final double SHOOTER_BACKOUT_VBUS = -0.2;
    private static final double WHIPPY_VBUS = 0.2;

    private static final int OI_DRIVER_CONTROLLER = 0;
    private static final int OI_OPERATOR_CONTROLLER = 1;
    private static final int OI_EMERGENCY_CONTROLLER = 2;

    // ======================== //
    /** Controllers & Subsystems */
    // ======================== //
    private final CommandXboxController driverController = new CommandXboxController(OI_DRIVER_CONTROLLER);
    private final CommandXboxController operatorController = new CommandXboxController(OI_OPERATOR_CONTROLLER);
    private final CommandXboxController emergencyController = new CommandXboxController(OI_EMERGENCY_CONTROLLER);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain;
    private final Infeed infeed = new Infeed();
    private final Shooter shooter = new Shooter();
    private final Conveyor conveyor = new Conveyor();
    private final Climber climber = new Climber();
    private final Pivot pivot = new Pivot();
    private final Fan m_fan = new Fan();
    private final FanPivot m_fanPivot = new FanPivot();
    private final Whippy whippy = new Whippy();

    private final Candle candle = new Candle();
    private final NoteSensing noteSensing = new NoteSensing();
    private final DriverCamera driverCamera = new DriverCamera();

    /** Vision Systems */

    private static final String SHOOTER_LIMELIGHT = "limelight-ii";
    private static final String CHASSIS_LIMELIGHT = "limelight-gii";
    private static final String INFEED_LIMELIGHT_3G = "limelight-gi";

    /** Pipeline for 3G MegaTag2 */
    private static final int MEGATAG_PIPELINE = 0;

    /** Pipeline for 3G 2D TY */
    private static final int TY_PIPELINE = 1;

    // private final PhotonVision rightVision = new
    // PhotonVision("Right_AprilTag_Camera",
    // VisionSystem.RIGHT_ROBOT_TO_CAMERA);
    // private final PhotonVision leftVision = new
    // PhotonVision("Left_AprilTag_Camera",
    // VisionSystem.LEFT_ROBOT_TO_CAMERA);

    private final Limelight shooterLimelight = new Limelight(SHOOTER_LIMELIGHT, new Transform3d());

    private final Limelight chassisLimelight = new Limelight(CHASSIS_LIMELIGHT,
            new Transform3d(-0.278, 0.17, 0.203, new Rotation3d(0, Units.degreesToRadians(33), 0)));
    private final Limelight infeedLimelight3G = new Limelight(INFEED_LIMELIGHT_3G, new Transform3d());

    /** Shooting Strategies */
    private final ShootingStrategy shooterLimelightStrategy = new ShootingStrategy(shooterLimelight,
            CameraLerpStrat.LimelightTY);

    private final ShootingStrategy odometryStrategy = new ShootingStrategy(drivetrain);

    private final ShootingStrategy chassisLimelight2dStrategy = new ShootingStrategy(chassisLimelight,
            CameraLerpStrat.Limelight3GTY);

    private ShootingStrategy selectedStrategy = chassisLimelight2dStrategy;

    private final HashMap<ShootingStrategy, String> strategyMap = new HashMap<>(Map.of(
            shooterLimelightStrategy, "MVR",
            odometryStrategy, "MegaTag2",
            chassisLimelight2dStrategy, "3G 2D"));

    private ShooterTableEntry entryToRun;

    // ====================== //
    /** Auton & Other Commands */
    // ====================== //
    private final Command ampPrep;
    private SendableChooser<Command> autonChooser;

    private final SubAutos m_autos;

    // ====================================================== //
    /** Drivetrain Constants, Magic numbers, and ` Limiters */
    // ====================================================== //
    private final SlewRateLimiter xLimiter = new SlewRateLimiter(4.);
    private final SlewRateLimiter yLimiter = new SlewRateLimiter(4.);
    private final SlewRateLimiter thetaLimiter = new SlewRateLimiter(4.);

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
        RightTrap(60.),
        BluePass(-40.0),
        RedPass(-150.0);

        public double Angle;

        private SnapDirection(double angle) {
            Angle = angle;
        }
    }

    private enum ClimbSequence {
        /* Default State */
        Default,

        /* Shooter Position & Juggle */
        Prep,

        /* Fan, Fan Position, Shooter, Climber Up (if climbing) */
        Fan,

        /* Shoot */
        Shoot,

        /* End of Sequence */
        End,

        /* Climb */
        Climb
    }

    private ClimbSequence currentSequence = ClimbSequence.Default;
    private final Map<ClimbSequence, Command> sequenceCommandMap = Map.of(
            ClimbSequence.Default, Commands.none(),
            ClimbSequence.Prep, prepClimbCommand(),
            ClimbSequence.Fan, fanReadyCommand(),
            ClimbSequence.Shoot, trapShootCommand(),
            ClimbSequence.End, endSequenceCommand(),
            ClimbSequence.Climb, climbCommand());

    private static double MAX_INDEX = 27.;
    private static double MIN_INDEX = 4.2;

    private static final double PIVOT_UP_THRESHOLD = 40.0;

    private static final double CAMERA_SWITCH_TIMEOUT = 1.5;

    private static ShooterTableEntry PASSING_SHOT = new ShooterTableEntry(Feet.zero(), 4, 0.69, Feet.zero());// 30.9;

    private final LinkedHashMap<Double, String> indexMap = new LinkedHashMap<>();

    private double currentIndex = MIN_INDEX;
    private double manualIndex = MIN_INDEX;
    private int presetIndex = 0;
    private boolean useManual = false;

    private boolean enableClimber = false;
    private boolean enableTrap = false;

    private double m_lastShot = 0.0;

    // ======================== //
    /** Swerve Control & Logging */
    // ======================== //
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MAX_SPEED * 0.02).withRotationalDeadband(MAX_ANGULAR_SPEED * 0.01)
            .withDriveRequestType(DriveRequestType.Velocity);

    private final SwerveRequest.RobotCentric robotRelativeDrive = new SwerveRequest.RobotCentric()
            .withDeadband(MAX_SPEED * 0.02).withRotationalDeadband(MAX_ANGULAR_SPEED * 0.01)
            .withDriveRequestType(DriveRequestType.Velocity);

    private final Telemetry logger = new Telemetry(MAX_SPEED);

    private final SwerveRequest.SwerveDriveBrake xDrive = new SwerveDriveBrake();
    private final SwerveRequest.FieldCentricFacingAngle snapDrive = new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(MAX_SPEED * 0.035)
            .withDriveRequestType(DriveRequestType.Velocity);

    public RobotContainer() {
        snapDrive.HeadingController = new PhoenixPIDController(10, 0., 0.);
        snapDrive.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

        m_autos = new SubAutos(noteSensing);

        /* Init Index Map */
        indexMap.put(4.2, "Speaker");
        indexMap.put(5.2, "SideWoofer");
        indexMap.put(10.0, "Protected");
        indexMap.put(13.0, "Chain");
        indexMap.put(16.0, "Truss");
        indexMap.put(19.0, "Left Wing");
        indexMap.put(22.0, "Right Wing");

        ShooterTable.setHeckinessLevel(() -> drivetrain.getRotation());

        /* Dashboard */
        DashboardStore.add("Shooter Table Index", () -> currentIndex);
        DashboardStore.add("Shooter Table Name",
                () -> indexMap.containsKey(currentIndex) ? indexMap.get(currentIndex) : "Manual");

        DashboardStore.add("Chassis 2D Distance",
                () -> {
                    var res = chassisLimelight.getTagDistance(7);
                    if (res.isPresent())
                        return Units.metersToFeet(res.get());
                    return Double.NaN;
                });

        // this is weird
        DashboardStore.add("Snapped",
                () -> drivetrain.getCurrentRequest().getClass().equals(snapDrive.getClass()));
        DashboardStore.add("Robot Relative",
                () -> drivetrain.getCurrentRequest().getClass().equals(robotRelativeDrive.getClass()));

        DashboardStore.add("Manual Indexing", () -> useManual);
        DashboardStore.add("Climber Enabled", () -> enableClimber);
        DashboardStore.add("Trap Enabled", () -> enableTrap);

        DashboardStore.add("Sequence", () -> currentSequence.name());

        DashboardStore.add("Limelight Distance",
                () -> shooterLimelightStrategy.getTargetEntry().Distance.in(Feet));
        DashboardStore.add("LimelightG Distance",
                () -> chassisLimelight2dStrategy.getTargetEntry().Distance.in(Feet));

        DashboardStore.add("Chassis MT2 Distance", () -> BeakUtils
                .goalTranslation(chassisLimelight.getBotposeEstimateMT2().pose.getTranslation()).getNorm());

        DashboardStore.add("Infeed MT2 Distance", () -> BeakUtils
                .goalTranslation(infeedLimelight3G.getBotposeEstimateMT2().pose.getTranslation()).getNorm());

        DashboardStore.add("Last Shot", () -> m_lastShot);
        DashboardStore.add("Odometry Distance",
                () -> Units.metersToFeet(
                        BeakUtils.goalTranslation(drivetrain.getTranslation())
                                .getNorm()));

        DashboardStore.add("Strategy", () -> strategyMap.get(selectedStrategy));

        LogStore.add("/Buttons/New Shoot", () -> emergencyController.getHID().getAButton()); // emergencyController.y().getAsBoolean());
        LogStore.add("/Buttons/Magic Shoot", () -> operatorController.getHID().getXButton()); // operatorController.x().getAsBoolean());

        initNamedCommands();

        initAutonChooser();

        ampPrep = pivot.runToClimbCommand()
                .alongWith(whippy.whippyWheelsCommand(WHIPPY_VBUS))
                .alongWith(shooter.setSlotCommand(Shooter.Slots.AMP)
                        .andThen(shooter.runShotCommand(ShotSpeeds.AMP)));

        configureBindings();
    }

    // ====================== //
    /** Auton & Named Commands */
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

        SmartDashboard.putData("Auto Chooser", autonChooser);
    }

    private void initNamedCommands() {
        /* ... */
        NamedCommands.registerCommand("Pivot Zero", pivot.zeroCommand());

        /* Infeed & Spit */
        NamedCommands.registerCommand("Smart Infeed", smartInfeedCommand());

        NamedCommands.registerCommand("Dumb Infeed",
                runBoth(true, SLOW_CONVEYOR_VBUS, INFEED_VBUS).withTimeout(.25));

        NamedCommands.registerCommand("Infeed", infeed.runMotorCommand(INFEED_VBUS)
                .alongWith(conveyor.runMotorCommand(FAST_CONVEYOR_VBUS)).repeatedly());// .withTimeout(1.5));

        NamedCommands.registerCommand("C Infeed", infeed.runMotorCommand(INFEED_VBUS)
                .alongWith(conveyor.runMotorCommand(FAST_CONVEYOR_VBUS)).repeatedly().withTimeout(1.0));

        NamedCommands.registerCommand("First Spit Note", infeed.runMotorCommand(INFEED_VBUS)
                .alongWith(conveyor.runMotorCommand(FAST_CONVEYOR_VBUS))
                .alongWith(shooter.spinBothCommand(0.20))
                .repeatedly());
        NamedCommands.registerCommand("Spit Note", infeed.runMotorCommand(INFEED_VBUS)
                .alongWith(conveyor.runMotorCommand(FAST_CONVEYOR_VBUS))
                .alongWith(shooter.spinBothCommand(0.11))
                .repeatedly());

        NamedCommands.registerCommand("Prepare Spit", shooter.spinBothCommand(0.15));

        NamedCommands.registerCommand("Fix Note", fixNoteCommand());

        NamedCommands.registerCommand("Finish Infeed",
                smartInfeedAutoCommand().andThen(shooter.runShotCommand(ShotSpeeds.FAST)));

        NamedCommands.registerCommand("Magic Shoot",
                Commands.waitSeconds(0.1).andThen(magicShootCommand(() -> odometryStrategy)));// updateDrivePoseMT2Command().repeatedly().withTimeout(0.1).andThen(magicShootCommand(()
                                                                                              // -> odometryStrategy)));

        NamedCommands.registerCommand("Mega Tag Shoot", magicShootCommand(() -> odometryStrategy));
        NamedCommands.registerCommand("Choose Shoot", magicShootCommand());

        NamedCommands.registerCommand("Spit Wipe Note 5-3",
                drivetrain.staticAlign(() -> Constants.AutoPoses.DOWNWARD_ROTATION).withTimeout(0.5)
                        .andThen(conveyCommand().withTimeout(0.5))
                        .andThen(drivetrain.staticAlign(() -> Rotation2d.fromDegrees(90.0)).withTimeout(0.5))
                        .onlyIf(noteSensing.hasInfedSupplier()));

        NamedCommands.registerCommand("Spit Wipe Note 2",
                drivetrain.staticAlign(() -> Constants.AutoPoses.UPWARD_ROTATION).withTimeout(0.5)
                        .andThen(runThree(() -> FAST_CONVEYOR_VBUS, () -> INFEED_VBUS, () -> 0.2).withTimeout(0.25))
                        .andThen(drivetrain.staticAlign(() -> Rotation2d.fromDegrees(90.0)).withTimeout(0.5))
                        .onlyIf(noteSensing.hasInfedSupplier()));

        /* Shooter & Pivot */
        NamedCommands.registerCommand("Fast Shooter",
                shooter.runShotCommand(ShotSpeeds.FAST));

        NamedCommands.registerCommand("Shooter",
                shooter.runShotCommand(ShotSpeeds.FAST, 0.85));

        NamedCommands.registerCommand("Stop Shooter", shooter.stopCommand());

        NamedCommands.registerCommand("Home Pivot", pivot.runToHomeCommand());

        NamedCommands.registerCommand("Rotate To Speaker Source PC3",
                drivetrain.staticAlign(
                        () -> Rotation2d.fromDegrees(BeakUtils.allianceIsBlue() ? -63 : -117)));

        /* 4 piece pivots */
        NamedCommands.registerCommand("Preload Note", pivot.runToPositionCommand(16.0)
                .alongWith(driverCamera.setShooterCameraCommand())); // 17

        NamedCommands.registerCommand("Note A", pivot.runToPositionCommand(11)); // 11
        NamedCommands.registerCommand("Note B", pivot.runToPositionCommand(14.625)); // 15
        NamedCommands.registerCommand("Note C", pivot.runToPositionCommand(13.5)); // 13.5
        NamedCommands.registerCommand("Preload Note at sub", pivot.runToPositionCommand(30));

        NamedCommands.registerCommand("Note 1", pivot.runToPositionCommand(17.5)); // 17.5

        /* Pathfinding Shots */
        NamedCommands.registerCommand("Amp Shot", pathfindingShotCommand(15.5, Constants.LEFT_SHOT, 0.875, 0));

        NamedCommands.registerCommand("Stationary Amp Shot", shootCommand(17.0));
        NamedCommands.registerCommand("Rojo stationary Amp Shot", shootCommand(17.5));
        NamedCommands.registerCommand("Sad Stationary Amp Shot 2", shootCommand(16.5));
        NamedCommands.registerCommand("Epic Amp Shot", shootCommand(7.5)); // 7.5

        NamedCommands.registerCommand("Center Pathfinding Shot", pathfindingShotCommand(
                13.0, Constants.CENTER_SHOT, 0.8, 0.));

        NamedCommands.registerCommand("Stationary Source Shot", shootCommand(22.1));
        NamedCommands.registerCommand("Magic Source Shot",
                magicShootCommand(21.6, () -> chassisLimelight2dStrategy, true));

        NamedCommands.registerCommand("Spitless Source Shot 1", shootCommand(15.8));
        NamedCommands.registerCommand("Spitless Source Shot 2", shootCommand(17));
        NamedCommands.registerCommand("Spitless Source Shot 3", shootCommand(17.2));

        NamedCommands.registerCommand("Right Preload", pivot.runOnce(pivot::zeroEncoder)
                .andThen(shootCommand(() -> ShooterTable.calcShooterTableEntry(Feet.of(5.2)))));

        NamedCommands.registerCommand("Center Preload", pivot.runOnce(pivot::zeroEncoder)
                .andThen(shootCommand(() -> ShooterTable.calcShooterTableEntry(Feet.of(4.2)))));

        NamedCommands.registerCommand("Note C Pathfinding",
                mirroredPathfindingShotCommand(12.35, Constants.NOTE_C_SHOT, 0.85, 0.));
        // PBAC Auto Commands
        NamedCommands.registerCommand("Preload Stationary", shootCommand(4.2));
        NamedCommands.registerCommand("Stationary Shot B", shootCommand(10));
        NamedCommands.registerCommand("Stationary Shot A", shootCommand(10));
        NamedCommands.registerCommand("Stationary Shot C", shootCommand(9.6));

        NamedCommands.registerCommand("P Amp Shot", shootCommand(13.5));
        NamedCommands.registerCommand("Stationary Shot Amp", shootCommand(13.5));

        NamedCommands.registerCommand("Source Pivot", pivot.runToPositionCommand(5.0));
        NamedCommands.registerCommand("Source Pivot 4", pivot.runToPositionCommand(6.0));
        NamedCommands.registerCommand("Source Pivot Red", pivot.runToPositionCommand(5.0));

        NamedCommands.registerCommand("Amp Pivot", pivot.runToPositionCommand(4.75));
        NamedCommands.registerCommand("Amp Pivot Red", pivot.runToPositionCommand(4.75));

        NamedCommands.registerCommand("Convey", conveyCommand());
        NamedCommands.registerCommand("4 Piece Pivot", pivot.runToPositionCommand(14));
        NamedCommands.registerCommand("P Pivot", pivot.runToPositionCommand(16));
        NamedCommands.registerCommand("Stationary Source Shot", shootCommand(15.3));

        NamedCommands.registerCommand("4 Piece A", pivot.runToPositionCommand(13.75));
        NamedCommands.registerCommand("4 Piece C", pivot.runToPositionCommand(14.0));

        /* Sub Autos */
        NamedCommands.registerCommand("5 Or 4", m_autos.note5or4());
        NamedCommands.registerCommand("4 Or 3", m_autos.note4or3());

        NamedCommands.registerCommand("1 Or 2", m_autos.note1or2());
        NamedCommands.registerCommand("2 Or 3", m_autos.note2or3());

        NamedCommands.registerCommand("4 Or 5", m_autos.note4or5());
        NamedCommands.registerCommand("5 Or 3", m_autos.note5or3());
    }

    // =========================== //
    /** Bindings & Default Commands */
    // =========================== //
    private void configureBindings() {

        /* Climber Limit Switch Triggers */
        new Trigger(climber::forwardLimit).onTrue(climber.hitForwardLimitCommand());
        new Trigger(climber::reverseLimit).onTrue(climber.hitReverseLimitCommand());

        // ================ //
        /* Default Commands */
        // ================ //

        drivetrain.setDefaultCommand(
                drivetrain.applyRequest(() -> drive
                        .withVelocityX(getXSpeed(true))
                        .withVelocityY(getYSpeed(true))
                        .withRotationalRate(getRotationSpeed())));

        conveyor.setDefaultCommand(conveyor.runMotorCommand(0.));
        infeed.setDefaultCommand(infeed.runMotorCommand(0.));

        // ================= //
        /* DRIVER CONTROLLER */
        // ================= //

        // ========================= //
        /* Infeed & Conveyor Control */
        // ========================= //

        /* Dumb Infeed */
        driverController.leftBumper().onTrue(runBoth(true, SLOW_CONVEYOR_VBUS, INFEED_VBUS))
                .onFalse(coolNoteFixCommand(0.15).andThen(driverCamera.setShooterCameraCommand()));

        /* Smart Infeed */
        // driverController.leftTrigger()
        // .whileTrue(smartInfeedCommand().andThen(driverCamera.setShooterCameraCommand()));
        driverController.leftTrigger().whileTrue(runBoth(true, SLOW_CONVEYOR_VBUS, INFEED_VBUS))
                .onFalse(conveyBackCommand(-1.5, 0.5).alongWith(driverCamera.setShooterCameraCommand()));

        // ========================== //
        /* Drivetain & Vision Control */
        // ========================== //

        /* Robot-Relative Drive */
        driverController.y().toggleOnTrue(drivetrain.applyRequest(() -> robotRelativeDrive
                .withVelocityX(getXSpeed(false))
                .withVelocityY(getYSpeed(false))
                .withRotationalRate(getRotationSpeed())));

        /* X-Drive */
        driverController.x().whileTrue(drivetrain.applyRequest(() -> xDrive));

        /* Reset Field-Centric Heading */
        driverController.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative(new Pose2d())));

        /* Toggle Chassis Mode */
        driverController.rightBumper().onTrue(Commands.runOnce(() -> currentSpeed = SLOW_SPEED))
                .onFalse(Commands.runOnce(() -> currentSpeed = BASE_SPEED));

        /* Shooter Lock */
        driverController.leftStick().onTrue(magicLockCommand());

        // ========================= //
        /* Misc */
        // ========================= //

        /* End snap, limelight & stop all motors */
        driverController.rightStick().onTrue(stopAllCommand(true).alongWith(drivetrain.runOnce(() -> {
        })));

        driverController.a().onTrue(runEntryCommand(() -> PASSING_SHOT, () -> ShotSpeeds.FAST)
                .andThen(Commands.either(
                        snapCommand(SnapDirection.BluePass),
                        snapCommand(SnapDirection.RedPass),
                        () -> BeakUtils.allianceIsBlue())));

        // =================== //
        /* OPERATOR CONTROLLER */
        // =================== //

        // ========================= //
        /* Driver Help Control */
        // ========================= //

        /* Snap to Amp */
        operatorController.povUp().toggleOnTrue(snapCommand(SnapDirection.Left));

        // ======================= //
        /* Shooter Control */
        // ======================= //

        /* Spin Up Shooter */
        operatorController.leftTrigger()
                .onTrue(runEntryCommand(() -> ShooterTable.calcShooterTableEntry(Feet.of(currentIndex)),
                        () -> ShotSpeeds.FAST))
                .onFalse(stopAllCommand()
                        .andThen(Commands.runOnce(this::setCameraWithWait)));

        /* Convey Note */
        operatorController.rightTrigger()
                .whileTrue(runBoth(false, FAST_CONVEYOR_VBUS, SLOW_INFEED_VBUS));

        /* Magic Shoot */
        operatorController.x().toggleOnTrue(magicShootCommand());

        /* Magic Shoot Strategies */
        operatorController.povRight().onTrue(setStrategyCommand(shooterLimelightStrategy));
        operatorController.povDown().onTrue(setStrategyCommand(chassisLimelight2dStrategy)
                .alongWith(chassisLimelight.setPipelineCommand(TY_PIPELINE)));
        operatorController.povLeft().onTrue(setStrategyCommand(odometryStrategy)
                .alongWith(chassisLimelight.setPipelineCommand(MEGATAG_PIPELINE)));

        /* Manual/Preset Mode */
        operatorController.back()
                .onTrue(Commands.runOnce(() -> useManual = !useManual).andThen(this::pushIndexData));

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

        /* Shuttle Prep */
        operatorController.leftStick().onTrue(shuttleCommand());

        // ========================= //
        /* Pivot Control */
        // ========================= //

        /* Zero Pivot */
        operatorController.start().onTrue(zeroCommand());

        /* Run Pivot Zero */
        // operatorController.a().onTrue(pivot.runToHomeCommand());
        operatorController.a()
                .onTrue(Commands.runOnce(() -> selectedStrategy = odometryStrategy).andThen(shuttleCommand()));

        /* Zero Climber */
        operatorController.rightStick().onTrue(safeClimbCommand(climber.zeroCommand()));

        // ================ //
        /* Amp & Trap Magic */
        // ================ //

        operatorController.b().onTrue(ampPrep).onFalse(stopAllCommand(true));

        operatorController.y()
                .onTrue(toggleTrapCommand());

        // ==================== //
        /* EMERGENCY CONTROLLER */
        // ==================== //

        // ==================== //
        /* Manual Pivot Control */
        // ==================== //

        // /* Bump Pivot Up */
        emergencyController.rightBumper()
                .onTrue(pivot.runOnce(() -> pivot.runToPosition(pivot.getPosition() + 1)));

        /* Bump Pivot Down */
        emergencyController.leftBumper()
                .onTrue(pivot.runOnce(() -> pivot.runToPosition(pivot.getPosition() - 1)));

        // ============== //
        /* Manual Climber */
        // ============== //

        /* Climber Up */
        emergencyController.rightTrigger(0.2).whileTrue(
                safeClimbCommand(climber.runMotorCommand(CLIMBER_VBUS, true)))
                .onFalse(climber.stopCommand());

        /* Climber Down FULL SEND */
        emergencyController.leftTrigger(0.2).whileTrue(
                safeClimbCommand(climber.runMotorCommand(-FAST_CLIMBER_VBUS, true)))
                .onFalse(climber.holdCurrentPositionCommand());

        /* Ready Climb */
        emergencyController.povUp()
                .onTrue(safeClimbCommand(climber.runToPositionCommand(CLIMBER_VBUS,
                        ClimberPositions.READY, false)))
                .onFalse(climber.stopCommand());

        /* Climb */
        emergencyController.povDown()
                .onTrue(safeClimbCommand(climber.runToPositionCommand(CLIMBER_VBUS,
                        ClimberPositions.CLIMB, true)))
                .onFalse(climber.holdCurrentPositionCommand());

        /* funk */
        emergencyController.povLeft().onTrue(safeClimbCommand(climber.holdCommand()))
                .onFalse(climber.stopCommand());

        /* Stop Hold */
        emergencyController.povRight().onTrue(Commands.runOnce(() -> climber.getCurrentCommand().cancel()));

        // ======================= //
        /* Trap & Climb Sequencing */
        // ======================= //

        /* Enable Trap */
        emergencyController.start().onTrue(Commands.runOnce(() -> enableTrap = !enableTrap));

        /* Enable Climber */
        emergencyController.back().onTrue(Commands.runOnce(() -> enableClimber = !enableClimber));

        /* Climb Sequence */
        emergencyController.a().onTrue(sequenceCommand());
        emergencyController.b().onTrue(dumbCancelClimbCommand());

        /* Prime Fan Pivot & Shooter Pivot */
        emergencyController.x().toggleOnTrue(
                Commands.startEnd(() -> {
                    m_fanPivot.runToTrap();
                    m_fan.runMotor(FAN_VBUS);
                },
                        () -> {
                            m_fanPivot.hold();
                            m_fan.stop();
                        },
                        m_fanPivot, m_fan));

        // emergencyController.b().toggleOnTrue(coolShootCommand());
        // emergencyController.y()
        // .toggleOnTrue(magicShootCommand(() -> selectedStrategy, ztrue,
        // Rotation2d.fromDegrees(-4.0)));
        emergencyController.y().onTrue(toggleTrapCommand());

        /* Full Outfeed: left Y */
        emergencyController.axisGreaterThan(XboxController.Axis.kLeftY.value, 0.2)
                .or(emergencyController.axisLessThan(XboxController.Axis.kLeftY.value, -0.2))
                .whileTrue(
                        runThree(
                                () -> -emergencyController.getLeftY(),
                                () -> -emergencyController.getLeftY(),
                                () -> -emergencyController.getLeftY()));

        if (Utils.isSimulation()) {
            drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
        }

        drivetrain.registerTelemetry(logger::telemeterize);

    }

    //

    // =========================================== //
    /* Additional Commands, Getters, and Utilities */
    // =========================================== //

    //

    // =========================================== //
    /* Joystick & Driving Values */
    // =========================================== //

    //

    private double getRotationSpeed() {
        return scaleDriverController(-driverController.getRightX(),
                thetaLimiter, currentSpeed) *
                MAX_ANGULAR_SPEED;
    }

    private double getYSpeed(boolean flip) {
        return (flip ? getDriveSignum()
                : 1) * scaleDriverController(-driverController.getLeftX(), yLimiter, currentSpeed)
                * MAX_SPEED;
    }

    private double getXSpeed(boolean flip) {
        return (flip ? getDriveSignum()
                : 1) * scaleDriverController(-driverController.getLeftY(), xLimiter, currentSpeed)
                * MAX_SPEED;
    }

    /** Invert drivetrain based on alliance */
    private int getDriveSignum() {
        return BeakUtils.allianceIsBlue() ? 1 : -1;
    }

    /** Set Snap Direction Toggle */
    private Command snapCommand(SnapDirection direction) {
        return drivetrain.applyRequest(() -> snapDrive
                .withVelocityX(getXSpeed(true))
                .withVelocityY(getYSpeed(true))
                .withTargetDirection(Rotation2d.fromDegrees(direction.Angle)));
    }

    /** Joystick Scaling */
    private double scaleDriverController(double controllerInput, SlewRateLimiter limiter, double baseSpeedPercent) {
        return limiter.calculate(
                controllerInput * (baseSpeedPercent
                        + driverController.getRightTriggerAxis() * (1 - baseSpeedPercent)));
    }

    //

    // =========================================== //
    /** Climb Commands & Sequencing */
    // =========================================== //

    //

    /** Only run climb command if pivot good */
    private Command safeClimbCommand(Command command) {
        return command.onlyIf(() -> pivot.getPosition() > PIVOT_UP_THRESHOLD);
    }

    private Command safePivotCommand(Command command) {
        return command.onlyIf(climber::reverseLimitOn);
    }

    /** Iterate Climb Sequence */
    private void incrementSequence() {
        ClimbSequence seq = ClimbSequence.Default;

        switch (currentSequence) {
            case Default:
                seq = enableTrap || enableClimber ? ClimbSequence.Prep : ClimbSequence.Default;
                break;
            case Prep:
                seq = ClimbSequence.Fan;
                break;
            case Fan:
                seq = enableTrap ? ClimbSequence.Shoot : ClimbSequence.End;
                break;
            case Shoot:
                seq = ClimbSequence.End;
                break;
            case End:
                seq = enableClimber ? ClimbSequence.Climb : ClimbSequence.Default;
                break;
            default:
                seq = ClimbSequence.Default;
                break;
        }

        currentSequence = seq;
    }

    /** Climb Preparation */
    private Command prepClimbCommand() {
        // Pivot => Trap Position
        // Shooter => Trap Mode
        return pivot.runToTrapCommand()
                .alongWith(shooter.setSlotCommand(Slots.TRAP));
    }

    /** Prime the fan & shooter */
    private Command fanReadyCommand() {
        // Fan Pivot Up
        // Shooting Routine
        // Climber Up
        return m_fanPivot.runToTrapCommand()
                .alongWith(m_fan.runMotorCommand(FAN_VBUS)
                        .andThen(BeakCommands.repeatCommand(fixNoteCommand(), 2))
                        .andThen(shooter.runShotCommand(ShotSpeeds.TRAP))
                        .onlyIf(() -> enableTrap))
                .alongWith(
                        climber.runToPositionCommand(CLIMBER_VBUS, ClimberPositions.READY,
                                false)
                                .onlyIf(() -> enableClimber && pivot
                                        .getPosition() > PIVOT_UP_THRESHOLD));
    }

    private Command dumbCancelClimbCommand() {
        // Climber Down
        // Stops fan spinn
        // After 2s, Fan Down
        // Finally, Pivot Down
        return Commands.runOnce(() -> currentSequence = ClimbSequence.Default).andThen(m_fanPivot.runToTrapCommand()
                .andThen(safeClimbCommand(climber.zeroCommand().until(climber::reverseLimitOn))
                        .andThen(pivot.runToHomeCommand())
                        .alongWith(m_fan.stopCommand())
                        .alongWith(shooter.stopCommand())
                        .alongWith(Commands.waitSeconds(1).andThen(m_fanPivot.runToHomeCommand()))));
    }

    /** Shoot */
    private Command trapShootCommand() {
        // Shoot Note
        return conveyCommand();
    }

    /** Home everything */
    private Command endSequenceCommand() {
        // Fan Down
        // Fan Stop
        // Shooter Stop/Fast Mode
        // Pivot Down if not climbing
        return m_fanPivot.runToHomeCommand()
                .alongWith(m_fan.stopCommand())
                .alongWith(shooter.stopCommand().andThen(shooter.setSlotCommand(Slots.FAST)))
                .alongWith(safePivotCommand(pivot.runToHomeCommand()).unless(() -> enableClimber));
    }

    /** Climb!!!!! */
    private Command climbCommand() {
        // Climber Down
        return safeClimbCommand(climber.runToPositionCommand(CLIMBER_VBUS, ClimberPositions.CLIMB, true));
    }

    /** Update Sequence */
    private Command sequenceCommand() {
        return Commands.runOnce(this::incrementSequence)
                .andThen(Commands.select(sequenceCommandMap, () -> currentSequence));
    }

    private Command toggleTrapCommand() {
        return Commands.either(pivot.runToTrapCommand(),
                safeClimbCommand(climber.zeroCommand().until(climber::reverseLimitOn))
                        .andThen(pivot.runToHomeCommand()),
                pivot::getIsAtHome);
    }

    // =========================== //
    /* Auton Stuff that's gotta go */
    // =========================== //

    /** yee haw */
    private Command mirroredPathfindingShotCommand(double pivotAngle, Pose2d target, double scale,
            double endVelocity) {
        Pose2d redPose = new Pose2d(
                target.getTranslation(),
                target.getRotation().minus(Rotation2d.fromDegrees(6.)));

        return runBoth(false, FAST_CONVEYOR_VBUS, INFEED_VBUS).repeatedly()
                .alongWith(pivot.runToPositionCommand(pivotAngle))
                .alongWith(
                        Commands.either(
                                drivetrain.mirrorablePathFindCommand(target, scale,
                                        endVelocity),
                                drivetrain.mirrorablePathFindCommand(redPose, scale,
                                        endVelocity),
                                BeakUtils::allianceIsBlue));
    }

    /** Pathfinding Auton Shot */
    private Command pathfindingShotCommand(double targetDistance, Pose2d target, double scale, double endVelocity) {
        return drivetrain
                .mirrorablePathFindCommand(target, scale, endVelocity)
                .deadlineWith(smartInfeedCommand().withTimeout(0.6).andThen(coolNoteFixCommand(0.2))
                        .andThen(shooter.runShotCommand(ShotSpeeds.FAST)))
                .andThen(shootCommand(targetDistance));
    }

    // =========================== //
    /* Conveyance Sequences */
    // =========================== //

    /** Fix Note Sequence */
    private Command fixNoteCommand() {
        return runBoth(true, FAST_CONVEYOR_VBUS, INFEED_VBUS).withTimeout(0.25).andThen(
                conveyBackCommand(-2.0, 0.25));
    }

    /** Fix Note Backwards */
    private Command conveyBackCommand(double rotations, double timeout) {
        return shooter.spinMotorLeftCommand(SHOOTER_BACKOUT_VBUS).repeatedly()
                .raceWith(conveyor.runXRotations(rotations).withTimeout(timeout))
                .alongWith(infeed.runMotorCommand(0.))
                .andThen(shooter.stopCommand());
    }

    /** Special Note Fix */
    private Command coolNoteFixCommand(double timeout) {
        return shooter.spinMotorLeftCommand(SHOOTER_BACKOUT_VBUS).repeatedly()
                .alongWith(infeed.runMotorCommand(0.))
                .alongWith(conveyor.runMotorCommand(-0.2)).withTimeout(timeout)
                .andThen(shooter.stopCommand())
                .andThen(conveyor.brakeStopCommand());
    }

    /** Convey the Note */
    private Command conveyCommand() {
        return conveyor.runXRotations(20.).alongWith(infeed.runMotorCommand(SLOW_INFEED_VBUS));
    }

    /** Smart Infeed Command Generator */
    private Command smartInfeedCommand() {
        return runBoth(true, SLOW_CONVEYOR_VBUS, INFEED_VBUS)
                .until(noteSensing.hasInfedSupplier())
                .andThen(runBoth(true, 0., 0.).withTimeout(0.1))
                .andThen(conveyBackCommand(-4.0, 0.5))
                .finallyDo(shooter::stop);
    }

    private Command smartInfeedAutoCommand() {
        return runBoth(true, SLOW_CONVEYOR_VBUS, INFEED_VBUS)
                .until(noteSensing.hasInfedSupplier())
                .andThen(runBoth(true, 0., 0.).withTimeout(0.1))
                .andThen(conveyBackCommand(-4.0, 0.1))
                .finallyDo(shooter::stop);
    }

    /** Run both Conveyor and Infeed */
    private Command runBoth(boolean stopShooter, double conveyorVbus, double infeedVbus) {
        return shooter.brakeStopCommand().onlyIf(() -> stopShooter)
                .alongWith(infeed.runMotorCommand(infeedVbus)
                        .alongWith(conveyor.runMotorCommand(conveyorVbus)).repeatedly());
    }

    /** Run Conveyor, Infeed, and shooter if backwards */
    private Command runThree(Supplier<Double> conveyorVbus, Supplier<Double> infeedVbus,
            Supplier<Double> shooterVbus) {
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

    //

    // =========================== //
    /* Magic Shoot & Lock Sequences */
    // =========================== //

    //

    /**
     * Generate a command to continuously run the shooter while aligning to the
     * target.
     * 
     * @param strategy The {@link ShootingStrategy} to use.
     */
    private Command magicLockCommand(Supplier<ShootingStrategy> strategy) {
        return fixNoteCommand().unless(noteSensing.conveyorSeesNoteSupplier())
                .andThen(driverCamera.setShooterCameraCommand())
                .andThen(drivetrain.speakerLock(() -> getXSpeed(true), () -> getYSpeed(true), strategy)
                        .alongWith(runEntryCommand(() -> strategy.get().getTargetEntry(true),
                                () -> ShotSpeeds.FAST).repeatedly()))
                .finallyDo(driverCamera::setInfeedCamera);
    }

    /**
     * Generate a command to continuously run the shooter while aligning with the
     * selected strategy.
     */
    private Command magicLockCommand() {
        return magicLockCommand(() -> selectedStrategy);
    }

    private Command shuttleCommand() {
        return updateDrivePoseMT2Command()
                .andThen(drivetrain
                        .applyRequest(() -> snapDrive.withTargetDirection(
                                BeakUtils.passingTranslation(odometryStrategy
                                        .getDrivetrainFieldTranslation(true))
                                        .getAngle())
                                .withVelocityX(getXSpeed(true))
                                .withVelocityY(getYSpeed(true)))
                        .alongWith(runEntryCommand(
                                () -> ShooterTable.calcShuttleTableEntry(
                                        Meters.of(odometryStrategy
                                                .getDrivetrainGoalTranslation(
                                                        true)
                                                .getNorm())),
                                () -> ShotSpeeds.FAST))
                        .repeatedly());
    }

    /**
     * Generate a command to run the shooter, convey, & stop everything thereafter.
     * 
     * @param entry The entry to run.
     */
    private Command shootCommand(Supplier<ShooterTableEntry> entry) {
        return driverCamera.setShooterCameraCommand()
                .andThen(runEntryCommand(entry, () -> ShotSpeeds.FAST))
                .andThen(Commands.waitUntil(shooterAndPivotReady()))
                .andThen(conveyCommand().withTimeout(0.75))
                .finallyDo(() -> {
                    shooter.stop();
                    pivot.runToPosition(Pivot.HOLD_POSITION);
                    setCameraWithWait();
                });
    }

    /**
     * Generate a command to shoot based on a target distance.
     * 
     * @param distance The target distance, in feet.
     */
    private Command shootCommand(double distance) {
        return shootCommand(() -> ShooterTable.calcShooterTableEntry(Feet.of(distance)));
    }

    /**
     * Generate a command to use the specified entry to run a magic shot.
     * 
     * @param entry    The entry to run.
     * @param strategy The strategy to use for rotation.
     * @param lock     Whether or not to align the drivetrain.
     */
    private Command magicShootCommand(Supplier<ShooterTableEntry> entry, Supplier<ShootingStrategy> strategy,
            boolean lock, Rotation2d offset) {
        return Commands.runOnce(() -> entryToRun = entry.get())
                .andThen(fixNoteCommand().unless(noteSensing.conveyorSeesNoteSupplier()))
                .andThen(driverCamera.setShooterCameraCommand())
                .andThen(runEntryCommand(() -> entryToRun,
                        () -> ShotSpeeds.FAST)
                        .alongWith(drivetrain.speakerAlign(strategy, offset).withTimeout(0.5)
                                .unless(() -> Math.abs(strategy.get().getTargetOffset().getDegrees()) < 0.25))
                        .onlyIf(() -> lock))
                .andThen(shootCommand(() -> entryToRun));
    }

    /**
     * Generate a command to use the specified entry to run a magic shot.
     * 
     * @param entry    The entry to run.
     * @param strategy The strategy to use for rotation.
     * @param lock     Whether or not to align the drivetrain.
     */
    private Command magicShootCommand(Supplier<ShooterTableEntry> entry, Supplier<ShootingStrategy> strategy,
            boolean lock) {
        return magicShootCommand(entry, strategy, lock, ShootingStrategy.OFFSET);
    }

    /**
     * Generate a command to use the specified distance to run a magic shot.
     * 
     * @param distance The shot to run.
     * @param strategy The strategy to use for rotation.
     * @param lock     Whether or not to align the drivetrain.
     */
    private Command magicShootCommand(double distance, Supplier<ShootingStrategy> strategy,
            boolean lock) {
        return magicShootCommand(() -> ShooterTable.calcShooterTableEntry(Feet.of(distance)), strategy, lock);
    }

    /**
     * Generate a command to use the specified strategy to run a magic shot.
     * 
     * @param strategy The {@link ShootingStrategy} to use.
     * @param lock     Whether or not to align the drivetrain.
     */
    private Command magicShootCommand(Supplier<ShootingStrategy> strategy, boolean lock, Rotation2d offset) {
        return magicShootCommand(() -> strategy.get().getTargetEntry(), strategy, lock, offset);
    }

    /**
     * Generate a command to use the specified strategy to run a magic shot.
     * 
     * @param strategy The {@link ShootingStrategy} to use.
     * @param lock     Whether or not to align the drivetrain.
     */
    private Command magicShootCommand(Supplier<ShootingStrategy> strategy, boolean lock) {
        return magicShootCommand(() -> strategy.get().getTargetEntry(), strategy, lock, ShootingStrategy.OFFSET);
    }

    /**
     * Generate a command to use the specified strategy to run a magic, aligning
     * shot.
     * 
     * @param strategy The {@link ShootingStrategy} to use.
     */
    private Command magicShootCommand(Supplier<ShootingStrategy> strategy) {
        return magicShootCommand(strategy, true);
    }

    /**
     * Generate a command to use the specified strategy to run a magic, aligning
     * shot using the current strategy.
     */
    private Command magicShootCommand() {
        return magicShootCommand(() -> selectedStrategy, true);
    }

    /** Run a Shooter Table Entry */
    private Command runEntryCommand(Supplier<ShooterTableEntry> entry, Supplier<ShotSpeeds> speed) {
        return shooter.runEntryCommand(entry, speed)
                .alongWith(pivot.runToPositionCommand(() -> entry.get().Angle))
                .alongWith(Commands.runOnce(() -> m_lastShot = entry.get().Distance.in(Feet))
                        .onlyIf(() -> entry.get().Distance != null));
    }

    /** Shooter & Pivot Both Ready */
    private BooleanSupplier shooterAndPivotReady() {
        return () -> shooter.isReady() && pivot.inPosition();
    }

    /** Set the strategy to use for shooting/locking. */
    private Command setStrategyCommand(ShootingStrategy strategy) {
        return Commands.runOnce(() -> selectedStrategy = strategy);
    }

    //

    // ==================== //
    /* Stopping & Zeroing */
    // ==================== //

    //

    /** Stop all motors and zero everything */
    private Command stopAllCommand(boolean switchCamera) {
        return Commands.parallel(
                infeed.stopCommand(),
                conveyor.stopCommand(),
                shooter.stopCommand().andThen(shooter.setSlotCommand(Shooter.Slots.FAST)),
                pivot.runToHomeCommand(),
                m_fan.stopCommand(),
                m_fanPivot.runToHomeCommand(),
                whippy.stopCommand(),
                driverCamera.setInfeedCameraCommand().onlyIf(() -> switchCamera),
                Commands.runOnce(() -> currentSequence = ClimbSequence.Default));
    }

    /** That but don't reset the camera */
    private Command stopAllCommand() {
        return stopAllCommand(false);
    }

    /** Zeroing Command */
    public Command zeroCommand() {
        return pivot.zeroCommand().alongWith(m_fanPivot.runToHomeCommand());// .alongWith(climber.zeroCommand());
    }

    /** Asynchronous Zero */
    public void zero() {
        zeroCommand().schedule();
    }

    //

    // ====================== //
    /* Miscellaneous Stuff */
    // ====================== //

    //

    /** Set infeed camera asynchronously */
    private void setCameraWithWait() {
        Commands.waitSeconds(CAMERA_SWITCH_TIMEOUT).andThen(driverCamera.setInfeedCameraCommand()).schedule();
    }

    /** Stop Shooter */
    public void stopShooter() {
        shooter.stop();
    }

    /** Push limelight data to the CANdle */
    public Command encodeLimelights() {
        return candle.encodeLimelights(chassisLimelight, chassisLimelight, infeedLimelight3G);
    }

    /** Put Current ST Index Data to Dashboard */
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

    /** Auton Command */
    public Command getAutonomousCommand() {
        return new InstantCommand(() -> drivetrain.seedFieldRelative(new Pose2d()))
                .andThen(NamedCommands.getCommand("AprilTag Zero"))
                .andThen(autonChooser.getSelected());
    }

    // ================ //
    /** Vision Utilities */
    // ================ //

    // /** Return approx. 3d pose */
    // public Optional<EstimatedRobotPose> getBestPose() {
    // Pose2d drivetrainPose = drivetrain.getPose();

    // Optional<EstimatedRobotPose> front =
    // rightVision.getCameraResult(drivetrainPose);
    // Optional<EstimatedRobotPose> back =
    // leftVision.getCameraResult(drivetrainPose);

    // int numPoses = 0;

    // numPoses += front.isPresent() ? 1 : 0;
    // numPoses += back.isPresent() ? 1 : 0;

    // Optional<Pose2d> pose = Optional.empty();

    // if (numPoses == 1) {
    // pose = Optional
    // .of(new Pose2d((front.isEmpty() ? back :
    // front).get().estimatedPose.toPose2d()
    // .getTranslation(),
    // drivetrainPose.getRotation()));
    // } else if (numPoses == 2) {
    // // average the poses
    // Pose3d frontP = front.get().estimatedPose;
    // Pose3d backP = back.get().estimatedPose;

    // Translation2d frontT = frontP.getTranslation().toTranslation2d();
    // Translation2d backT = backP.getTranslation().toTranslation2d();

    // pose = Optional.of(
    // new Pose2d(frontT.plus(backT).div(2.),
    // drivetrainPose.getRotation()));
    // }

    // if (pose.isPresent()) {
    // return Optional.of(new EstimatedRobotPose(
    // new Pose3d(pose.get()
    // .plus(new Transform2d(Units.inchesToMeters(13.), 0.,
    // new Rotation2d()))),
    // (front.isEmpty() ? back : front).get().timestampSeconds,
    // null, null));
    // }

    // return Optional.empty();
    // }

    public void updateMTRot() {
        chassisLimelight.setRobotRotationMT2(drivetrain.getRotation().getDegrees());
        infeedLimelight3G.setRobotRotationMT2(drivetrain.getRotation().getDegrees());

    }

    public void updateDrivePoseMT2() {
        updateMTRot();

        // Apply Chassis Limelight
        var visionResult = chassisLimelight.getBotposeEstimateMT2();
        var visionStdDevs = chassisLimelight.getSTDevsXY(drivetrain);
        if (visionStdDevs.isPresent()) {
            // System.out.println("Current Pose: X: " + drivetrain.getPose().getX() + " --
            // Y: " + drivetrain.getPose().getY());
            // System.out.println("Vision Pose: X: " + visionResult.pose.getX() + " -- Y: "
            // + visionResult.pose.getY());
            // System.out.println("UPDATING POSE BY: " +
            // drivetrain.getTranslation().getDistance(visionResult.pose.getTranslation()));
            drivetrain.addVisionMeasurement(visionResult.pose, visionResult.timestampSeconds,
                    VecBuilder.fill(visionStdDevs.get()[0], visionStdDevs.get()[1], Double.MAX_VALUE));

        }

        // Apply Infeed Limelight
        visionResult = infeedLimelight3G.getBotposeEstimateMT2();
        visionStdDevs = infeedLimelight3G.getSTDevsXY(drivetrain);
        if (visionStdDevs.isPresent())
            drivetrain.addVisionMeasurement(visionResult.pose, visionResult.timestampSeconds,
                    VecBuilder.fill(visionStdDevs.get()[0], visionStdDevs.get()[1], Double.MAX_VALUE));
    }

    public Command updateDrivePoseMT2Command() {
        return Commands.runOnce(this::updateDrivePoseMT2);
    }

    public void setMT2Pipeline() {
        chassisLimelight.setPipeline(MEGATAG_PIPELINE);
        selectedStrategy = odometryStrategy;
    }

    public void setChassisPipeline() {
        selectedStrategy = chassisLimelight2dStrategy;
        chassisLimelight.setPipeline(TY_PIPELINE);
    }

    public void setTeleopMT2RotationThresholds() {
        chassisLimelight.setTeleopMT2Threshold();
        infeedLimelight3G.setTeleopMT2Threshold();
    }

    public void setAutonMT2RotationThresholds() {
        chassisLimelight.setAutonMT2Threshold();
        infeedLimelight3G.setAutonMT2Threshold();
    }

    public void homeFanPivot() {
        m_fanPivot.runToHomeCommand().schedule();
    }
}
