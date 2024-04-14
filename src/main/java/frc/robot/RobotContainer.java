// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Feet;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.SwerveDriveBrake;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
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
import frc.robot.commands.AlignDrivetrain;
import frc.robot.commands.vision.LimelightAcquire;
import frc.robot.commands.vision.AlignToSpeaker;
import frc.robot.commands.vision.SpeakerLockOn;
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
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.NoteSensing;
import frc.robot.utils.PhotonVision;
import frc.robot.utils.ShooterTable;
import frc.robot.utils.ShooterTable.ShooterTableEntry;
import frc.robot.utils.ShooterTable.VisionTableEntry.CameraLerpStrat;
import frc.robot.utils.ShootingStrategy;
import frc.robot.utils.VisionSystem;

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

    private static final String SHOOTER_LIMELIGHT = "limelight-ii";
    private static final String MEGA_LEFT_LIMELIGHT = "limelight-gii";
    private static final String MEGA_RIGHT_LIMELIGHT = "limelight-gi";

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

    private final PhotonVision rightVision = new PhotonVision("Right_AprilTag_Camera",
            VisionSystem.RIGHT_ROBOT_TO_CAMERA);
    private final PhotonVision leftVision = new PhotonVision("Left_AprilTag_Camera", VisionSystem.LEFT_ROBOT_TO_CAMERA);

    // private final Limelight infeedCamera = new Limelight("limelight", new
    // Transform3d());
    private final Limelight shooterLimelight = new Limelight(SHOOTER_LIMELIGHT, new Transform3d());

    private final Limelight megaLeftVision = new Limelight(MEGA_LEFT_LIMELIGHT, new Transform3d());
    private final Limelight megaRightVision = new Limelight(MEGA_RIGHT_LIMELIGHT, new Transform3d());

    // ====================== //
    /** Auton & Other Commands */
    // ====================== //
    private final Command ampPrep;
    private SendableChooser<Command> autonChooser;

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

    /* Shooting Strategies */
    private final ShootingStrategy shooterLimelightStrategy = new ShootingStrategy(shooterLimelight,
            CameraLerpStrat.LimelightTY);

    private final ShootingStrategy odometryStrategy = new ShootingStrategy(drivetrain);

    private final ShootingStrategy chassisLimelight2dStrategy = new ShootingStrategy(megaLeftVision,
            CameraLerpStrat.Limelight3GTY);

    private ShootingStrategy selectedStrategy = odometryStrategy;

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
        snapDrive.HeadingController = new PhoenixPIDController(3., 0., 0.);
        snapDrive.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

        /* Init Index Map */
        indexMap.put(4.2, "Speaker");
        indexMap.put(10.0, "Protected");
        indexMap.put(13.0, "Chain");
        indexMap.put(16.0, "Truss");
        indexMap.put(19.0, "Left Wing");
        indexMap.put(22.0, "Right Wing");

        ShooterTable.setHeckinessLevel(() -> drivetrain.getState().Pose.getRotation());

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
        DashboardStore.add("Trap Enabled", () -> enableTrap);

        DashboardStore.add("Sequence", () -> currentSequence.name());

        DashboardStore.add("Limelight Distance", () -> shooterLimelightStrategy.getTargetEntry().Distance.in(Feet));
        DashboardStore.add("LimelightG Distance", () ->  chassisLimelight2dStrategy.getTargetEntry().Distance.in(Feet));
        DashboardStore.add("Limelight Yaw", () -> LimelightHelpers.getTX(SHOOTER_LIMELIGHT));

        DashboardStore.add("Last Shot", () -> m_lastShot);
        DashboardStore.add("Odometry Distance",
                () -> Units.metersToFeet(BeakUtils.goalTranslation(drivetrain.getState().Pose).getNorm()));

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
        NamedCommands.registerCommand("Pivot Zero", pivot.zeroCommand());

        NamedCommands.registerCommand("AprilTag Zero",
                new InstantCommand(() -> drivetrain.addMeasurementCommand(this::getBestPose)));

        NamedCommands.registerCommand("Limelight Acquire",
                new LimelightAcquire(() -> 0.6,
                        drivetrain)
                        .until(noteSensing.hasInfedSupplier())
                        .raceWith(smartInfeedCommand().withTimeout(1.2)));

        /* Infeed & Spit */
        NamedCommands.registerCommand("Smart Infeed", smartInfeedCommand());

        NamedCommands.registerCommand("Dumb Infeed",
                runBoth(true, SLOW_CONVEYOR_VBUS, INFEED_VBUS).withTimeout(.25));

        NamedCommands.registerCommand("Infeed", infeed.runMotorCommand(INFEED_VBUS)
                .alongWith(conveyor.runMotorCommand(FAST_CONVEYOR_VBUS)).repeatedly());// .withTimeout(1.5));

        NamedCommands.registerCommand("First Spit Note", infeed.runMotorCommand(INFEED_VBUS)
                .alongWith(conveyor.runMotorCommand(FAST_CONVEYOR_VBUS))
                .alongWith(shooter.spinBothCommand(0.15))
                .repeatedly());

        NamedCommands.registerCommand("Spit Note", infeed.runMotorCommand(INFEED_VBUS)
                .alongWith(conveyor.runMotorCommand(FAST_CONVEYOR_VBUS))
                .alongWith(shooter.spinBothCommand(0.11))
                .repeatedly());

        NamedCommands.registerCommand("Prepare Spit", shooter.spinBothCommand(0.15));

        NamedCommands.registerCommand("Fix Note", fixNoteCommand());

        NamedCommands.registerCommand("Finish Infeed",
                smartInfeedAutoCommand().andThen(shooter.runShotCommand(ShotSpeeds.FAST)));

        NamedCommands.registerCommand("Magic Shoot", magicShootCommand(() -> chassisLimelight2dStrategy));
        NamedCommands.registerCommand("Mega Tag Shoot", magicShootCommand(() -> odometryStrategy));
        NamedCommands.registerCommand("Choose Shoot", magicShootCommand());

        /* Shooter & Pivot */
        NamedCommands.registerCommand("Fast Shooter",
                shooter.runShotCommand(ShotSpeeds.FAST));

        NamedCommands.registerCommand("Shooter",
                shooter.runShotCommand(ShotSpeeds.FAST, 0.85));

        NamedCommands.registerCommand("Stop Shooter", shooter.stopCommand());

        NamedCommands.registerCommand("Home Pivot", pivot.runToHomeCommand());

        NamedCommands.registerCommand("Rotate To Speaker Source PC3", new AlignDrivetrain(drivetrain,
                () -> drivetrain.getState().Pose.getRotation().getRadians(), () -> Units.degreesToRadians(
                        BeakUtils.allianceIsBlue() ? -63 : -117),
                false));

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
        NamedCommands.registerCommand("Sad Stationary Amp Shot 2", shootCommand(16.5));
        NamedCommands.registerCommand("Epic Amp Shot", shootCommand(7.5)); // 7.5

        NamedCommands.registerCommand("Center Pathfinding Shot", pathfindingShotCommand(
                13.0, Constants.CENTER_SHOT, 0.8, 0.));

        // NamedCommands.registerCommand("Source Shot",
        // pathfindingShotCommand(21.1, Constants.RIGHT_SHOT, 0.75, 0.));

        NamedCommands.registerCommand("Stationary Source Shot", shootCommand(22.1));

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
        NamedCommands.registerCommand("Stationary Shot B", shootCommand(9));
        NamedCommands.registerCommand("Stationary Shot A", shootCommand(9));
        NamedCommands.registerCommand("Stationary Shot C", shootCommand(9));
        NamedCommands.registerCommand("P Amp Shot", shootCommand(13.5));
        NamedCommands.registerCommand("Stationary Shot Amp", shootCommand(13.5));
        NamedCommands.registerCommand("Source Pivot", pivot.runToPositionCommand(4.75));
        NamedCommands.registerCommand("Convey", conveyCommand());
        NamedCommands.registerCommand("Amp Pivot", pivot.runToPositionCommand(9));
        NamedCommands.registerCommand("4 Piece Pivot", pivot.runToPositionCommand(14));
    }

    // =========================== //
    /** Bindings & Default Commands */
    // =========================== //
    private void configureBindings() {

        /* Climber Limit Switch Triggers */
        new Trigger(climber::forwardLimit).onTrue(climber.hitForwardLimitCommand());
        new Trigger(climber::reverseLimit).onTrue(climber.hitReverseLimitCommand());

        // // LED Triggers //
        // // infeed has jam
        // new Trigger(conveyor.hasJamSupplier()).onTrue(CANdle.blink(Color.ORANGE, 5))
        // .onFalse(CANdle.runBurnyBurnCommand());
        // // checks if it has a note
        // new Trigger(conveyor.hasInfedSupplier()).onTrue(CANdle.blink(Color.GREEN, 5))
        // .onFalse(CANdle.runBurnyBurnCommand());
        // // flashes purple
        // new Trigger(shooter.isReadySupplier()).onTrue(CANdle.blink(Color.PURPLE, 3))
        // .onFalse(CANdle.runBurnyBurnCommand())
        // // Flashes white while shooting
        // new
        // Trigger(shooter.isRunningSupplier()).onTrue(CANdle.runShootFlow(Color.WHITE))
        // .onFalse(CANdle.runBurnyBurnCommand());
        // // Turns red while the shooter is inoperable
        // new Trigger(shooter.isWorkingSupplier())
        // .onTrue(CANdle.runBurnyBurnCommand())
        // .onFalse(CANdle.blink(Color.RED, 10));

        // ================ //
        /* Default Commands */
        // ================ //

        drivetrain.setDefaultCommand(
                drivetrain.applyRequest(() -> drive
                        .withVelocityX(getXSpeed(true).getAsDouble())
                        .withVelocityY(getYSpeed(true).getAsDouble())
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
        driverController.leftTrigger()
                .whileTrue(smartInfeedCommand().andThen(driverCamera.setShooterCameraCommand()));

        // ========================== //
        /* Drivetain & Vision Control */
        // ========================== //

        /* Robot-Relative Drive */
        driverController.y().toggleOnTrue(drivetrain.applyRequest(() -> robotRelativeDrive
                .withVelocityX(getXSpeed(false).getAsDouble())
                .withVelocityY(getYSpeed(false).getAsDouble())
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
        operatorController.povRight().onTrue(Commands.runOnce(() -> selectedStrategy = shooterLimelightStrategy));
        operatorController.povDown().onTrue(Commands.runOnce(() -> selectedStrategy = chassisLimelight2dStrategy));
        operatorController.povLeft().onTrue(Commands.runOnce(() -> selectedStrategy = odometryStrategy));

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
        /* Pivot Control */
        // ========================= //

        /* Zero Pivot */
        operatorController.start().onTrue(zeroCommand());

        /* Run Pivot Zero */
        operatorController.a().onTrue(pivot.runToHomeCommand());

        /* Zero Climber */
        operatorController.rightStick().onTrue(safeClimbCommand(climber.zeroCommand()));

        // ================ //
        /* Amp & Trap Magic */
        // ================ //

        operatorController.b().onTrue(ampPrep).onFalse(stopAllCommand(true));

        operatorController.y()
                .onTrue(pivot.runToTrapCommand());

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
                .onTrue(safeClimbCommand(climber.runToPositionCommand(CLIMBER_VBUS, ClimberPositions.READY, false)))
                .onFalse(climber.stopCommand());

        /* Climb */
        emergencyController.povDown()
                .onTrue(safeClimbCommand(climber.runToPositionCommand(CLIMBER_VBUS, ClimberPositions.CLIMB, true)))
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

        /* Prime Fan Pivot & Shooter Pivot */
        emergencyController.x().toggleOnTrue(
                Commands.startEnd(() -> {
                    m_fanPivot.runToPosition(FanPivot.TRAP_POSITION);
                    m_fan.runMotor(FAN_VBUS);
                },
                        () -> {
                            m_fanPivot.runToPosition(0.0);
                            m_fan.stop();
                        },
                        m_fanPivot, m_fan));

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

    // =========================================== //
    /** Additional Commands, Getters, and Utilities */
    // =========================================== //

    //

    // =========================================== //
    /** Joystick & Driving Values */
    // =========================================== //

    //

    private double getRotationSpeed() {
        return scaleDriverController(-driverController.getRightX(),
                thetaLimiter, currentSpeed) *
                MAX_ANGULAR_SPEED;
    }

    private DoubleSupplier getYSpeed(boolean flip) {
        return () -> (flip ? getDriveSignum()
                : 1) * scaleDriverController(-driverController.getLeftX(), yLimiter, currentSpeed) * MAX_SPEED;
    }

    private DoubleSupplier getXSpeed(boolean flip) {
        return () -> (flip ? getDriveSignum()
                : 1) * scaleDriverController(-driverController.getLeftY(), xLimiter, currentSpeed) * MAX_SPEED;
    }

    /** Invert drivetrain based on alliance */
    private int getDriveSignum() {
        return BeakUtils.allianceIsBlue() ? 1 : -1;
    }

    /** Set Snap Direction Toggle */
    private Command snapCommand(SnapDirection direction) {
        return drivetrain.applyRequest(() -> snapDrive
                .withVelocityX(getXSpeed(true).getAsDouble())
                .withVelocityY(getYSpeed(true).getAsDouble())
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

    /** Iterate Climb Sequence */
    private void updateSequence() {
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
        return pivot.runToTrapCommand()
                .alongWith(shooter.setSlotCommand(Slots.TRAP));
    }

    /** Prime the fan & shooter */
    private Command fanReadyCommand() {
        return m_fanPivot.runToTrapCommand()
                .alongWith(m_fan.runMotorCommand(FAN_VBUS)
                        .andThen(BeakCommands.repeatCommand(fixNoteCommand(), 2))
                        .andThen(shooter.runShotCommand(ShotSpeeds.TRAP)).onlyIf(() -> enableTrap))
                .alongWith(
                        climber.runToPositionCommand(CLIMBER_VBUS, ClimberPositions.READY, false)
                                .onlyIf(() -> enableClimber && pivot.getPosition() > PIVOT_UP_THRESHOLD));
    }

    /** Shoot */
    private Command trapShootCommand() {
        return conveyCommand();
    }

    /** Home everything */
    private Command endSequenceCommand() {
        return m_fanPivot.runToPositionCommand(0.0)
                .alongWith(m_fan.stopCommand())
                .alongWith(shooter.stopCommand().andThen(shooter.setSlotCommand(Slots.FAST)))
                .alongWith(pivot.runToHomeCommand().unless(() -> enableClimber));
    }

    /** Climb!!!!! */
    private Command climbCommand() {
        return safeClimbCommand(climber.runToPositionCommand(CLIMBER_VBUS, ClimberPositions.CLIMB, true));
    }

    /** Update Sequence */
    private Command sequenceCommand() {
        return Commands.runOnce(this::updateSequence)
                .andThen(Commands.select(sequenceCommandMap, () -> currentSequence));
    }

    // =========================== //
    /* Auton Stuff that's gotta go */
    // =========================== //

    /** yee haw */
    private Command mirroredPathfindingShotCommand(double pivotAngle, Pose2d target, double scale, double endVelocity) {
        Pose2d redPose = new Pose2d(
                target.getTranslation(),
                target.getRotation().minus(Rotation2d.fromDegrees(6.)));

        return runBoth(false, FAST_CONVEYOR_VBUS, INFEED_VBUS).repeatedly()
                .alongWith(pivot.runToPositionCommand(pivotAngle))
                .alongWith(
                        Commands.either(
                                drivetrain.mirrorablePathFindCommand(target, scale, endVelocity),
                                drivetrain.mirrorablePathFindCommand(redPose, scale, endVelocity),
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
                conveyBackCommand(-2.0, 0.1));
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
        return driverCamera.setShooterCameraCommand()
                .andThen(new SpeakerLockOn(drivetrain, getXSpeed(true), getYSpeed(true), strategy.get()))
                .alongWith(runEntryCommand(strategy.get()::getTargetEntry, () -> ShotSpeeds.FAST).repeatedly())
                .finallyDo(driverCamera::setInfeedCamera);
    }

    /**
     * Generate a command to continuously run the shooter while aligning with the
     * selected strategy.
     */
    private Command magicLockCommand() {
        return magicLockCommand(() -> selectedStrategy);
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
                .andThen(conveyCommand())
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
     * Generate a command to use the specified strategy to run a magic shot.
     * 
     * @param strategy The {@link ShootingStrategy} to use.
     * @param lock     Whether or not to align the drivetrain.
     */
    private Command magicShootCommand(Supplier<ShootingStrategy> strategy, boolean lock) {
        return driverCamera.setShooterCameraCommand()
                .andThen(runEntryCommand(strategy.get()::getTargetEntry,
                        () -> ShotSpeeds.FAST)
                        .alongWith(new AlignToSpeaker(drivetrain, strategy.get()).withTimeout(0.5))
                        .onlyIf(() -> lock))
                .andThen(shootCommand(strategy.get()::getTargetEntry));
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
                m_fanPivot.runToPositionCommand(0.),
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
        return pivot.zeroCommand().alongWith(m_fanPivot.runToPositionCommand(0.));// .alongWith(climber.zeroCommand());
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
        return candle.encodeLimelights(shooterLimelight, megaLeftVision, megaRightVision);
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

    // ======= //
    /** Logging */
    // ======= //
    public void logValues() {
        drivetrain.logValues();
        conveyor.logValues();
        infeed.logValues();
        shooter.logValues();
        climber.logValues();
        pivot.logValues();
        m_fan.logValues();
    }

    // ================ //
    /** Vision Utilities */
    // ================ //

    /** Return approx. 3d pose */
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

    public void updateMTRot() {
        megaLeftVision.setRobotRotationMT2(drivetrain.getState().Pose.getRotation().getDegrees());
        megaRightVision.setRobotRotationMT2(drivetrain.getState().Pose.getRotation().getDegrees());
    }

    public void updateDrivePoseMT2() {
        updateMTRot();

        if (Math.abs(drivetrain.getCurrentRobotChassisSpeeds().omegaRadiansPerSecond) > 0.33) {
            return;
        }

        var llLeftPoseEst = megaLeftVision.getBotposeEstimateMT2();
        var llRightPoseEst = megaRightVision.getBotposeEstimateMT2();
        Pose2d llAvgPose;

        if (llLeftPoseEst.tagCount <= 0 && llRightPoseEst.tagCount <= 0) {
            return;
        } else if (llLeftPoseEst.tagCount <= 0) {
            llAvgPose = new Pose2d(llRightPoseEst.pose.getTranslation(), drivetrain.getState().Pose.getRotation());
        } else if (llRightPoseEst.tagCount <= 0) {
            llAvgPose = new Pose2d(llLeftPoseEst.pose.getTranslation(), drivetrain.getState().Pose.getRotation());
        } else {
            llAvgPose = new Pose2d(
                    llLeftPoseEst.pose.getTranslation().plus(llRightPoseEst.pose.getTranslation()).div(2.),
                    drivetrain.getState().Pose.getRotation());
        }

        double llAvgTimestamp = (llLeftPoseEst.timestampSeconds + llRightPoseEst.timestampSeconds) / 2;
        drivetrain.addVisionMeasurement(llAvgPose, llAvgTimestamp);
    }

    public Command updateDrivePoseMT2Command() {
        return Commands.runOnce(this::updateDrivePoseMT2);
    }
}
