// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Meters;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
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
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AlignDrivetrain;
import frc.robot.commands.RotateToSpeaker;
import frc.robot.commands.vision.LimelightAcquire;
import frc.robot.commands.vision.ShooterAlign;
import frc.robot.commands.vision.ShooterAlignEpic;
import frc.robot.commands.vision.ShooterAlignStrafe;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climber;
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
import frc.robot.subsystems.Climber.ClimberPositions;
import frc.robot.utils.BeakCommands;
import frc.robot.utils.DashboardStore;
import frc.robot.utils.DriverCamera;
import frc.robot.utils.Limelight;
import frc.robot.utils.EPICShooterTable;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.NoteSensing;
import frc.robot.utils.PhotonVision;
import frc.robot.utils.ShooterTable;
import frc.robot.utils.VisionSystem;
import frc.robot.utils.ShooterTable.ShooterTableEntry;
import frc.robot.utils.ShooterTable.VisionTableEntry.CameraLerpStrat;

public class RobotContainer {
    // =============================================== //
    /* Magic numbers, Vbus constants, and OI constants */
    // =============================================== //
    private static final double CLIMBER_VBUS = 0.75;
    private static final double FAST_CLIMBER_VBUS = 0.95;
    private static final double INFEED_VBUS = 0.8;
    private static final double SLOW_INFEED_VBUS = 0.5;

    // private static final double PIVOT_VBUS = 0.15;
    private static final double SLOW_CONVEYOR_VBUS = 0.5;
    private static final double FAST_CONVEYOR_VBUS = 0.85;

    private static final double FAN_VBUS = 1.;
    // private static final double FAN_PIVOT_VBUS = 0.2;

    private static final double SHOOTER_BACKOUT_VBUS = -0.2;
    private static final double WHIPPY_VBUS = 0.2;

    private static final int OI_DRIVER_CONTROLLER = 0;
    private static final int OI_OPERATOR_CONTROLLER = 1;
    private static final int OI_EMERGENCY_CONTROLLER = 2;

    private final StringLogEntry autonPhase;

    private static final int SHOOTING_PIPELINE = 0;
    // private static final int TRAP_PIPELINE = 2;

    private static final String SHOOTER_LIMELIGHT = "limelight-iii";
    private static final String MEGA_LEFT_LIMELIGHT = "limelight-gii";
    private static final String MEGA_RIGHT_LIMELIGHT = "limelight-gi";

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
    private final Pivot pivot = new Pivot();
    private final Fan m_fan = new Fan();
    private final FanPivot m_fanPivot = new FanPivot();
    private final Whippy whippy = new Whippy();

    // private final Candle CANdle = new Candle();
    private final NoteSensing noteSensing = new NoteSensing();
    private final DriverCamera driverCamera = new DriverCamera();

    private final PhotonVision rightVision = new PhotonVision("Right_AprilTag_Camera",
            VisionSystem.RIGHT_ROBOT_TO_CAMERA);
    private final PhotonVision leftVision = new PhotonVision("Left_AprilTag_Camera", VisionSystem.LEFT_ROBOT_TO_CAMERA);
    private final PhotonVision shooterVision = new PhotonVision("Shooter-OV2311", VisionSystem.SHOOTER_ROBOT_TO_CAMERA);
    private final PhotonVision stationaryVision = new PhotonVision("Stationary-OV2311-Camera",
            VisionSystem.STATIONARY_ROBOT_TO_CAMERA);

    private final Limelight infeedCamera = new Limelight("limelight", new Transform3d());
    private final Limelight shooterLimelight = new Limelight(SHOOTER_LIMELIGHT, new Transform3d());

    private final Limelight megaLeftVision = new Limelight(MEGA_LEFT_LIMELIGHT, new Transform3d());
    private final Limelight megaRightVision = new Limelight(MEGA_RIGHT_LIMELIGHT, new Transform3d());

    public Conveyor getConveyor() {
        return conveyor;
    }

    // ====================== //
    /* Auton & Other Commands */
    // ====================== //
    private final Command ampPrep;
    private SendableChooser<Command> autonChooser;

    private static final double MAGIC_LOCK_DISTANCE_AMBIGUITY_THRESHOLD = 2;
    private double magicLockLastPos = Double.NaN;

    private short autonPathIncrement = 0;

    // ====================================================== //
    /* Drivetrain Constants, Magic numbers, and ` Limiters */
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

    // private boolean isSnappedToSpeaker = false;
    // private boolean isInMagicShoot = false;

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
    private static final double MAGIC_SHOOT_TIMER_THRESHOLD = 5.0;

    private static ShooterTableEntry PASSING_SHOT = new ShooterTableEntry(Feet.zero(), 4, 0.69);// 30.9;

    private final LinkedHashMap<Double, String> indexMap = new LinkedHashMap<>();

    private double currentIndex = MIN_INDEX;
    private double manualIndex = MIN_INDEX;
    private int presetIndex = 0;
    private boolean useManual = false;

    private boolean enableClimber = false;
    private boolean enableTrap = false;

    private double m_lastShot = 0.0;

    private double previousLimelightDistance = 20.0;
    private final Timer m_limelightTimer = new Timer();

    // ======================== //
    /* Swerve Control & Logging */
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

    /* LL */
    // private static final double SHOOTER_CAM_PITCH =
    // Units.degreesToRadians(36.15); // 32. //-4.65 ??
    // private static final double SHOOTER_CAM_HEIGHT =
    // Units.inchesToMeters(13.125); // 12.375
    // private static final double SPEAKER_TAG_HEIGHT =
    // Units.inchesToMeters(57.125);

    public RobotContainer() {
        // shooterVision.s?etPipeline(Vision.SHOOTER_PIPELINE_INDEX);

        autonPhase = new StringLogEntry(DataLogManager.getLog(), "Auton Phase");

        snapDrive.HeadingController = new PhoenixPIDController(3., 0., 0.);
        snapDrive.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

        /* Init Index Map */
        indexMap.put(4.2, "Speaker");
        indexMap.put(10.0, "Protected");
        indexMap.put(13.0, "Chain");
        indexMap.put(16.0, "Truss");
        indexMap.put(19.0, "Left Wing");
        indexMap.put(22.0, "Right Wing");

        /* Dashboard */
        DashboardStore.add("Shooter Table Index", () -> currentIndex);
        DashboardStore.add("Shooter Table Name",
                () -> indexMap.containsKey(currentIndex) ? indexMap.get(currentIndex) : "Manual");

        DashboardStore.add("Stationary Distance",
                () -> {
                    Optional<Double> distance = stationaryVision.getTagDistance(7);
                    return distance.isPresent() ? distance.get()
                            : Double.NaN;
                });

        // this is weird
        DashboardStore.add("Snapped", () -> drivetrain.getCurrentRequest().getClass().equals(snapDrive.getClass()));
        DashboardStore.add("Robot Relative",
                () -> drivetrain.getCurrentRequest().getClass().equals(robotRelativeDrive.getClass()));

        DashboardStore.add("Manual Indexing", () -> useManual);
        DashboardStore.add("Climber Enabled", () -> enableClimber);
        DashboardStore.add("Trap Enabled", () -> enableTrap);

        DashboardStore.add("Sequence", () -> currentSequence.name());

        DashboardStore.add("Limelight Distance", () -> getBestSTEntryLLY().Distance.in(Feet));
        DashboardStore.add("Limelight Yaw", () -> LimelightHelpers.getTX(SHOOTER_LIMELIGHT));

        DashboardStore.add("Last Shot", () -> m_lastShot);
        DashboardStore.add("Odometry Distance", () -> getBestSTEntry().Distance.in(Feet));

        initNamedCommands();

        initAutonChooser();

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

        SmartDashboard.putData("Auto Chooser", autonChooser);
    }

    private void initNamedCommands() {
        NamedCommands.registerCommand("Pivot Zero", pivot.zeroCommand());

        NamedCommands.registerCommand("AprilTag Zero",
                new InstantCommand(() -> drivetrain.addMeasurementCommand(this::getBestPose))
                        .andThen(appendAutonPhase("AprilTag Zeroed")));

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
                smartInfeedCommand().andThen(shooter.runShotCommand(ShotSpeeds.FAST)));

        NamedCommands.registerCommand("Magic Shoot", fastMagicShootCommand());

        NamedCommands.registerCommand("Finnish Path", appendAutonPhase("Finished path " + autonPathIncrement++));

        /* Shooter & Pivot */
        NamedCommands.registerCommand("Fast Shooter",
                shooter.runShotCommand(ShotSpeeds.FAST));

        NamedCommands.registerCommand("Shooter",
                shooter.runShotCommand(ShotSpeeds.FAST, 0.85));

        NamedCommands.registerCommand("Stop Shooter", shooter.stopCommand());

        NamedCommands.registerCommand("Home Pivot", pivot.runToHomeCommand());

        NamedCommands.registerCommand("Rotate To Speaker Source PC3", new AlignDrivetrain(drivetrain,
                () -> drivetrain.getState().Pose.getRotation().getRadians(), () -> Units.degreesToRadians(
                        allianceIsBlue() ? -63 : -117),
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

        NamedCommands.registerCommand("Stationary Amp Shot", stationaryShot(17.0));
        NamedCommands.registerCommand("Sad Stationary Amp Shot 2", stationaryShot(16.5));
        NamedCommands.registerCommand("Epic Amp Shot", stationaryShot(7.5)); // 7.5

        NamedCommands.registerCommand("Center Pathfinding Shot", pathfindingShotCommand(
                13.0, Constants.CENTER_SHOT, 0.8, 0.));

        NamedCommands.registerCommand("Source Shot",
                pathfindingShotCommand(21.1, Constants.RIGHT_SHOT, 0.75, 0.));

        NamedCommands.registerCommand("Stationary Source Shot", stationaryShot(22.1));

        NamedCommands.registerCommand("Spitless Source Shot 1", stationaryShotNoPV(15.8));
        NamedCommands.registerCommand("Spitless Source Shot 2", stationaryShotNoPV(17));
        NamedCommands.registerCommand("Spitless Source Shot 3", stationaryShotNoPV(17.2));

        NamedCommands.registerCommand("Right Preload", pivot.runOnce(pivot::zeroEncoder)
                .andThen(shotSequence(() -> ShooterTable.calcShooterTableEntry(Feet.of(5.2)))));

        NamedCommands.registerCommand("Center Preload", pivot.runOnce(pivot::zeroEncoder)
                .andThen(shotSequence(() -> ShooterTable.calcShooterTableEntry(Feet.of(4.2)))));

        NamedCommands.registerCommand("Note C Pathfinding",
                mirroredPathfindingShotCommand(12.35, Constants.NOTE_C_SHOT, 0.85, 0.));
        // PBAC Auto Commands
        NamedCommands.registerCommand("Preload Stationary", stationaryShot(4.2));
        NamedCommands.registerCommand("Stationary Shot B", stationaryShot(14.625));
        NamedCommands.registerCommand("Stationary Shot A", stationaryShot(11));
        NamedCommands.registerCommand("Stationary Shot C", stationaryShot(13.5));
        NamedCommands.registerCommand("Source Pivot", pivot.runToPositionCommand(8));
        NamedCommands.registerCommand("Convey", conveyCommand());
    }

    // =========================== //
    /* Bindings & Default Commands */
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
                        .withVelocityX(
                                getDriveSignum()
                                        * scaleDriverController(-driverController.getLeftY(),
                                                xLimiter,
                                                currentSpeed)
                                        * MAX_SPEED)
                        .withVelocityY(getDriveSignum()
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
                .withVelocityX(
                        scaleDriverController(-driverController.getLeftY(),
                                xLimiter,
                                currentSpeed)
                                * MAX_SPEED)
                .withVelocityY(
                        scaleDriverController(-driverController.getLeftX(),
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
        driverController.rightStick().onTrue(stopAllCommand(true).alongWith(drivetrain.runOnce(() -> {
        })));

        // driverController.leftStick().whileTrue(new ShooterAlignWhileStrafing(
        // () -> (DriverStation.getAlliance().isPresent() &&
        // DriverStation.getAlliance().get() == Alliance.Red ? -1
        // : 1) * scaleDriverController(-driverController.getLeftX(), yLimiter,
        // currentSpeed) * MAX_SPEED,
        // drivetrain, trapVision).alongWith(new InstantCommand(() -> isSnappedToSpeaker
        // = true))
        // .andThen(new InstantCommand(() -> isSnappedToSpeaker = false)));

        driverController.leftStick().whileTrue(new ShooterAlign(drivetrain, shooterLimelight));

        driverController.a().onTrue(runEntryCommand(() -> PASSING_SHOT, () -> ShotSpeeds.FAST)
                .andThen(Commands.either(
                        snapCommand(SnapDirection.BluePass),
                        snapCommand(SnapDirection.RedPass),
                        () -> allianceIsBlue())));

        driverController.povUp().whileTrue(new RotateToSpeaker(drivetrain));

        // =================== //
        /* OPERATOR CONTROLLER */
        // =================== //

        // ========================= //
        /* Driver Help Control */
        // ========================= //

        /* Snap to Amp */
        operatorController.povUp().toggleOnTrue(snapCommand(SnapDirection.Left));

        // /* Limelight Square */
        // operatorController.leftStick().toggleOnTrue(new LimelightSquare(true,
        // () -> scaleDriverController(-driverController.getLeftY(), xLimeAquireLimiter,
        // currentSpeed) * MAX_SPEED,
        // () -> scaleDriverController(-driverController.getLeftX(), yLimeAquireLimiter,
        // currentSpeed) * MAX_SPEED,
        // drivetrain));

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
        // operatorController.povRight().toggleOnTrue(magicShootCommand()
        // .andThen(Commands.runOnce(this::setCameraWithWait)));
        operatorController.povRight().toggleOnTrue(magicLockCommand());

        /* Other Epic Magic Shoot Woohoo Wah Wah */
        operatorController.povDown().toggleOnTrue(megaTag2ShootCommand());
        operatorController.x().toggleOnTrue(fastMagicShootCommand());
        operatorController.povLeft().toggleOnTrue(magicShootNoLockCommand());

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
                .onTrue(pivot.runOnce(() -> pivot.runToPosition(pivot.getPosition() + 2)));

        /* Bump Pivot Down */
        emergencyController.leftBumper()
                .onTrue(pivot.runOnce(() -> pivot.runToPosition(pivot.getPosition() - 2)));

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

        /* Cool Outfeed: right Y */
        emergencyController.rightStick().onTrue(disengageClimberCommand());

        if (Utils.isSimulation()) {
            drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
        }

        drivetrain.registerTelemetry(logger::telemeterize);

    }

    // =========================================== //
    /* Additional Commands, Getters, and Utilities */
    // =========================================== //

    // /* Megatag epicnmess */
    // public void addMegaTagPose() {
    // LimelightHelpers.SetRobotOrientation(
    // SHOOTER_LIMELIGHT, drivetrain.getState().Pose.getRotation().getDegrees(), 0,
    // 0, 0, 0, 0);

    // var mt2 =
    // LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(SHOOTER_LIMELIGHT);

    // boolean rejectUpdate = false;
    // if (mt2.tagCount == 0)
    // rejectUpdate = true;

    // if (!rejectUpdate)
    // drivetrain.addVisionMeasurement(mt2.pose, mt2.timestampSeconds);
    // }

    /* Start LL Timer */
    public void startTimer() {
        m_limelightTimer.start();
    }

    /* Set infeed camera asynchronously */
    private void setCameraWithWait() {
        Commands.waitSeconds(CAMERA_SWITCH_TIMEOUT).andThen(driverCamera.setInfeedCameraCommand()).schedule();
    }

    /* Disengage Climber & Fix Things */
    private Command disengageClimberCommand() {
        return climber.runToPositionCommand(CLIMBER_VBUS, ClimberPositions.DISENGAGE, false)
                .andThen(Commands.waitSeconds(0.2))
                .andThen(stopAllCommand());
    }

    /* Invert drivetrain based on alliance */
    private int getDriveSignum() {
        return allianceIsBlue() ? 1 : -1;
    }

    /* Only run climb command if pivot good */
    private Command safeClimbCommand(Command command) {
        return command.onlyIf(() -> pivot.getPosition() > PIVOT_UP_THRESHOLD);
    }

    /* Check if we're blue */
    private boolean allianceIsBlue() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        return alliance.isPresent() && alliance.get() == Alliance.Blue;
    }

    /* Stop Shooter */
    public void stopShooter() {
        shooter.stop();
    }

    /* Climb Sequence */
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

    private Command prepClimbCommand() {
        return pivot.runToTrapCommand()
                .alongWith(shooter.setSlotCommand(Slots.TRAP));
    }

    private Command fanReadyCommand() {
        return m_fanPivot.runToTrapCommand()
                .alongWith(m_fan.runMotorCommand(FAN_VBUS)
                        .andThen(BeakCommands.repeatCommand(fixNoteCommand(), 2))
                        .andThen(shooter.runShotCommand(ShotSpeeds.TRAP)).onlyIf(() -> enableTrap))
                .alongWith(
                        climber.runToPositionCommand(CLIMBER_VBUS, ClimberPositions.READY, false)
                                .onlyIf(() -> enableClimber && pivot.getPosition() > PIVOT_UP_THRESHOLD));
    }

    private Command trapShootCommand() {
        return conveyCommand();
    }

    private Command endSequenceCommand() {
        return m_fanPivot.runToPositionCommand(0.0)
                .alongWith(m_fan.stopCommand())
                .alongWith(shooter.stopCommand().andThen(shooter.setSlotCommand(Slots.FAST)))
                .alongWith(pivot.runToHomeCommand().unless(() -> enableClimber));
    }

    private Command climbCommand() {
        return safeClimbCommand(climber.runToPositionCommand(CLIMBER_VBUS, ClimberPositions.CLIMB, true));
    }

    private Command sequenceCommand() {
        return Commands.runOnce(this::updateSequence)
                .andThen(Commands.select(sequenceCommandMap, () -> currentSequence));
    }

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
                                this::allianceIsBlue));
    }

    /* Pathfinding Auton Shot */
    private Command pathfindingShotCommand(double targetDistance, Pose2d target, double scale, double endVelocity) {
        return drivetrain
                .mirrorablePathFindCommand(target, scale, endVelocity)
                .deadlineWith(smartInfeedCommand().withTimeout(0.6).andThen(coolNoteFixCommand(0.2))
                        .andThen(shooter.runShotCommand(ShotSpeeds.FAST)))
                .andThen(stationaryShot(targetDistance));
    }

    private Command stationaryShot(double targetDistance) {
        return new ShooterAlign(drivetrain, shooterLimelight).withTimeout(0.5)
                .andThen(shotSequence(() -> ShooterTable.calcShooterTableEntry(Feet.of(targetDistance))));
    }

    private Command stationaryShotNoPV(double targetDistance) {
        return shotSequence(() -> ShooterTable.calcShooterTableEntry(Feet.of(targetDistance)));
    }

    /* Fix Note Sequence */
    private Command fixNoteCommand() {
        return runBoth(true, FAST_CONVEYOR_VBUS, INFEED_VBUS).withTimeout(0.25).andThen(
                conveyBackCommand(-2.0, 0.1));
    }

    /* Entry Shot Sequence */
    private Command shotSequence(Supplier<ShooterTableEntry> entry, double timeout) {
        return driverCamera.setShooterCameraCommand()
                .alongWith(runEntryCommand(entry, () -> ShotSpeeds.FAST))
                .andThen(Commands.waitUntil(shooterAndPivotReady()).withTimeout(timeout))
                .andThen(conveyCommand())
                .andThen(Commands.waitSeconds(0.1))
                .andThen(shooter.stopCommand().alongWith(pivot.runToHomeCommand()))
                // asynchronously set camera
                .finallyDo(() -> setCameraWithWait());
    }

    private Command shotSequence(Supplier<ShooterTableEntry> entry) {
        return shotSequence(entry, 0.5);
    }

    /* Magic shoot but awesome */
    private Command magicShootCommand() {
        return driverCamera.setShooterCameraCommand()
                .andThen(shooter.runShotCommand(ShotSpeeds.FAST))
                .alongWith(new ShooterAlign(drivetrain, shooterLimelight).withTimeout(0.5))
                .andThen(Commands.waitSeconds(0.1))
                .andThen(runEntryCommand(() -> getBestSTEntryVision(), () -> ShotSpeeds.FAST))
                .andThen(Commands.waitUntil(shooterAndPivotReady()))
                .andThen(Commands.waitSeconds(0.1))
                .andThen(conveyCommand())
                .finallyDo(() -> {
                    shooter.stop();
                    pivot.runToPosition(Pivot.HOLD_POSITION);
                    setCameraWithWait();
                });
    }

    private Command magicLockCommand() {
        return driverCamera.setShooterCameraCommand()
                .andThen(new ShooterAlignStrafe(drivetrain, () -> getDriveSignum()
                        * scaleDriverController(-driverController.getLeftY(), xLimiter, currentSpeed) * MAX_SPEED,
                        () -> getDriveSignum()
                                * scaleDriverController(-driverController.getLeftX(), yLimiter, currentSpeed)
                                * MAX_SPEED,
                        stationaryVision))
                .alongWith(
                        runEntryCommand(this::getBestSTEntryPhotonStationaryDist, () -> ShotSpeeds.FAST).repeatedly());
    }

    private Command magicShootNoLockCommand() {
        return driverCamera.setShooterCameraCommand()
                // .andThen(shooter.runShotCommand(ShotSpeeds.FAST))
                // .alongWith(new LimeShooterAlign(drivetrain).withTimeout(0.5))
                // .andThen(Commands.waitSeconds(0.1))
                .andThen(runEntryCommand(() -> getBestSTEntryVision(), () -> ShotSpeeds.FAST))
                .andThen(Commands.waitUntil(shooterAndPivotReady()))
                .andThen(conveyCommand())
                .finallyDo(() -> {
                    shooter.stop();
                    pivot.runToPosition(Pivot.HOLD_POSITION);
                    setCameraWithWait();
                });
    }

    private Command appendAutonPhase(String name) {
        return new InstantCommand(() -> {
            autonPhase.append(name);
            System.out.println(name);
        });
    }

    /* Magic shoot but awesome3 */
    private Command fastMagicShootCommand() {
        return driverCamera.setShooterCameraCommand().alongWith(appendAutonPhase("Set Camera"))
                .andThen(
                        Commands.runOnce(() -> getBestSTEntryVision())
                                .alongWith(appendAutonPhase("Got Vision Distance")))
                .andThen(runEntryCommand(() -> ShooterTable.calcShooterTableEntry(Feet.of(previousLimelightDistance)),
                        () -> ShotSpeeds.FAST).alongWith(appendAutonPhase("Commanded Shooter and Pivot to position"))
                        .alongWith(new ShooterAlignEpic(drivetrain, shooterLimelight).withTimeout(0.5)
                                .andThen(appendAutonPhase("Aligned to speaker por favor"))))
                .andThen(Commands.waitUntil(shooterAndPivotReady())
                        .andThen(appendAutonPhase("Shooter and pivot are ready")))
                .andThen(conveyCommand().andThen(appendAutonPhase("Conveyed the note into the shooter")))
                .finallyDo(() -> {
                    shooter.stop();
                    pivot.runToPosition(Pivot.HOLD_POSITION);
                    autonPhase.append("Pathplanner driving");
                    setCameraWithWait();
                });
    }

    /* Magic shoot but awesomer */
    private Command megaTag2ShootCommand() {
        return driverCamera.setShooterCameraCommand()
                .andThen(shooter.runShotCommand(ShotSpeeds.FAST))
                .alongWith(new RotateToSpeaker(drivetrain).withTimeout(0.5))
                .andThen(runEntryCommand(() -> getBestSTEntry(), () -> ShotSpeeds.FAST))
                .andThen(Commands.waitUntil(shooterAndPivotReady()))
                .andThen(conveyCommand())
                .finallyDo(() -> {
                    shooter.stop();
                    pivot.runToPosition(Pivot.HOLD_POSITION);
                    setCameraWithWait();
                });
    }

    /* Fix Note Backwards */
    private Command conveyBackCommand(double rotations, double timeout) {
        return shooter.spinMotorLeftCommand(SHOOTER_BACKOUT_VBUS).repeatedly()
                .raceWith(conveyor.runXRotations(rotations).withTimeout(timeout))
                .alongWith(infeed.runMotorCommand(0.))
                .andThen(shooter.stopCommand());
    }

    /* Special Note Fix */
    private Command coolNoteFixCommand(double timeout) {
        return shooter.spinMotorLeftCommand(SHOOTER_BACKOUT_VBUS).repeatedly()
                .alongWith(infeed.runMotorCommand(0.))
                .alongWith(conveyor.runMotorCommand(-0.2)).withTimeout(timeout)
                .andThen(shooter.stopCommand())
                .andThen(conveyor.brakeStopCommand());
    }

    /* Convey the Note */
    private Command conveyCommand() {
        return conveyor.runXRotations(20.).alongWith(infeed.runMotorCommand(SLOW_INFEED_VBUS));
    }

    /* Stop all motors and zero everything */
    private Command stopAllCommand(boolean switchCamera) {
        return Commands.parallel(
                infeed.stopCommand(),
                conveyor.stopCommand(),
                shooter.stopCommand().andThen(shooter.setSlotCommand(Shooter.Slots.FAST)),
                pivot.runToHomeCommand(),
                m_fan.stopCommand(),
                m_fanPivot.runToPositionCommand(0.),
                whippy.stopCommand(),
                setShooterPipelineCommand(SHOOTING_PIPELINE),
                driverCamera.setInfeedCameraCommand().onlyIf(() -> switchCamera),
                Commands.runOnce(() -> currentSequence = ClimbSequence.Default));
    }

    /* That but don't reset the camera */
    private Command stopAllCommand() {
        return stopAllCommand(false);
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
                        getDriveSignum() * scaleDriverController(-driverController.getLeftY(),
                                xLimiter,
                                currentSpeed)
                                * MAX_SPEED)
                .withVelocityY(
                        getDriveSignum() * scaleDriverController(-driverController.getLeftX(),
                                yLimiter,
                                currentSpeed)
                                * MAX_SPEED)
                .withTargetDirection(Rotation2d.fromDegrees(direction.Angle)));
    }

    /* Smart Infeed Command Generator */
    private Command smartInfeedCommand() {
        return runBoth(true, SLOW_CONVEYOR_VBUS, INFEED_VBUS)
                .until(noteSensing.hasInfedSupplier())
                .andThen(runBoth(true, 0., 0.).withTimeout(0.1))
                .andThen(conveyBackCommand(-4.0, 0.5))
                .finallyDo(shooter::stop);
    }

    /* Run a Shooter Table Entry */
    private Command runEntryCommand(Supplier<ShooterTableEntry> entry, Supplier<ShotSpeeds> speed) {
        return shooter.runEntryCommand(entry, speed)
                .alongWith(pivot.runToPositionCommand(() -> entry.get().Angle))
                .alongWith(Commands.runOnce(() -> m_lastShot = entry.get().Distance.in(Feet))
                        .onlyIf(() -> entry.get().Distance != null));
    }

    /* Shooter & Pivot Both Ready */
    private BooleanSupplier shooterAndPivotReady() {
        return () -> shooter.isReady() && pivot.inPosition();
    }

    /* Run both Conveyor and Infeed */
    private Command runBoth(boolean stopShooter, double conveyorVbus, double infeedVbus) {
        return shooter.brakeStopCommand().onlyIf(() -> stopShooter)
                .alongWith(infeed.runMotorCommand(infeedVbus)
                        .alongWith(conveyor.runMotorCommand(conveyorVbus)).repeatedly());
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
                .andThen(NamedCommands.getCommand("AprilTag Zero"))
                .andThen(autonChooser.getSelected());
    }

    /* Zeroing Command */
    public Command zeroCommand() {
        return pivot.zeroCommand().alongWith(m_fanPivot.runToPositionCommand(0.));// .alongWith(climber.zeroCommand());
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
        m_fan.logValues();
    }

    // ================ //
    /* Vision Utilities */
    // ================ //

    /* Set Shooter Pipeline */
    public void setShooterPipeline(int index) {
        LimelightHelpers.setPipelineIndex(SHOOTER_LIMELIGHT, index);
    }

    public Command setShooterPipelineCommand(int index) {
        return Commands.runOnce(() -> setShooterPipeline(index));
    }

    /* Configure Field Origins */
    public void configVisionFieldOrigins() {
        // shooterVision.configFieldOrigin();
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
        Translation2d translation = new Translation2d(pose.getX() - Constants.SPEAKER_DISTANCE_TARGET.getX(),
                pose.getY() - Constants.SPEAKER_DISTANCE_TARGET.getY());

        ShooterTableEntry entryPicked = ShooterTable
                .calcShooterTableEntry(Meters.of(translation.getNorm()));

        SmartDashboard.putNumber("Distance", Units.metersToFeet(translation.getNorm()));

        SmartDashboard.putNumber("ST Angle", entryPicked.Angle);
        SmartDashboard.putNumber("ST Left", entryPicked.Percent);

        return entryPicked;
    }

    // public ShooterTableEntry[] getBestSTEntryAllStrats() {
    // ShooterTableEntry[] steArr = new ShooterTableEntry[5];

    // steArr[0] = getBestSTEntryPhotonYDistance();
    // steArr[1] = getBestSTEntryPhotonY();
    // steArr[2] = getBestSTEntryPhotonY();
    // steArr[3] = getBestSTEntryLLArea();
    // steArr[4] = getBestSTEntryLLAreaMulti();
    // return steArr;
    // }

    // private ShooterTableEntry getBestSTEntryPhotonYDistance() {
    // double deltaH = Units.metersToFeet(SPEAKER_TAG_HEIGHT - SHOOTER_CAM_HEIGHT);
    // double angle =
    // Units.degreesToRadians(LimelightHelpers.getTY(SHOOTER_LIMELIGHT));
    // var ste = ShooterTable.calcShooterTableEntryCamera(deltaH / Math.tan(angle +
    // SHOOTER_CAM_PITCH),
    // CameraLerpStrat.LimeLightTYDistance);

    // SmartDashboard.putNumber("Limelight TY tangented distance",
    // ste.Distance.in(Feet));
    // return ste;
    // }

    private ShooterTableEntry getBestSTEntryLLY() {
        ShooterTableEntry entry;

        if (LimelightHelpers.getTV(SHOOTER_LIMELIGHT)) {
            entry = ShooterTable.calcShooterTableEntryCamera(LimelightHelpers.getTY(SHOOTER_LIMELIGHT),
                    CameraLerpStrat.LimelightTY);
            previousLimelightDistance = entry.Distance.in(Feet);
            m_limelightTimer.restart();
        } else {
            double dist = m_limelightTimer.get() > MAGIC_SHOOT_TIMER_THRESHOLD ? 20.0 : previousLimelightDistance;
            entry = ShooterTable.calcShooterTableEntry(Feet.of(dist));
            previousLimelightDistance = dist;
        }

        return entry;
    }

    public void updateMTRot() {
        megaLeftVision.setRobotRotationMT2(drivetrain.getState().Pose.getRotation().getDegrees());
        megaRightVision.setRobotRotationMT2(drivetrain.getState().Pose.getRotation().getDegrees());
    }

    public void updateDrivePoseMT2() {
        updateMTRot();
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
        updateMTRot();
        var llLeftPoseEst = megaLeftVision.getBotposeEstimateMT2();
        var llRightPoseEst = megaRightVision.getBotposeEstimateMT2();
        Pose2d llAvgPose;

        if (llLeftPoseEst.tagCount <= 0 && llRightPoseEst.tagCount <= 0)
            llAvgPose = new Pose2d();
        else if (llLeftPoseEst.tagCount <= 0)
            llAvgPose = new Pose2d(llRightPoseEst.pose.getTranslation(), drivetrain.getState().Pose.getRotation());
        else if (llRightPoseEst.tagCount <= 0)
            llAvgPose = new Pose2d(llLeftPoseEst.pose.getTranslation(), drivetrain.getState().Pose.getRotation());
        else
            llAvgPose = new Pose2d(
                    llLeftPoseEst.pose.getTranslation().plus(llRightPoseEst.pose.getTranslation()).div(2.),
                    drivetrain.getState().Pose.getRotation());

        double llAvgTimestamp = (llLeftPoseEst.timestampSeconds + llRightPoseEst.timestampSeconds) / 2;
        return drivetrain.addMeasurementCommand(() -> llAvgPose, () -> llAvgTimestamp);
    }

    // private ShooterTableEntry getBestSTEntryPhotonY() {
    // Optional<Double> distance = shooterVision
    // .getTagDistance(getAllianceSpeakerTag());

    // var ste =
    // ShooterTable.calcShooterTableEntryCamera(Units.metersToFeet(distance.isEmpty()
    // ? 0
    // : distance.get()),
    // CameraLerpStrat.PhotonVisionDistance);

    // SmartDashboard.putNumber("PhotonVision 2d distance", ste.Distance.in(Feet));
    // return ste;
    // }

    private ShooterTableEntry getBestSTEntryVision() {
        return getBestSTEntryLLY();
    }

    private ShooterTableEntry getBestSTEntryLLYEPIC() {
        return EPICShooterTable.getShooterTableEntryCamera(LimelightHelpers.getTY(SHOOTER_LIMELIGHT),
                CameraLerpStrat.LimelightTY);
    }

    private ShooterTableEntry getBestSTEntryPhotonStationaryDist() {
        Optional<Double> camDist = stationaryVision.getTagDistance(allianceIsBlue() ? 7 : 4);
        var ste = ShooterTable.calcShooterTableEntryCamera(camDist.isEmpty() ? 0 : camDist.get(),
                CameraLerpStrat.PhotonVisionStationaryDistance);

        SmartDashboard.putNumber("PhotonVision Stationary 2d distance", ste.Distance.in(Feet));
        return ste;
    }

    // private ShooterTableEntry getBestSTEntryLLArea() {
    // LimelightHelpers.setPipelineIndex(SHOOTER_LIMELIGHT, 0);
    // var ste =
    // ShooterTable.calcShooterTableEntryCamera(LimelightHelpers.getTA(SHOOTER_LIMELIGHT),
    // CameraLerpStrat.LimelightArea);

    // SmartDashboard.putNumber("Limelight TA distance", ste.Distance.in(Feet));
    // return ste;
    // }

    // private ShooterTableEntry getBestSTEntryLLAreaMulti() {
    // LimelightHelpers.setPipelineIndex(SHOOTER_LIMELIGHT, 1);
    // var ste =
    // ShooterTable.calcShooterTableEntryCamera(LimelightHelpers.getTA(SHOOTER_LIMELIGHT),
    // CameraLerpStrat.LimelightMultiTagArea);

    // SmartDashboard.putNumber("Limelight MultiTag TA distance",
    // ste.Distance.in(Feet));
    // LimelightHelpers.setPipelineIndex(SHOOTER_LIMELIGHT, 0);
    // return ste;
    // }

}
