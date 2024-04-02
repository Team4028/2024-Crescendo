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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
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
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.vision.LimelightAcquire;
import frc.robot.commands.vision.LimelightSquare;
import frc.robot.commands.vision.ShooterAlign;
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
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Whippy;
import frc.robot.subsystems.Climber.ClimberPositions;
import frc.robot.utils.BeakCommands;
import frc.robot.utils.DashboardStore;
import frc.robot.utils.DriverCamera;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.NoteSensing;
import frc.robot.utils.ShooterTable;
import frc.robot.utils.ShooterTable.ShooterTableEntry;
import frc.robot.utils.ShooterTable.ShooterTableEntry.CameraLerpStrat;

public class RobotContainer {
    // =============================================== //
    /* Magic numbers, Vbus constants, and OI constants */
    // =============================================== //
    private static final double CLIMBER_VBUS = 0.75;
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

    private static final int SHOOTING_PIPELINE = 0;
    // private static final int TRAP_PIPELINE = 2;

    private static final String SHOOTER_LIMELIGHT = "limelight-shooter";

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

    private final Vision rightVision = new Vision("Right_AprilTag_Camera", Vision.RIGHT_ROBOT_TO_CAMERA);
    private final Vision leftVision = new Vision("Left_AprilTag_Camera", Vision.LEFT_ROBOT_TO_CAMERA);
    private final Vision trapVision = new Vision("Shooter-OV2311", Vision.SHOOTER_ROBOT_TO_CAMERA);

    public Conveyor getConveyor() {
        return conveyor;
    }

    // ====================== //
    /* Auton & Other Commands */
    // ====================== //
    private final Command ampPrep;
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

    private boolean isSnappedToSpeaker = false;
    // private boolean isInMagicShoot = false;

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

    private final LinkedHashMap<Double, String> indexMap = new LinkedHashMap<>();

    private double currentIndex = MIN_INDEX;
    private double manualIndex = MIN_INDEX;
    private int presetIndex = 0;
    private boolean useManual = false;

    private boolean enableClimber = false;
    private boolean enableTrap = false;

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
        trapVision.setPipeline(Vision.SHOOTER_PIPELINE_INDEX);

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

        // this is weird
        DashboardStore.add("Snapped", () -> drivetrain.getCurrentRequest().getClass().equals(snapDrive.getClass()));
        DashboardStore.add("Robot Relative",
                () -> drivetrain.getCurrentRequest().getClass().equals(robotRelativeDrive.getClass()));

        DashboardStore.add("Manual Indexing", () -> useManual);
        DashboardStore.add("Climber Enabled", () -> enableClimber);
        DashboardStore.add("Trap Enabled", () -> enableTrap);

        DashboardStore.add("Sequence", () -> currentSequence.name());

        DashboardStore.add("Aligned to Speaker", () -> isSnappedToSpeaker);
        // DashboardStore.add("Executing magic shoot", () -> isInMagicShoot);

        DashboardStore.add("Photon Distance", () -> getBestSTEntryPhotonY().Distance.in(Feet));

        DashboardStore.add("Limelight TY", () -> getBestSTEntryLLY().Distance.in(Feet));

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

        NamedCommands.registerCommand("AprilTag Zero", new InstantCommand(() -> {
            Optional<EstimatedRobotPose> pose = getBestPose();
            if (pose.isEmpty())
                return;

            drivetrain.seedFieldRelative(pose.get().estimatedPose.toPose2d());
        }, leftVision, rightVision));

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
                smartInfeedCommand().andThen(shooter.runShotCommand(ShotSpeeds.FAST))
                        .andThen(pivot.runToPositionCommand(5.)));

        /* Shooter & Pivot */
        NamedCommands.registerCommand("Fast Shooter",
                shooter.runShotCommand(ShotSpeeds.FAST));

        NamedCommands.registerCommand("Shooter",
                shooter.runShotCommand(ShotSpeeds.FAST, 0.85));

        NamedCommands.registerCommand("Stop Shooter", shooter.stopCommand());

        NamedCommands.registerCommand("Home Pivot", pivot.runToHomeCommand());

        /* 4 piece pivots */
        NamedCommands.registerCommand("Preload Note", pivot.runToPositionCommand(17.0));
        NamedCommands.registerCommand("Note A", pivot.runToPositionCommand(11.0));
        NamedCommands.registerCommand("Note B", pivot.runToPositionCommand(15.0));

        NamedCommands.registerCommand("Note 1", pivot.runToPositionCommand(17.5));

        /* Pathfinding Shots */
        NamedCommands.registerCommand("Amp Shot", pathfindingShotCommand(15.5, Constants.LEFT_SHOT, 0.875, 0));

        NamedCommands.registerCommand("Stationary Amp Shot", stationaryShot(16.5));
        NamedCommands.registerCommand("Epic Amp Shot", stationaryShot(7.5));

        NamedCommands.registerCommand("Center Pathfinding Shot", pathfindingShotCommand(
                13.0, Constants.CENTER_SHOT, 0.8, 0.));

        NamedCommands.registerCommand("Source Shot",
                pathfindingShotCommand(20.8, Constants.RIGHT_SHOT, 0.75, 0.));

        NamedCommands.registerCommand("Stationary Source Shot", stationaryShot(20.8));

        NamedCommands.registerCommand("Right Preload", pivot.runOnce(pivot::zeroEncoder)
                .andThen(shotSequence(() -> ShooterTable.calcShooterTableEntry(Feet.of(5.2)))));

        NamedCommands.registerCommand("Center Preload", pivot.runOnce(pivot::zeroEncoder)
                .andThen(shotSequence(() -> ShooterTable.calcShooterTableEntry(Feet.of(4.2)))));

        NamedCommands.registerCommand("Note C Pathfinding",
                mirroredPathfindingShotCommand(12.35, Constants.NOTE_C_SHOT, 0.85, 0.));
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

        driverController.a().and(driverController.povUp()).onTrue(pivot.runDyn(Direction.kForward))
                .onFalse(pivot.runMotorCommand(0.));
        driverController.a().and(driverController.povDown()).onTrue(pivot.runDyn(Direction.kReverse))
                .onFalse(pivot.runMotorCommand(0.));
        driverController.b().and(driverController.povUp()).onTrue(pivot.runQuasi(Direction.kForward))
                .onFalse(pivot.runMotorCommand(0.));
        driverController.b().and(driverController.povDown()).onTrue(pivot.runQuasi(Direction.kReverse))
                .onFalse(pivot.runMotorCommand(0.));

        // ========================= //
        /* Infeed & Conveyor Control */
        // ========================= //

        /* Dumb Infeed */
        driverController.leftTrigger().onTrue(runBoth(true, SLOW_CONVEYOR_VBUS, INFEED_VBUS))
                .onFalse(coolNoteFixCommand(0.15).andThen(driverCamera.setShooterCameraCommand()));

        /* Smart Infeed */
        driverController.leftBumper()
                .toggleOnTrue(smartInfeedCommand().andThen(driverCamera.setShooterCameraCommand()));

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

        // driverController.leftStick().whileTrue(new ShooterAlignWhileStrafing(
        // () -> (DriverStation.getAlliance().isPresent() &&
        // DriverStation.getAlliance().get() == Alliance.Red ? -1
        // : 1) * scaleDriverController(-driverController.getLeftX(), yLimiter,
        // currentSpeed) * MAX_SPEED,
        // drivetrain, trapVision).alongWith(new InstantCommand(() -> isSnappedToSpeaker
        // = true))
        // .andThen(new InstantCommand(() -> isSnappedToSpeaker = false)));

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
                .onTrue(runEntryCommand(() -> ShooterTable.calcShooterTableEntry(Feet.of(currentIndex)),
                        () -> ShotSpeeds.FAST))
                .onFalse(stopAllCommand().andThen(driverCamera.setInfeedCameraCommand()));

        /* Convey Note */
        operatorController.rightTrigger()
                .whileTrue(runBoth(false, FAST_CONVEYOR_VBUS, SLOW_INFEED_VBUS));

        /* Magic Shoot */
        operatorController.x().toggleOnTrue(magicShootCommand().andThen(driverCamera.setInfeedCameraCommand()));

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
                climber.runMotorCommand(CLIMBER_VBUS, true))
                .onFalse(climber.stopCommand());

        /* Climber Down */
        emergencyController.leftTrigger(0.2).whileTrue(
                climber.runMotorCommand(-CLIMBER_VBUS, true))
                .onFalse(climber.stopCommand());

        /* Ready Climb */
        emergencyController.povUp().onTrue(climber.runToPositionCommand(CLIMBER_VBUS, ClimberPositions.READY))
                .onFalse(climber.stopCommand());

        /* Climb */
        emergencyController.povDown().onTrue(climber.runToPositionCommand(CLIMBER_VBUS, ClimberPositions.CLIMB))
                .onFalse(climber.stopCommand());

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

        /* Run Shooter at Trap Speeds */
        emergencyController.y().onTrue(
                NamedCommands.getCommand("Fix Note").andThen(
                        NamedCommands.getCommand("Fix Note"))
                        .andThen(Commands.waitSeconds(0.2))
                        .andThen(shooter.setSlotCommand(Shooter.Slots.TRAP))
                        .andThen(shooter.runShotCommand(ShotSpeeds.TRAP)));

        /* Trap Shoot */
        emergencyController.b().onTrue(conveyCommand())
                .onFalse(shooter.setSlotCommand(Slots.FAST).andThen(shooter.stopCommand())
                        .alongWith(m_fan.stopCommand()));

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

        if (Utils.isSimulation()) {
            drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
        }

        drivetrain.registerTelemetry(logger::telemeterize);

    }

    // =========================================== //
    /* Additional Commands, Getters, and Utilities */
    // =========================================== //

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
                .alongWith(Commands.either(
                        m_fan.runMotorCommand(FAN_VBUS)
                                .andThen(BeakCommands.repeatCommand(fixNoteCommand(), 2))
                                .andThen(shooter.runShotCommand(ShotSpeeds.TRAP)),
                        Commands.none(),
                        () -> enableTrap))
                .alongWith(Commands.either(
                        climber.runToPositionCommand(CLIMBER_VBUS, ClimberPositions.READY),
                        Commands.none(),
                        () -> enableClimber));
    }

    private Command trapShootCommand() {
        return conveyCommand();
    }

    private Command endSequenceCommand() {
        return m_fanPivot.runToPositionCommand(0.0)
                .alongWith(m_fan.stopCommand())
                .alongWith(shooter.stopCommand().andThen(shooter.setSlotCommand(Slots.FAST)))
                .alongWith(
                        Commands.either(
                                Commands.none(),
                                pivot.runToHomeCommand(),
                                () -> enableClimber));
    }

    private Command climbCommand() {
        return climber.runToPositionCommand(CLIMBER_VBUS, ClimberPositions.CLIMB);
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
        return new ShooterAlign(drivetrain, trapVision).withTimeout(0.4)
                .andThen(shotSequence(() -> ShooterTable.calcShooterTableEntry(Feet.of(targetDistance))));
    }

    /* Fix Note Sequence */
    private Command fixNoteCommand() {
        return runBoth(true, FAST_CONVEYOR_VBUS, INFEED_VBUS).withTimeout(0.25).andThen(
                conveyBackCommand(-2.0, 0.1));
    }

    /* Entry Shot Sequence */
    private Command shotSequence(Supplier<ShooterTableEntry> entry) {
        return runEntryCommand(entry, () -> ShotSpeeds.FAST)
                .andThen(Commands.waitUntil(shooterAndPivotReady()).withTimeout(0.5))
                .andThen(conveyCommand())
                .andThen(Commands.waitSeconds(0.1))
                .andThen(shooter.stopCommand().alongWith(pivot.runToHomeCommand()));
    }

    /* Magic shoot but awesome */
    private Command magicShootCommand() {
        return shooter.runShotCommand(ShotSpeeds.FAST)
                .alongWith(new ShooterAlign(drivetrain, trapVision)).withTimeout(0.4)
                .andThen(runEntryCommand(() -> getBestSTEntryVision(), () -> ShotSpeeds.FAST))
                .andThen(Commands.waitUntil(shooterAndPivotReady()))
                .andThen(Commands.waitSeconds(0.1))
                .andThen(conveyCommand())
                .andThen(Commands.waitSeconds(0.2))
                .finallyDo(() -> {
                    shooter.stop();
                    pivot.runToPosition(Pivot.HOLD_POSITION);
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
    private Command stopAllCommand() {
        return Commands.parallel(
                infeed.stopCommand(),
                conveyor.stopCommand(),
                shooter.stopCommand().andThen(shooter.setSlotCommand(Shooter.Slots.FAST)),
                pivot.runToHomeCommand(),
                m_fan.stopCommand(),
                m_fanPivot.runToPositionCommand(0.),
                whippy.stopCommand(),
                setShooterPipelineCommand(SHOOTING_PIPELINE),
                Commands.runOnce(() -> currentSequence = ClimbSequence.Default));
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
        return runBoth(true, SLOW_CONVEYOR_VBUS, INFEED_VBUS)
                .until(noteSensing.hasInfedSupplier())
                .andThen(runBoth(true, 0., 0.).withTimeout(0.1))
                .andThen(conveyBackCommand(-4.0, 0.5))
                .andThen(shooter.spinMotorLeftCommand(0.));
    }

    /* Run a Shooter Table Entry */
    private Command runEntryCommand(Supplier<ShooterTableEntry> entry, Supplier<ShotSpeeds> speed) {
        return shooter.runEntryCommand(entry, speed)
                .alongWith(pivot.runToPositionCommand(() -> entry.get().Angle));
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
        return pivot.zeroCommand().alongWith(m_fanPivot.runToPositionCommand(0.)).alongWith(climber.zeroCommand());
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
        SmartDashboard.putNumber("Raw TY", LimelightHelpers.getTY(SHOOTER_LIMELIGHT));

        var ste = ShooterTable.calcShooterTableEntryCamera(LimelightHelpers.getTY(SHOOTER_LIMELIGHT),
                CameraLerpStrat.LimelightTY);

        SmartDashboard.putNumber("Limelight TY distance", ste.Distance.in(Feet));
        return ste;
    }

    private ShooterTableEntry getBestSTEntryPhotonY() {
        Optional<Double> distance = trapVision
                .getTagDistance(DriverStation.getAlliance().get() == Alliance.Blue ? 7 : 4);

        var ste = ShooterTable.calcShooterTableEntryCamera(Units.metersToFeet(distance.isEmpty()
                ? 0
                : distance.get()),
                CameraLerpStrat.PhotonVisionDistance);

        SmartDashboard.putNumber("PhotonVision 2d distance", ste.Distance.in(Feet));
        return ste;
    }

    private ShooterTableEntry getBestSTEntryVision() {
        return getBestSTEntryLLY();
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

    public void printDistanceValues() {
        int tagID = DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Red ? 4 : 7;

        // var right = rightVision.getBestTranslationToTarget(tagID);
        // var left = leftVision.getBestTranslationToTarget(tagID);

        // Translation2d tr;

        // if (right.isPresent() && left.isPresent()) {
        // tr = right.get().plus(left.get()).div(2.);
        // } else if (right.isPresent()) {
        // tr = right.get();
        // } else if (left.isPresent()) {
        // tr = left.get();
        // } else {
        // tr = new Translation2d();
        // }

        // SmartDashboard.putNumber("Swerve Distance",
        // Units.metersToFeet(tr.getNorm()));

        // var rightYaw = rightVision.getTagYaw(tagID);
        // var leftYaw = leftVision.getTagYaw(tagID);

        // double yaw = 0.;

        // if (rightYaw.isPresent() && leftYaw.isPresent()) {
        // yaw = rightYaw.get() + leftYaw.get() / 2.;
        // } else if (rightYaw.isPresent()) {
        // yaw = rightYaw.get();
        // } else if (leftYaw.isPresent()) {
        // yaw = leftYaw.get();
        // }

        // SmartDashboard.putNumber("Swerve Yaw", yaw);

        // var fiducials =
        // LimelightHelpers.getLatestResults(SHOOTER_LIMELIGHT).targetingResults.targets_Fiducials;

        // for (var fiducial : fiducials) {
        // if (fiducial.fiducialID == tagID) {
        // SmartDashboard.putNumber("Shooter Yaw", fiducial.tx);

        // double tagZMeters = trapVision.layout().getTagPose(tagID).get().getZ();
        // double angle = Units.degreesToRadians(fiducial.ty);

        // double tangent = Math.tan(SHOOTER_CAM_PITCH + angle);
        // double deltaHeight = Units.metersToFeet(tagZMeters/* SPEAKER_TAG_HEIGHT */ -
        // SHOOTER_CAM_HEIGHT);

        // SmartDashboard.putNumber("Shooter ty", angle);
        // SmartDashboard.putNumber("Shooter tangent", tangent);
        // SmartDashboard.putNumber("Shooter dh", deltaHeight);

        // SmartDashboard.putNumber("Shooter Distance",
        // deltaHeight / tangent);
        // }
        // }

        Optional<Double> shooterYaw = trapVision.getTagYaw(tagID);

        if (shooterYaw.isPresent()) {
            SmartDashboard.putNumber("Shooter Yaw", shooterYaw.get());
        } else {
            SmartDashboard.putNumber("Shooter Yaw", 0.);
        }

        Optional<Double> shooterDist = trapVision.getTagDistance(tagID);

        if (shooterDist.isPresent()) {
            SmartDashboard.putNumber("Shooter Distance",
                    Units.metersToFeet(shooterDist.get()));
        } else {
            SmartDashboard.putNumber("Shooter Distance", 0.);
        }

    }
}
