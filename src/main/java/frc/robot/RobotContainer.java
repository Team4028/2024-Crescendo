// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Meters;

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
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.RotateToSpeaker;
import frc.robot.commands.Autons;
import frc.robot.commands.Autons.Notes;
import frc.robot.commands.Autons.StartPoses;
import frc.robot.commands.vision.LimelightAcquire;
import frc.robot.commands.vision.LimelightSquare;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Climber.ClimberPositions;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Fan;
import frc.robot.subsystems.Infeed;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShotSpeeds;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Whippy;
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

    // ====================== //
    /* Auton & Other Commands */
    // ====================== //
    private final Command magicShootCommand, magicTrapCommand, ampPrep, ampShoot;
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
        Right(270);

        public double Angle;

        private SnapDirection(double angle) {
            Angle = angle;
        }
    }

    private enum ShooterTableIndex {
        Close(4.2, "Close Shot"),
        Protected(10.0, "Protected"),
        Chain(13.0, "Chain"),
        Truss(16.0, "Truss"),
        Wing(19.0, "Wing"),
        Neutral(22.0, "Neutral Zone");

        public double Index;
        public String Name;

        private ShooterTableIndex(double index, String name) {
            Index = index;
            Name = name;
        }
    }

    private List<ShooterTableIndex> indexList = List.of(ShooterTableIndex.values());

    private int currentIndex = 0;

    // ======================== //
    /* Swerve Control & Logging */
    // ======================== //
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MAX_SPEED * 0.02).withRotationalDeadband(MAX_ANGULAR_SPEED * 0.01) // Add a 50%
                                                                                             // deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                                     // driving in open loop
    private final Telemetry logger = new Telemetry(MAX_SPEED);

    private final SwerveRequest.SwerveDriveBrake xDrive = new SwerveDriveBrake();
    private final SwerveRequest.FieldCentricFacingAngle snapDrive = new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(MAX_SPEED * 0.035)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    public RobotContainer() {
        snapDrive.HeadingController = new PhoenixPIDController(2., 0., 0.);
        snapDrive.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
        // TODO: Failsafe timer based on Infeed ToF
        initNamedCommands();

        autons = new Autons(drivetrain, shooter, conveyor, infeed,
                smartInfeedCommand());

        initAutonChooser();

        magicShootCommand = new RotateToSpeaker(drivetrain).andThen(Commands.runOnce(() -> {
            ShooterTableEntry entry = getBestSTEntry();
            shooter.runEntry(entry, ShotSpeeds.FAST);
            pivot.runToPosition(entry.Angle);
        }, shooter, pivot)).andThen(Commands.waitUntil(shooter.isReadySupplier()))
                .andThen(Commands.waitSeconds(0.5))
                .andThen(conveyor.runXRotations(10)
                        .alongWith(infeed.runMotorCommand(SLOW_INFEED_VBUS)))
                .andThen(Commands.waitSeconds(0.2))
                .finallyDo(this::stopAll);

        magicTrapCommand = drivetrain.pathFindCommand(Constants.LEFT_TRAP_TARGET, .2,
                0)
                .andThen(shooter.setSlotCommand(Shooter.Slots.TRAP))
                .andThen(new WaitCommand(2))
                .andThen(pivot.runToTrapCommand())
                .andThen(m_fan.runMotorCommand(FAN_VBUS))
                .andThen(shooter.runShotCommand(ShotSpeeds.TRAP).repeatedly()
                        .until(shooterAndPivotReady()).withTimeout(4))
                .andThen(conveyor.runXRotations(20))
                .andThen(shooter.stopCommand())
                .andThen(pivot.runToHomeCommand());

        ampPrep = pivot.runToClimbCommand()
                .alongWith(whippy.whippyWheelsCommand(WHIPPY_VBUS))
                .alongWith(shooter.runShotCommand(ShotSpeeds.AMP));

        ampShoot = conveyor.runXRotations(20.).alongWith(infeed.runMotorCommand(INFEED_VBUS))
                .andThen(new WaitCommand(1.0))
                .finallyDo(this::stopAll);

        configureBindings();
    }

    // ====================== //
    /* Auton & Named Commands */
    // ====================== //
    private void initAutonChooser() {
        autonChooser = AutoBuilder.buildAutoChooser();
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
                conveyor.runXRotations(20.).alongWith(infeed.runMotorCommand(SLOW_INFEED_VBUS))
                        .andThen(shooter.stopCommand()));

        NamedCommands.registerCommand("Spit Note", infeed.runMotorCommand(INFEED_VBUS)
                .alongWith(conveyor.runMotorCommand(FAST_CONVEYOR_VBUS))
                .alongWith(shooter.spinBothCommand(0.15))
                .repeatedly());

        NamedCommands.registerCommand("Prepare Spit", shooter.spinBothCommand(0.15));

        NamedCommands.registerCommand("Limelight Acquire",
                new LimelightAcquire(() -> xLimeAquireLimiter.calculate(0.5),
                        drivetrain)
                        .alongWith(smartInfeedCommand()));

        NamedCommands.registerCommand("Smart Infeed", smartInfeedCommand());

        ShooterTableEntry twoHalfEntry = new ShooterTableEntry(Feet.of(0),
                10.0, 1.0); // TODO: fix

        // TODO: We may want a command that constantly updates the shooter table and
        // runs the shooter/pivot based on that
        // makes shooting on the move/faster autons much easier

        NamedCommands.registerCommand("Start Shooter",
                runEntryCommand(() -> twoHalfEntry, () -> ShotSpeeds.FAST));

        ShooterTableEntry fourEntry = new ShooterTableEntry(Feet.of(0), 21.5, 1.0);
        NamedCommands.registerCommand("4 Piece Shooter", runEntryCommand(() -> fourEntry, () -> ShotSpeeds.FAST));

        NamedCommands.registerCommand("Stop Shooter", shooter.stopCommand());
        NamedCommands.registerCommand("Stop Infeed", runBoth(0., 0.));

        NamedCommands.registerCommand("Center Pathfinding Shot", drivetrain
                .pathFindCommand(new Pose2d(4.99, 6.66, new Rotation2d(Units.degreesToRadians(13.3))), 0.75, 0)
                .alongWith(runEntryCommand(() -> twoHalfEntry, () -> ShotSpeeds.FAST).repeatedly()
                        .until(shooterAndPivotReady()))
                .andThen(conveyor.runXRotations(20.).alongWith(infeed.runMotorCommand(SLOW_INFEED_VBUS))
                        .withTimeout(1.0))
                .andThen(shooter.stopCommand()));

        NamedCommands.registerCommand("Right Center Pathfinding Shot", drivetrain
                .pathFindCommand(new Pose2d(4.3, 1.9, Rotation2d.fromDegrees(-40)), 0.75, 0)
                .alongWith(runEntryCommand(() -> twoHalfEntry, () -> ShotSpeeds.FAST).repeatedly()
                        .until(shooterAndPivotReady()))
                .andThen(conveyor.runXRotations(20.).alongWith(infeed.runMotorCommand(SLOW_INFEED_VBUS))
                        .withTimeout(1.0))
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

        /*
         * Spin up Shooter
         * When shooter ready, feed
         * Stop shooter
         */
        NamedCommands.registerCommand("2.5 Stationary Shot",
                runEntryCommand(() -> twoHalfEntry, () -> ShotSpeeds.FAST)
                        .repeatedly().until(shooterAndPivotReady())
                        .andThen(conveyor.runXRotations(20.).alongWith(infeed.runMotorCommand(SLOW_INFEED_VBUS)))
                        .andThen(shooter.stopCommand()));

        NamedCommands.registerCommand("2.5 Final Note", drivetrain
                .pathFindCommand(new Pose2d(3.71, 6.51, new Rotation2d(Units.degreesToRadians(-9.1))), 0.75, 0));
        NamedCommands.registerCommand("2.5 Final Note Right", drivetrain
                .pathFindCommand(new Pose2d(3.71, 1.8, new Rotation2d(Units.degreesToRadians(9.1))), 0.75, 0.));

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
                        .withVelocityX(scaleDriverController(-driverController.getLeftY(),
                                xLimiter,
                                currentSpeed) * MAX_SPEED)
                        .withVelocityY(scaleDriverController(-driverController.getLeftX(),
                                yLimiter,
                                currentSpeed) * MAX_SPEED)
                        .withRotationalRate(
                                scaleDriverController(-driverController.getRightX(),
                                        thetaLimiter, currentSpeed) *
                                        MAX_ANGULAR_SPEED))
                        .alongWith(Commands.runOnce(() -> SmartDashboard.putBoolean("Snapped", false))));

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

        // =================== //
        /* OPERATOR CONTROLLER */
        // =================== //

        // ========================= //
        /* Driver Help Control */
        // ========================= //

        /* Snap to Amp */
        operatorController.povLeft().toggleOnTrue(Commands.either(
                snapCommand(SnapDirection.Left), snapCommand(SnapDirection.Right),
                () -> {
                    var alliance = DriverStation.getAlliance();
                    return alliance.isPresent() && alliance.get() == Alliance.Blue;
                }));

        /* Limelight Square */
        operatorController.leftStick().toggleOnTrue(new LimelightSquare(true,
                () -> scaleDriverController(-driverController.getLeftY(), xLimeAquireLimiter, currentSpeed) * MAX_SPEED,
                () -> scaleDriverController(-driverController.getLeftX(), yLimeAquireLimiter, currentSpeed) * MAX_SPEED,
                drivetrain));

        // ======================= //
        /* Shooter Control */
        // ======================= //

        /* Spin Up Shooter */
        operatorController.leftBumper()
                .onTrue(Commands.runOnce(() -> {
                    ShooterTableEntry entry = ShooterTable
                            .calcShooterTableEntry(Feet.of(indexList.get(currentIndex).Index));
                    shooter.runEntry(entry, ShotSpeeds.FAST);
                    pivot.runToPosition(entry.Angle);
                }, shooter, pivot))
                .onFalse(stopAllCommand());

        /* Convey Note */
        operatorController.rightBumper()
                .whileTrue(runBoth(FAST_CONVEYOR_VBUS, SLOW_INFEED_VBUS).repeatedly());

        /* Magic Shoot */
        operatorController.x().toggleOnTrue(magicShootCommand);

        /* Shooter Table Index Down */
        operatorController.leftTrigger(0.5).onTrue(new InstantCommand(() -> {
            currentIndex = MathUtil.clamp(currentIndex - 1, 0, ShooterTableIndex.values().length - 1);
            pushIndexData();
        }));

        /* Shooter Table Index Up */
        operatorController.rightTrigger(0.5).onTrue(new InstantCommand(() -> {
            currentIndex = MathUtil.clamp(currentIndex + 1, 0, ShooterTableIndex.values().length - 1);
            pushIndexData();
        }));

        // ========================= //
        /* Climber & Zeroing Control */
        // ========================= //

        /* Zero Climber & Pivot */
        operatorController.start().onTrue(zeroCommand());

        /* Run Pivot & Climber to Zero */
        operatorController.a().onTrue(
                climber.runToPositionCommand(ClimberPositions.HOME).andThen(
                        Commands.waitUntil(climber.inPositionSupplier()),
                        pivot.runToHomeCommand()));

        /* CLIMB */
        operatorController.povDown().onTrue(climber.climbCommand());

        /* Tension Chain */
        operatorController.povRight().onTrue(climber.runToPositionCommand(Climber.ClimberPositions.TENSION));

        /* Prime Climber & Pivot */
        operatorController.povUp().onTrue(pivot.runToClimbCommand().andThen(
                Commands.waitUntil(pivot.inPositionSupplier()),
                climber.runToPositionCommand(Climber.ClimberPositions.READY)));

        // TODO: no explicit trap position command
        // trap, amp, climb sequences should all do this on their own
        // w/ explicit position on emergency

        // ================ //
        /* Amp & Trap Magic */
        // ================ //

        operatorController.b().onTrue(ampPrep).onFalse(ampShoot);

        // operatorController.y().toggleOnTrue(magicTrapCommand);
        operatorController.y().onTrue(pivot.runToTrapCommand());

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
        emergencyController.y().onTrue(m_fan.runMotorCommand(FAN_VBUS)).onFalse(m_fan.stopCommand());

        emergencyController.back().onTrue(m_fan.runToPositionCommand(0.));
        emergencyController.start().whileTrue(m_fan.runPivotCommand(FAN_PIVOT_VBUS));

        /* ST test */
        // emergencyController.back().onTrue(Commands.runOnce(() -> getBestSTEntry()));

        /* Full Outfeed: left Y */
        emergencyController.axisGreaterThan(XboxController.Axis.kLeftY.value, 0.2)
                .or(emergencyController.axisLessThan(XboxController.Axis.kLeftY.value, -0.2))
                .whileTrue(
                        runThree(
                                () -> -emergencyController.getLeftY(),
                                () -> -emergencyController.getLeftY(),
                                () -> emergencyController.getLeftY() > 0. ? -emergencyController.getLeftY() : 0.));

        /* Cool Outfeed: right Y */
        emergencyController.axisLessThan(XboxController.Axis.kRightY.value, -0.2)
                .whileTrue(
                        runThree(
                                () -> emergencyController.getRightY(),
                                () -> -emergencyController.getRightY(),
                                () -> emergencyController.getRightY()));

        emergencyController.b().onTrue(shooter.runShotCommand(ShotSpeeds.TRAP)).onFalse(shooter.stopCommand());
        emergencyController.x().onTrue(m_fan.runToTrapCommand());

        if (Utils.isSimulation()) {
            drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
        }

        drivetrain.registerTelemetry(logger::telemeterize);

    }

    // =========================================== //
    /* Additional Commands, Getters, and Utilities */
    // =========================================== //

    /* Stop all motors and zero everything */
    private void stopAll() {
        infeed.stop();
        conveyor.stop();
        shooter.stop();
        climber.runToPositionCommand(ClimberPositions.HOME)
                .andThen(Commands.waitUntil(climber.inPositionSupplier()),
                        pivot.runToHomeCommand())
                .schedule();
        m_fan.stop();
        m_fan.runToPosition(0.);
        whippy.stop();
    }

    private Command stopAllCommand() {
        return Commands.runOnce(this::stopAll, infeed, conveyor, shooter, climber, m_fan, pivot, whippy);
    }

    /* Put Current ST Index Data to Dashboard */
    private void pushIndexData() {
        ShooterTableIndex idx = indexList.get(currentIndex);
        SmartDashboard.putNumber("Shooter Table Index", idx.Index);
        SmartDashboard.putString("Shooter Table Name", idx.Name);
    }

    /* Set Snap Direction Toggle */
    private Command snapCommand(SnapDirection direction) {
        return drivetrain.applyRequest(() -> snapDrive
                .withVelocityX(scaleDriverController(-driverController.getLeftY(),
                        xLimiter,
                        currentSpeed) * MAX_SPEED)
                .withVelocityY(scaleDriverController(-driverController.getLeftX(),
                        yLimiter,
                        currentSpeed) * MAX_SPEED)
                .withTargetDirection(Rotation2d.fromDegrees(direction.Angle)))
                .alongWith(Commands.runOnce(() -> SmartDashboard.putBoolean("Snapped", true)));
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

    /* Test Shooter Table */
    private ShooterTableEntry getBestSTEntry() {
        Pose2d pose = drivetrain.getState().Pose;

        Transform2d dist = pose.minus(Constants.SPEAKER_DISTANCE_TARGET);
        Translation2d translation = dist.getTranslation(); // get everything in feet

        ShooterTableEntry entryPicked = ShooterTable
                .calcShooterTableEntry(Meters.of(translation.getNorm()));

        SmartDashboard.putNumber("Distance", Units.metersToFeet(translation.getNorm()));

        SmartDashboard.putNumber("ST Angle", entryPicked.Angle);
        SmartDashboard.putNumber("ST Left", entryPicked.Percent);

        return entryPicked;
    }
}
