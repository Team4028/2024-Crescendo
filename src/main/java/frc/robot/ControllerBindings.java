package frc.robot;

import static edu.wpi.first.units.Units.Feet;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.CommandFactory.SnapDirection;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.VBusConstants;
import frc.robot.Constants.VisionConstants;
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
import frc.robot.subsystems.Whippy;
import frc.robot.utils.BeakUtils;
import frc.robot.utils.DriverCamera;
import frc.robot.utils.Limelight;
import frc.robot.utils.NoteSensing;
import frc.robot.utils.ShooterTable;
import frc.robot.utils.ShooterTable.ShooterTableEntry;
import frc.robot.utils.ShootingStrategy;
import frc.robot.utils.SubAutos;

public class ControllerBindings {

    private static ShooterTableEntry PASSING_SHOT = new ShooterTableEntry(Feet.zero(), 4, 0.69, Feet.zero());// 30.9;

    private CommandXboxController driverController, operatorController, emergencyController;
    private final CommandSwerveDrivetrain drivetrain;
    private final Infeed infeed;
    private final Shooter shooter;
    private final Conveyor conveyor;
    private final Climber climber;
    private final Pivot pivot;
    private final Fan fan;
    private final FanPivot fanPivot;
    private final Whippy whippy;
    private final Candle candle;
    private final NoteSensing noteSensing;
    private final DriverCamera driverCamera;
    private final Limelight shooterLimelight, chassisLimelight, infeedLimelight3G;
    private final ShootingStrategy shooterLimelightStrategy, odometryStrategy, chassisLimelight2dStrategy;
    private final SlewRateLimiter xLimiter, yLimiter, thetaLimiter;
    private final SubAutos autos;
    private final CommandFactory commandFactory;
    private final DashboardLogging dashboardLogging;
    private final RobotContainer robotContainer;
    private final SwerveRequest.FieldCentric drive;
    private final SwerveRequest.RobotCentric robotRelativeDrive;
    private final SwerveRequest.SwerveDriveBrake xDrive;
    private final Telemetry logger;

    public ControllerBindings(int[] controllerIDs, CommandSwerveDrivetrain drivetrain, Infeed infeed, Shooter shooter,
            Conveyor conveyor, Climber climber, Pivot pivot, Fan fan, FanPivot fanPivot, Whippy whippy, Candle candle,
            NoteSensing noteSensing, DriverCamera driverCamera, Limelight shooterLimelight, Limelight chassisLimelight,
            Limelight infeedLimelight3G, ShootingStrategy shooterLimelightStrategy, ShootingStrategy odometryStrategy,
            ShootingStrategy chassisLimelight2dStrategy, SlewRateLimiter xLimiter, SlewRateLimiter yLimiter,
            SlewRateLimiter thetaLimiter, SubAutos autos, RobotContainer robotContainer) {

        driverController = new CommandXboxController(controllerIDs[0]);
        operatorController = new CommandXboxController(controllerIDs[1]);
        emergencyController = new CommandXboxController(controllerIDs[2]);

        logger = new Telemetry(DrivetrainConstants.MAX_SPEED);

        drive = new SwerveRequest.FieldCentric().withDeadband(DrivetrainConstants.MAX_SPEED * 0.02)
                .withRotationalDeadband(DrivetrainConstants.MAX_ANGULAR_SPEED * 0.01)
                .withDriveRequestType(DriveRequestType.Velocity);

        robotRelativeDrive = new SwerveRequest.RobotCentric().withDeadband(DrivetrainConstants.MAX_SPEED * 0.02)
                .withRotationalDeadband(DrivetrainConstants.MAX_ANGULAR_SPEED * 0.01)
                .withDriveRequestType(DriveRequestType.Velocity);

        xDrive = new SwerveRequest.SwerveDriveBrake();

        this.drivetrain = drivetrain;
        this.infeed = infeed;
        this.shooter = shooter;
        this.conveyor = conveyor;
        this.climber = climber;
        this.pivot = pivot;
        this.fan = fan;
        this.fanPivot = fanPivot;
        this.whippy = whippy;
        this.candle = candle;
        this.noteSensing = noteSensing;
        this.driverCamera = driverCamera;
        this.shooterLimelight = shooterLimelight;
        this.chassisLimelight = chassisLimelight;
        this.infeedLimelight3G = infeedLimelight3G;
        this.shooterLimelightStrategy = shooterLimelightStrategy;
        this.odometryStrategy = odometryStrategy;
        this.chassisLimelight2dStrategy = chassisLimelight2dStrategy;
        this.xLimiter = xLimiter;
        this.yLimiter = yLimiter;
        this.thetaLimiter = thetaLimiter;
        this.autos = autos;
        this.robotContainer = robotContainer;
        commandFactory = robotContainer.getCommandFactory();
        dashboardLogging = robotContainer.getDashboardLogging();

        configureBindings();
    }

    // =========================== //
    /** Bindings & Default Commands */
    // =========================== //
    private void configureBindings() {

        /* Climber Limit Switch Triggers */
        new Trigger(climber::forwardLimit).onTrue(climber.hitForwardLimitCommand());
        new Trigger(climber::reverseLimit).onTrue(climber.hitReverseLimitCommand());
        new Trigger(climber.reverseLimitOnSupplier()).and(() -> commandFactory.getCancelClimbRequestShooterDown())
                .onTrue(pivot.runToHomeCommand()
                        .andThen(Commands.runOnce(() -> commandFactory.setCancelClimbRequestShooterDown(false)))); // I
        // hate
        // it
        // but
        // it
        // works
        // lol

        // ================ //
        /* Default Commands */
        // ================ //

        drivetrain.setDefaultCommand(drivetrain.applyRequest(() -> drive.withVelocityX(robotContainer.getXSpeed(true))
                .withVelocityY(robotContainer.getYSpeed(true)).withRotationalRate(robotContainer.getRotationSpeed())));

        conveyor.setDefaultCommand(conveyor.runMotorCommand(0.));
        infeed.setDefaultCommand(infeed.runMotorCommand(0.));

        // ================= //
        /* DRIVER CONTROLLER */
        // ================= //

        // ========================= //
        /* Infeed & Conveyor Control */
        // ========================= //

        /* Dumb Infeed */
        driverController.leftBumper()
                .onTrue(commandFactory.runBoth(true, VBusConstants.SLOW_CONVEYOR_VBUS, VBusConstants.INFEED_VBUS))
                .onFalse(commandFactory.coolNoteFixCommand(0.15).andThen(driverCamera.setShooterCameraCommand()));

        /* Smart Infeed */
        // driverController.leftTrigger()
        // .whileTrue(smartInfeedCommand().andThen(driverCamera.setShooterCameraCommand()));
        driverController.leftTrigger()
                .whileTrue(commandFactory.runBoth(true, VBusConstants.SLOW_CONVEYOR_VBUS, VBusConstants.INFEED_VBUS))
                .onFalse(commandFactory.conveyBackCommand(-1.5, 0.5).alongWith(driverCamera.setShooterCameraCommand()));

        // ========================== //
        /* Drivetain & Vision Control */
        // ========================== //

        /* Robot-Relative Drive */
        driverController.y()
                .toggleOnTrue(drivetrain.applyRequest(() -> robotRelativeDrive
                        .withVelocityX(robotContainer.getXSpeed(false)).withVelocityY(robotContainer.getYSpeed(false))
                        .withRotationalRate(robotContainer.getRotationSpeed())));

        /* X-Drive */
        driverController.x().whileTrue(drivetrain.applyRequest(() -> xDrive));

        /* Reset Field-Centric Heading */
        driverController.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative(new Pose2d())));

        /* Toggle Chassis Mode */
        driverController.rightBumper()
                .onTrue(Commands.runOnce(() -> commandFactory.setCurrentSpeed(DrivetrainConstants.SLOW_SPEED)))
                .onFalse(Commands.runOnce(() -> commandFactory.setCurrentSpeed(DrivetrainConstants.BASE_SPEED)));

        /* Shooter Lock */
        // driverController.leftStick().onTrue(magicLockCommand());

        // ========================= //
        /* Misc */
        // ========================= //

        /* End snap, limelight & stop all motors */
        driverController.rightStick().onTrue(commandFactory.stopAllCommand(true).alongWith(drivetrain.runOnce(() -> {
        })));

        driverController.a()
                .onTrue(commandFactory.runEntryCommand(() -> PASSING_SHOT, () -> ShotSpeeds.FAST)
                        .andThen(Commands.either(commandFactory.snapCommand(SnapDirection.BluePass),
                                commandFactory.snapCommand(SnapDirection.RedPass), () -> BeakUtils.allianceIsBlue())));

        // =================== //
        /* OPERATOR CONTROLLER */
        // =================== //

        // ========================= //
        /* Driver Help Control */
        // ========================= //

        /* Snap to Amp */
        operatorController.povUp().toggleOnTrue(commandFactory.snapCommand(SnapDirection.Left));

        // ======================= //
        /* Shooter Control */
        // ======================= //

        /* Spin Up Shooter */
        operatorController.leftTrigger()
                .onTrue(commandFactory.runEntryCommand(
                        () -> ShooterTable.calcShooterTableEntry(Feet.of(commandFactory.getCurrentIndex())),
                        () -> ShotSpeeds.FAST))
                .onFalse(commandFactory.stopAllCommand().andThen(Commands.runOnce(commandFactory::setCameraWithWait)));

        /* Convey Note */
        operatorController.rightTrigger().whileTrue(
                commandFactory.runBoth(false, VBusConstants.FAST_CONVEYOR_VBUS, VBusConstants.SLOW_INFEED_VBUS));

        /* Magic Shoot */
        operatorController.x().toggleOnTrue(commandFactory.magicShootCommand());

        /* Magic Shoot Strategies */
        operatorController.povRight().onTrue(commandFactory.setStrategyCommand(shooterLimelightStrategy));
        operatorController.povDown().onTrue(commandFactory.setStrategyCommand(chassisLimelight2dStrategy)
                .alongWith(chassisLimelight.setPipelineCommand(VisionConstants.TY_PIPELINE)));
        operatorController.povLeft().onTrue(commandFactory.setStrategyCommand(odometryStrategy)
                .alongWith(chassisLimelight.setPipelineCommand(VisionConstants.MEGATAG_PIPELINE)));

        /* Manual/Preset Mode */
        operatorController.back()
                .onTrue(Commands.runOnce(() -> commandFactory.setUseManual(!commandFactory.getUseManual()))
                        .andThen(dashboardLogging::pushIndexData));

        /* Shooter Table Index Up */
        operatorController.rightBumper()
                .onTrue(Commands.either(
                        Commands.runOnce(() -> commandFactory.setManualIndex(commandFactory.getManualIndex() + 1)),
                        Commands.runOnce(() -> commandFactory.setPresetIndex(commandFactory.getPresetIndex() + 1)),
                        () -> commandFactory.getUseManual()).andThen(dashboardLogging::pushIndexData));

        /* Shooter Table Index Down */
        operatorController.leftBumper()
                .onTrue(Commands.either(
                        Commands.runOnce(() -> commandFactory.setManualIndex(commandFactory.getManualIndex() - 1)),
                        Commands.runOnce(() -> commandFactory.setPresetIndex(commandFactory.getPresetIndex() - 1)),
                        () -> commandFactory.getUseManual()).andThen(dashboardLogging::pushIndexData));

        // ========================= //
        /* Pivot Control */
        // ========================= //

        /* Zero Pivot */
        operatorController.start().onTrue(commandFactory.zeroCommand());

        /* Run Pivot Zero */
        // operatorController.a().onTrue(pivot.runToHomeCommand());
        operatorController.a().onTrue(Commands.runOnce(() -> commandFactory.setSelectedStrategy(odometryStrategy))
                .andThen(commandFactory.shuttleCommand()));

        operatorController.y().onTrue(Commands.runOnce(() -> commandFactory.setSelectedStrategy(odometryStrategy))
                .andThen(commandFactory.shuttleShortCommand()));

        /* Zero Climber */
        operatorController.leftStick().onTrue(commandFactory.safeClimbCommand(climber.zeroCommand()));
        /* End snap, limelight & stop all motors */
        operatorController.rightStick().onTrue(commandFactory.stopAllCommand(true).alongWith(drivetrain.runOnce(() -> {
        })));

        // ================ //
        /* Amp & Trap Magic */
        // ================ //

        operatorController.b().onTrue(commandFactory.getAmpPrep()).onFalse(commandFactory.stopAllCommand(true));

        /* Full Outfeed: left Y */
        operatorController.axisGreaterThan(XboxController.Axis.kLeftY.value, 0.2)
                .or(operatorController.axisLessThan(XboxController.Axis.kLeftY.value, -0.2))
                .whileTrue(commandFactory.runThree(() -> -operatorController.getLeftY(),
                        () -> -operatorController.getLeftY(), () -> -operatorController.getLeftY()));
        // ==================== //
        /* EMERGENCY CONTROLLER */
        // ==================== //

        // ==================== //
        /* Manual Pivot Control */
        // ==================== //

        // /* Bump Pivot Up */
        emergencyController.rightBumper().onTrue(pivot.runOnce(() -> pivot.runToPosition(pivot.getPosition() + 1)));

        /* Bump Pivot Down */
        emergencyController.leftBumper().onTrue(pivot.runOnce(() -> pivot.runToPosition(pivot.getPosition() - 1)));

        // ============== //
        /* Manual Climber */
        // ============== //
        // =====================================================================================================================
        // =====================================================================================================================

        // AGS -- Override Emergency Controller triggers to ignore limit switches for
        // CORI -- change made Oct 21, 2024

        /* Climber Up */
        emergencyController.rightTrigger(0.2).whileTrue(climber.runMotorCommand(VBusConstants.CLIMBER_VBUS, true));

        // emergencyController.rightTrigger(0.2).whileTrue(
        // safeClimbCommand(climber.runMotorCommand(CLIMBER_VBUS, true)))
        // .onFalse(climber.stopCommand());

        /* Climber Down FULL SEND */
        emergencyController.leftTrigger(0.2).whileTrue(climber.runMotorCommand(-VBusConstants.CLIMBER_VBUS, true));

        // emergencyController.leftTrigger(0.2).whileTrue(
        // safeClimbCommand(climber.runMotorCommand(-FAST_CLIMBER_VBUS, true)))
        // .onFalse(climber.holdCurrentPositionCommand());

        // =====================================================================================================================
        // =====================================================================================================================

        /* Ready Climb */
        emergencyController.povUp()
                .onTrue(commandFactory.safeClimbCommand(
                        climber.runToPositionCommand(VBusConstants.CLIMBER_VBUS, ClimberPositions.READY, false)))
                .onFalse(climber.stopCommand());

        /* Climb */
        emergencyController.povDown()
                .onTrue(commandFactory.safeClimbCommand(
                        climber.runToPositionCommand(VBusConstants.CLIMBER_VBUS, ClimberPositions.CLIMB, true)))
                .onFalse(climber.holdCurrentPositionCommand());

        /* funk */
        emergencyController.povLeft().onTrue(commandFactory.safeClimbCommand(climber.holdCommand()))
                .onFalse(climber.stopCommand());

        /* Stop Hold */
        emergencyController.povRight().onTrue(Commands.runOnce(() -> climber.getCurrentCommand().cancel()));

        // ======================= //
        /* Trap & Climb Sequencing */
        // ======================= //

        /* Enable Trap */
        emergencyController.start()
                .onTrue(Commands.runOnce(() -> commandFactory.setEnableTrap(!commandFactory.getEnableTrap())));

        /* Enable Climber */
        emergencyController.back()
                .onTrue(Commands.runOnce(() -> commandFactory.setEnableClimber(!commandFactory.getEnableClimber())));

        /* Climb Sequence */
        emergencyController.a().onTrue(commandFactory.sequenceCommand());
        emergencyController.y().onTrue(commandFactory.dumbCancelClimbCommand());

        /* Prime Fan Pivot & Shooter Pivot */
        emergencyController.x().toggleOnTrue( // very bad
                Commands.startEnd(() -> {
                    fanPivot.runToTrap();
                    fan.runMotor(VBusConstants.FAN_VBUS);
                }, () -> {
                    fanPivot.hold();
                    fan.stop();
                }, fanPivot, fan));

        // emergencyController.b().toggleOnTrue(coolShootCommand());
        // emergencyController.y()
        // .toggleOnTrue(magicShootCommand(() -> selectedStrategy, ztrue,
        // Rotation2d.fromDegrees(-4.0)));
        emergencyController.b().onTrue(commandFactory.toggleTrapCommand());

        /* Full Outfeed: left Y */
        emergencyController.axisGreaterThan(XboxController.Axis.kLeftY.value, 0.2)
                .or(emergencyController.axisLessThan(XboxController.Axis.kLeftY.value, -0.2))
                .whileTrue(commandFactory.runThree(() -> -emergencyController.getLeftY(),
                        () -> -emergencyController.getLeftY(), () -> -emergencyController.getLeftY()));

        if (Utils.isSimulation()) {
            drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
        }

        drivetrain.registerTelemetry(logger::telemeterize);

    }
}