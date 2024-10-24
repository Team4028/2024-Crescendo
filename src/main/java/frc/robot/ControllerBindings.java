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
import frc.robot.subsystems.Climber.ClimberPositions;
import frc.robot.subsystems.Shooter.ShotSpeeds;
import frc.robot.utils.BeakUtils;
import frc.robot.utils.ShooterTable;
import frc.robot.utils.ShooterTable.ShooterTableEntry;
import frc.robot.utils.SubAutos;
import frc.robot.utils.SubsystemContainer;

public class ControllerBindings {

    private static ShooterTableEntry PASSING_SHOT = new ShooterTableEntry(Feet.zero(), 4, 0.69, Feet.zero());// 30.9;

    private final CommandXboxController driverController, operatorController, emergencyController;
    private final SubsystemContainer subsystems;
    private final SlewRateLimiter xLimiter, yLimiter, thetaLimiter;
    private final SubAutos autos;
    private final CommandFactory commandFactory;
    private final DashboardLogging dashboardLogging;
    private final RobotContainer robotContainer;
    private final SwerveRequest.FieldCentric drive;
    private final SwerveRequest.RobotCentric robotRelativeDrive;
    private final SwerveRequest.SwerveDriveBrake xDrive;
    private final Telemetry logger;

    public ControllerBindings(int[] controllerIDs, SubsystemContainer subsystems, SlewRateLimiter xLimiter,
            SlewRateLimiter yLimiter,
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

        this.subsystems = subsystems;
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
        new Trigger(subsystems.climber::forwardLimit).onTrue(subsystems.climber.hitForwardLimitCommand());
        new Trigger(subsystems.climber::reverseLimit).onTrue(subsystems.climber.hitReverseLimitCommand());
        new Trigger(subsystems.climber.reverseLimitOnSupplier())
                .and(() -> commandFactory.getCancelClimbRequestShooterDown())
                .onTrue(subsystems.pivot.runToHomeCommand()
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

        subsystems.drivetrain.setDefaultCommand(subsystems.drivetrain.applyRequest(() -> drive
                .withVelocityX(robotContainer.getXSpeed(true))
                .withVelocityY(robotContainer.getYSpeed(true)).withRotationalRate(robotContainer.getRotationSpeed())));

        subsystems.conveyor.setDefaultCommand(subsystems.conveyor.runMotorCommand(0.));
        subsystems.infeed.setDefaultCommand(subsystems.infeed.runMotorCommand(0.));

        // ================= //
        /* DRIVER CONTROLLER */
        // ================= //

        // ========================= //
        /* Infeed & Conveyor Control */
        // ========================= //

        /* Dumb Infeed */
        driverController.leftBumper()
                .onTrue(commandFactory.runBoth(true, VBusConstants.SLOW_CONVEYOR_VBUS, VBusConstants.INFEED_VBUS))
                .onFalse(commandFactory.coolNoteFixCommand(0.15)
                        .andThen(subsystems.driverCamera.setShooterCameraCommand()));

        /* Smart Infeed */
        // driverController.leftTrigger()
        // .whileTrue(smartInfeedCommand().andThen(driverCamera.setShooterCameraCommand()));
        driverController.leftTrigger()
                .whileTrue(commandFactory.runBoth(true, VBusConstants.SLOW_CONVEYOR_VBUS, VBusConstants.INFEED_VBUS))
                .onFalse(commandFactory.conveyBackCommand(-1.5, 0.5)
                        .alongWith(subsystems.driverCamera.setShooterCameraCommand()));

        // ========================== //
        /* Drivetain & Vision Control */
        // ========================== //

        /* Robot-Relative Drive */
        driverController.y()
                .toggleOnTrue(subsystems.drivetrain.applyRequest(() -> robotRelativeDrive
                        .withVelocityX(robotContainer.getXSpeed(false)).withVelocityY(robotContainer.getYSpeed(false))
                        .withRotationalRate(robotContainer.getRotationSpeed())));

        /* X-Drive */
        driverController.x().whileTrue(subsystems.drivetrain.applyRequest(() -> xDrive));

        /* Reset Field-Centric Heading */
        driverController.start()
                .onTrue(subsystems.drivetrain.runOnce(() -> subsystems.drivetrain.seedFieldRelative(new Pose2d())));

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
        driverController.rightStick()
                .onTrue(commandFactory.stopAllCommand(true).alongWith(subsystems.drivetrain.runOnce(() -> {
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
        operatorController.povRight().onTrue(commandFactory.setStrategyCommand(subsystems.shooterLimelightStrategy));
        operatorController.povDown().onTrue(commandFactory.setStrategyCommand(subsystems.chassisLimelight2dStrategy)
                .alongWith(subsystems.chassisLimelight.setPipelineCommand(VisionConstants.TY_PIPELINE)));
        operatorController.povLeft().onTrue(commandFactory.setStrategyCommand(subsystems.odometryStrategy)
                .alongWith(subsystems.chassisLimelight.setPipelineCommand(VisionConstants.MEGATAG_PIPELINE)));

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
        operatorController.a()
                .onTrue(Commands.runOnce(() -> commandFactory.setSelectedStrategy(subsystems.odometryStrategy))
                        .andThen(commandFactory.shuttleCommand()));

        operatorController.y()
                .onTrue(Commands.runOnce(() -> commandFactory.setSelectedStrategy(subsystems.odometryStrategy))
                        .andThen(commandFactory.shuttleShortCommand()));

        /* Zero Climber */
        operatorController.leftStick().onTrue(commandFactory.safeClimbCommand(subsystems.climber.zeroCommand()));
        /* End snap, limelight & stop all motors */
        operatorController.rightStick()
                .onTrue(commandFactory.stopAllCommand(true).alongWith(subsystems.drivetrain.runOnce(() -> {
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
        emergencyController.rightBumper().onTrue(
                subsystems.pivot.runOnce(() -> subsystems.pivot.runToPosition(subsystems.pivot.getPosition() + 1)));

        /* Bump Pivot Down */
        emergencyController.leftBumper().onTrue(
                subsystems.pivot.runOnce(() -> subsystems.pivot.runToPosition(subsystems.pivot.getPosition() - 1)));

        // ============== //
        /* Manual Climber */
        // ============== //
        // =====================================================================================================================
        // =====================================================================================================================

        // AGS -- Override Emergency Controller triggers to ignore limit switches for
        // CORI -- change made Oct 21, 2024

        /* Climber Up */
        emergencyController.rightTrigger(0.2)
                .whileTrue(subsystems.climber.runMotorCommand(VBusConstants.CLIMBER_VBUS, true));

        // emergencyController.rightTrigger(0.2).whileTrue(
        // safeClimbCommand(climber.runMotorCommand(CLIMBER_VBUS, true)))
        // .onFalse(climber.stopCommand());

        /* Climber Down FULL SEND */
        emergencyController.leftTrigger(0.2)
                .whileTrue(subsystems.climber.runMotorCommand(-VBusConstants.CLIMBER_VBUS, true));

        // emergencyController.leftTrigger(0.2).whileTrue(
        // safeClimbCommand(climber.runMotorCommand(-FAST_CLIMBER_VBUS, true)))
        // .onFalse(climber.holdCurrentPositionCommand());

        // =====================================================================================================================
        // =====================================================================================================================

        /* Ready Climb */
        emergencyController.povUp()
                .onTrue(commandFactory.safeClimbCommand(
                        subsystems.climber.runToPositionCommand(VBusConstants.CLIMBER_VBUS, ClimberPositions.READY,
                                false)))
                .onFalse(subsystems.climber.stopCommand());

        /* Climb */
        emergencyController.povDown()
                .onTrue(commandFactory.safeClimbCommand(
                        subsystems.climber.runToPositionCommand(VBusConstants.CLIMBER_VBUS, ClimberPositions.CLIMB,
                                true)))
                .onFalse(subsystems.climber.holdCurrentPositionCommand());

        /* funk */
        emergencyController.povLeft().onTrue(commandFactory.safeClimbCommand(subsystems.climber.holdCommand()))
                .onFalse(subsystems.climber.stopCommand());

        /* Stop Hold */
        emergencyController.povRight().onTrue(Commands.runOnce(() -> subsystems.climber.getCurrentCommand().cancel()));

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
                    subsystems.fanPivot.runToTrap();
                    subsystems.fan.runMotor(VBusConstants.FAN_VBUS);
                }, () -> {
                    subsystems.fanPivot.hold();
                    subsystems.fan.stop();
                }, subsystems.fanPivot, subsystems.fan));

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
            subsystems.drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
        }

        subsystems.drivetrain.registerTelemetry(logger::telemeterize);

    }
}