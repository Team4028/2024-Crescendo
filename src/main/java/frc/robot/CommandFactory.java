package frc.robot;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Meters;

import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.VBusConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Climber.ClimberPositions;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShotSpeeds;
import frc.robot.subsystems.Shooter.Slots;
import frc.robot.utils.BeakCommands;
import frc.robot.utils.BeakUtils;
import frc.robot.utils.ShooterTable;
import frc.robot.utils.ShooterTable.ShooterTableEntry;
import frc.robot.utils.ShootingStrategy;
import frc.robot.utils.SubAutos;
import frc.robot.utils.SubsystemContainer;

public class CommandFactory {

    // ==============================
    // SECTION: Enums
    // ==============================

    public enum SnapDirection {
        None(Double.NaN), Forward(0), Left(90), Back(180), Right(270), LeftTrap(-60.), RightTrap(60.), BluePass(
                -40.0),
        RedPass(-150.0);

        public double Angle;

        private SnapDirection(double angle) {
            Angle = angle;
        }
    }

    public enum ClimbSequence {
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

    // ==============================
    // SECTION: Member Variables
    // ==============================

    private final SubsystemContainer subsystems;
    private final SlewRateLimiter xLimiter, yLimiter, thetaLimiter;
    private final SwerveRequest.FieldCentricFacingAngle snapDrive;
    private final SubAutos autos;
    private final Map<ClimbSequence, Command> sequenceCommandMap;
    private final RobotContainer robotContainer;

    // ==============================
    // SECTION: Constants
    // ==============================
    private static final double PIVOT_UP_THRESHOLD = 40.0;

    // ==============================
    // SECTION: Robot States
    // ==============================
    private ShootingStrategy selectedStrategy;
    private ClimbSequence currentSequence = ClimbSequence.Default;
    private double currentSpeed = DrivetrainConstants.BASE_SPEED;
    private ShooterTableEntry entryToRun;
    private boolean enableClimber = false;
    private boolean cancelClimbRequestShooterDown = false;
    private boolean enableTrap = false;
    private boolean useMT2 = true;
    private double lastShot = 0.0;
    private double currentIndex = Constants.MAX_INDEX;
    private double manualIndex = Constants.MIN_INDEX;
    private int presetIndex = 0;
    private boolean useManual = false;

    public CommandFactory(SubsystemContainer subsystems, SlewRateLimiter xLimiter, SlewRateLimiter yLimiter,
            SlewRateLimiter thetaLimiter, SubAutos autos, RobotContainer robotContainer) {
        this.subsystems = subsystems;
        this.xLimiter = xLimiter;
        this.yLimiter = yLimiter;
        this.thetaLimiter = thetaLimiter;
        this.autos = autos;

        sequenceCommandMap = Map.of(ClimbSequence.Default, Commands.none(), ClimbSequence.Prep, prepClimbCommand(),
                ClimbSequence.Fan, fanReadyCommand(), ClimbSequence.Shoot, trapShootCommand(), ClimbSequence.End,
                endSequenceCommand(), ClimbSequence.Climb, climbCommand());

        this.robotContainer = robotContainer;

        snapDrive = new SwerveRequest.FieldCentricFacingAngle().withDeadband(DrivetrainConstants.MAX_SPEED * 0.035)
                .withDriveRequestType(DriveRequestType.Velocity);

        snapDrive.HeadingController = new PhoenixPIDController(10, 0., 0.);
        snapDrive.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public Command getAmpPrep() {
        return subsystems.pivot.runToClimbCommand()
                .alongWith(subsystems.whippy.whippyWheelsCommand(VBusConstants.WHIPPY_VBUS))
                .alongWith(subsystems.shooter.setSlotCommand(Shooter.Slots.AMP)
                        .andThen(subsystems.shooter.runShotCommand(ShotSpeeds.AMP)));
    }

    // ==============================
    // SECTION: Driving
    // ==============================

    /** Set Snap Direction Toggle */
    public Command snapCommand(SnapDirection direction) {
        return subsystems.drivetrain.applyRequest(() -> snapDrive.withVelocityX(robotContainer.getXSpeed(true))
                .withVelocityY(robotContainer.getYSpeed(true))
                .withTargetDirection(Rotation2d.fromDegrees(direction.Angle)));
    }

    // ==============================
    // SECTION: Conveying
    // ==============================

    /** Fix Note Sequence */
    public Command fixNoteCommand() {
        return runBoth(true, VBusConstants.FAST_CONVEYOR_VBUS, VBusConstants.INFEED_VBUS).withTimeout(0.25)
                .andThen(conveyBackCommand(-2.0, 0.25));
    }

    /** Fix Note Backwards */
    public Command conveyBackCommand(double rotations, double timeout) {
        return subsystems.shooter.spinMotorLeftCommand(VBusConstants.SHOOTER_BACKOUT_VBUS).repeatedly()
                .raceWith(subsystems.conveyor.runXRotations(rotations).withTimeout(timeout))
                .alongWith(subsystems.infeed.runMotorCommand(0.))
                .andThen(subsystems.shooter.stopCommand());
    }

    /** Special Note Fix */
    public Command coolNoteFixCommand(double timeout) {
        return subsystems.shooter.spinMotorLeftCommand(VBusConstants.SHOOTER_BACKOUT_VBUS).repeatedly()
                .alongWith(subsystems.infeed.runMotorCommand(0.)).alongWith(subsystems.conveyor.runMotorCommand(-0.2))
                .withTimeout(timeout)
                .andThen(subsystems.shooter.stopCommand()).andThen(subsystems.conveyor.brakeStopCommand());
    }

    /** Convey the Note */
    public Command conveyCommand() {
        return subsystems.conveyor.runXRotations(20.)
                .alongWith(subsystems.infeed.runMotorCommand(VBusConstants.SLOW_INFEED_VBUS));
    }

    /** Smart Infeed Command Generator */
    public Command smartInfeedCommand() {
        return runBoth(true, VBusConstants.SLOW_CONVEYOR_VBUS, VBusConstants.INFEED_VBUS)
                .until(subsystems.noteSensing.hasInfedSupplier()).andThen(runBoth(true, 0., 0.).withTimeout(0.1))
                .andThen(conveyBackCommand(-4.0, 0.5)).finallyDo(subsystems.shooter::stop);
    }

    public Command smartInfeedAutoCommand() {
        return runBoth(true, VBusConstants.SLOW_CONVEYOR_VBUS, VBusConstants.INFEED_VBUS)
                .until(subsystems.noteSensing.hasInfedSupplier()).andThen(runBoth(true, 0., 0.).withTimeout(0.1))
                .andThen(conveyBackCommand(-4.0, 0.1)).finallyDo(subsystems.shooter::stop);
    }

    /** Run both Conveyor and Infeed */
    public Command runBoth(boolean stopShooter, double conveyorVbus, double infeedVbus) {
        return subsystems.shooter.brakeStopCommand().onlyIf(() -> stopShooter).alongWith(
                subsystems.infeed.runMotorCommand(infeedVbus)
                        .alongWith(subsystems.conveyor.runMotorCommand(conveyorVbus)).repeatedly());
    }

    /** Run Conveyor, Infeed, and shooter if backwards */
    public Command runThree(Supplier<Double> conveyorVbus, Supplier<Double> infeedVbus, Supplier<Double> shooterVbus) {
        return new FunctionalCommand(() -> {
        }, () -> {
            subsystems.infeed.runMotor(infeedVbus.get());
            subsystems.conveyor.runMotor(conveyorVbus.get());
            subsystems.shooter.spinMotorRight(0.3 * shooterVbus.get());
            subsystems.shooter.spinMotorLeft(0.3 * shooterVbus.get());
        }, (z) -> subsystems.shooter.stop(), () -> false, subsystems.infeed, subsystems.conveyor);
    }

    // ==============================
    // SECTION: Stop/Zero
    // ==============================

    /** Stop all motors and zero everything */
    public Command stopAllCommand(boolean switchCamera) {
        return Commands.parallel(subsystems.infeed.stopCommand(), subsystems.conveyor.stopCommand(),
                subsystems.shooter.stopCommand().andThen(subsystems.shooter.setSlotCommand(Shooter.Slots.FAST)),
                subsystems.pivot.runToHomeCommand(),
                subsystems.fan.stopCommand(), subsystems.fanPivot.runToHomeCommand(), subsystems.whippy.stopCommand(),
                subsystems.driverCamera.setInfeedCameraCommand().onlyIf(() -> switchCamera),
                Commands.runOnce(() -> currentSequence = ClimbSequence.Default));
    }

    /** That but don't reset the camera */
    public Command stopAllCommand() {
        return stopAllCommand(false);
    }

    /** Zeroing Command */
    public Command zeroCommand() {
        return subsystems.pivot.zeroCommand().alongWith(subsystems.fanPivot.runToHomeCommand());// .alongWith(climber.zeroCommand());
    }

    /** Asynchronous Zero */
    public void zero() {
        zeroCommand().schedule();
    }

    // ==============================
    // SECTION: Shooting
    // ==============================

    /**
     * Generate a command to continuously run the shooter while aligning to the
     * target.
     * 
     * @param strategy
     *                 The {@link ShootingStrategy} to use.
     */
    public Command magicLockCommand(Supplier<ShootingStrategy> strategy) {
        return fixNoteCommand().unless(subsystems.noteSensing.conveyorSeesNoteSupplier())
                .andThen(subsystems.driverCamera.setShooterCameraCommand())
                .andThen(subsystems.drivetrain
                        .speakerLock(() -> robotContainer.getXSpeed(true), () -> robotContainer.getYSpeed(true),
                                strategy)
                        .alongWith(runEntryCommand(() -> strategy.get().getTargetEntry(true), () -> ShotSpeeds.FAST)
                                .repeatedly()))
                .finallyDo(subsystems.driverCamera::setInfeedCamera);
    }

    /**
     * Generate a command to continuously run the shooter while aligning with the
     * selected strategy.
     */
    public Command magicLockCommand() {
        return magicLockCommand(() -> selectedStrategy);
    }

    public Command shuttleCommand() {
        return updateDrivePoseMT2Command()
                .andThen(
                        subsystems.drivetrain
                                .applyRequest(() -> snapDrive
                                        .withTargetDirection(BeakUtils
                                                .passingTranslation(
                                                        subsystems.odometryStrategy.getDrivetrainFieldTranslation(true))
                                                .getAngle())
                                        .withVelocityX(robotContainer.getXSpeed(true))
                                        .withVelocityY(robotContainer.getYSpeed(true)))
                                .alongWith(runEntryCommand(
                                        () -> ShooterTable.calcShuttleTableEntry(Meters
                                                .of(subsystems.odometryStrategy.getDrivetrainGoalTranslation(true)
                                                        .getNorm())),
                                        () -> ShotSpeeds.FAST))
                                .repeatedly());
    }

    public Command shuttleShortCommand() {
        return updateDrivePoseMT2Command()
                .andThen(
                        subsystems.drivetrain
                                .applyRequest(
                                        () -> snapDrive
                                                .withTargetDirection(BeakUtils
                                                        .passingTranslation(
                                                                subsystems.odometryStrategy
                                                                        .getDrivetrainFieldTranslation(true))
                                                        .getAngle()
                                                        .rotateBy(BeakUtils.allianceIsBlue()
                                                                ? Constants.SHUTTLE_SHORT_OFFSET_BLUE
                                                                : Constants.SHUTTLE_SHORT_OFFSET_RED))
                                                .withVelocityX(robotContainer.getXSpeed(true))
                                                .withVelocityY(robotContainer.getYSpeed(true)))
                                .alongWith(runEntryCommand(
                                        () -> ShooterTable.calcShortShuttleTableEntry(Meters
                                                .of(subsystems.odometryStrategy.getDrivetrainGoalTranslation(true)
                                                        .getNorm())),
                                        () -> ShotSpeeds.FAST))
                                .repeatedly());
    }

    /**
     * Generate a command to run the shooter, convey, & stop everything thereafter.
     * 
     * @param entry
     *              The entry to run.
     */
    public Command shootCommand(Supplier<ShooterTableEntry> entry) {
        return subsystems.driverCamera.setShooterCameraCommand().andThen(runEntryCommand(entry, () -> ShotSpeeds.FAST))
                .andThen(Commands.waitUntil(shooterAndPivotReady())).andThen(conveyCommand().withTimeout(0.75))
                .finallyDo(() -> {
                    subsystems.shooter.stop();
                    subsystems.pivot.runToPosition(Pivot.HOLD_POSITION);
                    setCameraWithWait();
                });
    }

    /**
     * Generate a command to shoot based on a target distance.
     * 
     * @param distance
     *                 The target distance, in feet.
     */
    public Command shootCommand(double distance) {
        return shootCommand(() -> ShooterTable.calcShooterTableEntry(Feet.of(distance)));
    }

    /**
     * Generate a command to use the specified entry to run a magic shot.
     * 
     * @param entry
     *                 The entry to run.
     * @param strategy
     *                 The strategy to use for rotation.
     * @param lock
     *                 Whether or not to align the drivetrain.
     */
    public Command magicShootCommand(Supplier<ShooterTableEntry> entry, Supplier<ShootingStrategy> strategy,
            boolean lock, Rotation2d offset) {
        return Commands.runOnce(() -> entryToRun = entry.get())
                .andThen(
                        fixNoteCommand().unless(subsystems.noteSensing.conveyorSeesNoteSupplier()))
                .andThen(subsystems.driverCamera.setShooterCameraCommand())
                .andThen(runEntryCommand(() -> entryToRun, () -> ShotSpeeds.FAST)
                        .alongWith(subsystems.drivetrain.speakerAlign(strategy, offset).withTimeout(0.5)
                                .unless(() -> Math.abs(strategy.get().getTargetOffset().getDegrees()) < 0.25))
                        .onlyIf(() -> lock))
                .andThen(shootCommand(() -> entryToRun));
    }

    /**
     * Generate a command to use the specified entry to run a magic shot.
     * 
     * @param entry
     *                 The entry to run.
     * @param strategy
     *                 The strategy to use for rotation.
     * @param lock
     *                 Whether or not to align the drivetrain.
     */
    public Command magicShootCommand(Supplier<ShooterTableEntry> entry, Supplier<ShootingStrategy> strategy,
            boolean lock) {
        return magicShootCommand(entry, strategy, lock, ShootingStrategy.OFFSET);
    }

    /**
     * Generate a command to use the specified distance to run a magic shot.
     * 
     * @param distance
     *                 The shot to run.
     * @param strategy
     *                 The strategy to use for rotation.
     * @param lock
     *                 Whether or not to align the drivetrain.
     */
    public Command magicShootCommand(double distance, Supplier<ShootingStrategy> strategy, boolean lock) {
        return magicShootCommand(() -> ShooterTable.calcShooterTableEntry(Feet.of(distance)), strategy, lock);
    }

    /**
     * Generate a command to use the specified strategy to run a magic shot.
     * 
     * @param strategy
     *                 The {@link ShootingStrategy} to use.
     * @param lock
     *                 Whether or not to align the drivetrain.
     */
    public Command magicShootCommand(Supplier<ShootingStrategy> strategy, boolean lock, Rotation2d offset) {
        return magicShootCommand(() -> strategy.get().getTargetEntry(), strategy, lock, offset);
    }

    /**
     * Generate a command to use the specified strategy to run a magic shot.
     * 
     * @param strategy
     *                 The {@link ShootingStrategy} to use.
     * @param lock
     *                 Whether or not to align the drivetrain.
     */
    public Command magicShootCommand(Supplier<ShootingStrategy> strategy, boolean lock) {
        return magicShootCommand(() -> strategy.get().getTargetEntry(), strategy, lock, ShootingStrategy.OFFSET);
    }

    /**
     * Generate a command to use the specified strategy to run a magic, aligning
     * shot.
     * 
     * @param strategy
     *                 The {@link ShootingStrategy} to use.
     */
    public Command magicShootCommand(Supplier<ShootingStrategy> strategy) {
        return magicShootCommand(strategy, true);
    }

    /**
     * Generate a command to use the specified strategy to run a magic, aligning
     * shot using the current strategy.
     */
    public Command magicShootCommand() {
        return magicShootCommand(() -> selectedStrategy, true);
    }

    /** Run a Shooter Table Entry */
    public Command runEntryCommand(Supplier<ShooterTableEntry> entry, Supplier<ShotSpeeds> speed) {
        return subsystems.shooter.runEntryCommand(entry, speed)
                .alongWith(subsystems.pivot.runToPositionCommand(() -> entry.get().Angle))
                .alongWith(Commands.runOnce(() -> lastShot = entry.get().Distance.in(Feet))
                        .onlyIf(() -> entry.get().Distance != null));
    }

    /** Shooter & Pivot Both Ready */
    public BooleanSupplier shooterAndPivotReady() {
        return () -> subsystems.shooter.isReady() && subsystems.pivot.inPosition();
    }

    /** Set the strategy to use for shooting/locking. */
    public Command setStrategyCommand(ShootingStrategy strategy) {
        return Commands.runOnce(() -> selectedStrategy = strategy);
    }

    // ==============================
    // SECTION: Auton Stuff
    // ==============================

    public Command mirroredPathfindingShotCommand(double pivotAngle, Pose2d target, double scale, double endVelocity) {
        Pose2d redPose = new Pose2d(target.getTranslation(), target.getRotation().minus(Rotation2d.fromDegrees(6.)));

        return runBoth(false, VBusConstants.FAST_CONVEYOR_VBUS, VBusConstants.INFEED_VBUS).repeatedly()
                .alongWith(subsystems.pivot.runToPositionCommand(pivotAngle))
                .alongWith(Commands.either(subsystems.drivetrain.mirrorablePathFindCommand(target, scale, endVelocity),
                        subsystems.drivetrain.mirrorablePathFindCommand(redPose, scale, endVelocity),
                        BeakUtils::allianceIsBlue));
    }

    /** Pathfinding Auton Shot */
    public Command pathfindingShotCommand(double targetDistance, Pose2d target, double scale, double endVelocity) {
        return subsystems.drivetrain.mirrorablePathFindCommand(target, scale, endVelocity)
                .deadlineWith(smartInfeedCommand()
                        .withTimeout(0.6).andThen(coolNoteFixCommand(0.2))
                        .andThen(subsystems.shooter.runShotCommand(ShotSpeeds.FAST)))
                .andThen(shootCommand(targetDistance));
    }

    // ==============================
    // SECTION: Climbing
    // ==============================

    /** Only run climb command if pivot good */
    public Command safeClimbCommand(Command command) {
        return command.onlyIf(() -> subsystems.pivot.getPosition() > PIVOT_UP_THRESHOLD);
    }

    public Command safePivotCommand(Command command) {
        return command.onlyIf(subsystems.climber::reverseLimitOn);
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
    public Command prepClimbCommand() {
        // Pivot => Trap Position
        // Shooter => Trap Mode
        return subsystems.pivot.runToTrapCommand().alongWith(subsystems.shooter.setSlotCommand(Slots.TRAP));
    }

    /** Prime the fan & shooter */
    public Command fanReadyCommand() {
        // Fan Pivot Up
        // Shooting Routine
        // Climber Up
        return subsystems.fanPivot.runToTrapCommand()
                .alongWith(subsystems.fan.runMotorCommand(VBusConstants.FAN_VBUS)
                        .andThen(BeakCommands.repeatCommand(fixNoteCommand(), 2))
                        .andThen(subsystems.shooter.runShotCommand(ShotSpeeds.TRAP)).onlyIf(() -> enableTrap))
                .alongWith(subsystems.climber
                        .runToPositionCommand(VBusConstants.CLIMBER_VBUS, ClimberPositions.READY, false)
                        .onlyIf(() -> enableClimber && subsystems.pivot.getPosition() > PIVOT_UP_THRESHOLD));
    }

    public Command dumbCancelClimbCommand() {

        return Commands.runOnce(() -> {
            currentSequence = ClimbSequence.Default;
            cancelClimbRequestShooterDown = true;
        }).andThen(subsystems.fanPivot.runToTrapCommand(), Commands.waitSeconds(0.2),
                safeClimbCommand(
                        subsystems.climber.zeroCommand().until(subsystems.climber.reverseLimitOnSupplier()).alongWith(
                                Commands.waitSeconds(1).andThen(subsystems.fanPivot.runToHomeCommand()),
                                subsystems.fan.stopCommand(),
                                subsystems.shooter.stopCommand())));
    }

    /** Shoot */
    public Command trapShootCommand() {
        // Shoot Note
        return conveyCommand();
    }

    /** Home everything */
    public Command endSequenceCommand() {
        // Fan Down
        // Fan Stop
        // Shooter Stop/Fast Mode
        // Pivot Down if not climbing
        return subsystems.fanPivot.runToHomeCommand().alongWith(subsystems.fan.stopCommand())
                .alongWith(subsystems.shooter.stopCommand().andThen(subsystems.shooter.setSlotCommand(Slots.FAST)))
                .alongWith(safePivotCommand(subsystems.pivot.runToHomeCommand()).unless(() -> enableClimber));
    }

    /** Climb!!!!! */
    public Command climbCommand() {
        // Climber Down
        return safeClimbCommand(
                subsystems.climber.runToPositionCommand(VBusConstants.CLIMBER_VBUS, ClimberPositions.CLIMB, true));
    }

    /** Update Sequence */
    public Command sequenceCommand() {
        return Commands.runOnce(this::incrementSequence)
                .andThen(Commands.select(sequenceCommandMap, () -> currentSequence));
    }

    public Command toggleTrapCommand() {
        return Commands.either(subsystems.pivot.runToTrapCommand(),
                safeClimbCommand(subsystems.climber.zeroCommand().until(subsystems.climber::reverseLimitOn))
                        .andThen(subsystems.pivot.runToHomeCommand()),
                subsystems.pivot::getIsAtHome);
    }

    // ==============================
    // SECTION: Vision
    // ==============================

    /** Set infeed camera asynchronously */
    public void setCameraWithWait() {
        Commands.waitSeconds(VisionConstants.CAMERA_SWITCH_TIMEOUT)
                .andThen(subsystems.driverCamera.setInfeedCameraCommand())
                .schedule();
    }

    public void updateMTRot() {
        subsystems.chassisLimelight.setRobotRotationMT2(subsystems.drivetrain.getRotation().getDegrees());
        subsystems.infeedLimelight3G.setRobotRotationMT2(subsystems.drivetrain.getRotation().getDegrees());

    }

    public void updateDrivePoseMT2() {
        updateMTRot();

        if (!useMT2)
            return;

        // Apply Chassis Limelight
        var visionResult = subsystems.chassisLimelight.getBotposeEstimateMT2();
        var visionStdDevs = subsystems.chassisLimelight.getSTDevsXY(subsystems.drivetrain);
        if (visionStdDevs.isPresent()) {
            subsystems.drivetrain.addVisionMeasurement(visionResult.pose, visionResult.timestampSeconds,
                    VecBuilder.fill(visionStdDevs.get()[0], visionStdDevs.get()[1], Double.MAX_VALUE));

        }

        // Apply Infeed Limelight
        visionResult = subsystems.infeedLimelight3G.getBotposeEstimateMT2();
        visionStdDevs = subsystems.infeedLimelight3G.getSTDevsXY(subsystems.drivetrain);
        if (visionStdDevs.isPresent())
            subsystems.drivetrain.addVisionMeasurement(visionResult.pose, visionResult.timestampSeconds,
                    VecBuilder.fill(visionStdDevs.get()[0], visionStdDevs.get()[1], Double.MAX_VALUE));
    }

    public Command updateDrivePoseMT2Command() {
        return Commands.runOnce(this::updateDrivePoseMT2);
    }

    public void setMT2Pipeline() {
        subsystems.chassisLimelight.setPipeline(VisionConstants.MEGATAG_PIPELINE);
        selectedStrategy = subsystems.odometryStrategy;
    }

    public void setChassisPipeline() {
        selectedStrategy = subsystems.chassisLimelight2dStrategy;
        subsystems.chassisLimelight.setPipeline(VisionConstants.TY_PIPELINE);
    }

    public void setTeleopMT2RotationThresholds() {
        subsystems.chassisLimelight.setTeleopMT2Threshold();
        subsystems.infeedLimelight3G.setTeleopMT2Threshold();
    }

    public void setAutonMT2RotationThresholds() {
        subsystems.chassisLimelight.setAutonMT2Threshold();
        subsystems.infeedLimelight3G.setAutonMT2Threshold();
    }

    // ==============================
    // SECTION: Misc.
    // ==============================

    /** Stop Shooter */
    public void stopShooter() {
        subsystems.shooter.stop();
    }

    /** Push limelight data to the CANdle */
    public Command encodeLimelights() {
        return subsystems.candle.encodeLimelights(subsystems.chassisLimelight, subsystems.chassisLimelight,
                subsystems.infeedLimelight3G);
    }

    // ==============================
    // SECTION: Getters/Setters
    // ==============================

    public ShootingStrategy getSelectedStrategy() {
        return selectedStrategy;
    }

    public void setSelectedStrategy(ShootingStrategy selectedStrategy) {
        this.selectedStrategy = selectedStrategy;
    }

    public boolean getCancelClimbRequestShooterDown() {
        return cancelClimbRequestShooterDown;
    }

    public void setCancelClimbRequestShooterDown(boolean cancelClimbRequestShooterDown) {
        this.cancelClimbRequestShooterDown = cancelClimbRequestShooterDown;
    }

    public boolean getEnableClimber() {
        return enableClimber;
    }

    public void setEnableClimber(boolean enableClimber) {
        this.enableClimber = enableClimber;
    }

    public boolean getEnableTrap() {
        return enableTrap;
    }

    public void setEnableTrap(boolean enableTrap) {
        this.enableTrap = enableTrap;
    }

    public boolean getUseMT2() {
        return useMT2;
    }

    public void setUseMT2(boolean useMT2) {
        this.useMT2 = useMT2;
    }

    public double getLastShot() {
        return lastShot;
    }

    public void setLastShot(double lastShot) {
        this.lastShot = lastShot;
    }

    public double getCurrentSpeed() {
        return currentSpeed;
    }

    public void setCurrentSpeed(double currentSpeed) {
        this.currentSpeed = currentSpeed;
    }

    public double getCurrentIndex() {
        return currentIndex;
    }

    public void setCurrentIndex(double currentIndex) {
        this.currentIndex = currentIndex;
    }

    public ClimbSequence getCurrentSequence() {
        return currentSequence;
    }

    public void setCurrentSequence(ClimbSequence currentSequence) {
        this.currentSequence = currentSequence;
    }

    public double getManualIndex() {
        return manualIndex;
    }

    public void setManualIndex(double manualIndex) {
        this.manualIndex = manualIndex;
    }

    public int getPresetIndex() {
        return presetIndex;
    }

    public void setPresetIndex(int presetIndex) {
        this.presetIndex = presetIndex;
    }

    public boolean getUseManual() {
        return useManual;
    }

    public void setUseManual(boolean useManual) {
        this.useManual = useManual;
    }
}
