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
import frc.robot.utils.DriverCamera;
import frc.robot.utils.Limelight;
import frc.robot.utils.NoteSensing;
import frc.robot.utils.ShooterTable;
import frc.robot.utils.ShooterTable.ShooterTableEntry;
import frc.robot.utils.ShootingStrategy;
import frc.robot.utils.SubAutos;

public class CommandFactory {

    // ==============================
    // SECTION: Enums
    // ==============================

    public enum SnapDirection {
        None(Double.NaN), Forward(0), Left(90), Back(180), Right(270), LeftTrap(-60.), RightTrap(60.), BluePass(
                -40.0), RedPass(-150.0);

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

    public CommandFactory(CommandSwerveDrivetrain drivetrain, Infeed infeed, Shooter shooter, Conveyor conveyor,
            Climber climber, Pivot pivot, Fan fan, FanPivot fanPivot, Whippy whippy, Candle candle,
            NoteSensing noteSensing, DriverCamera driverCamera, Limelight shooterLimelight, Limelight chassisLimelight,
            Limelight infeedLimelight3G, ShootingStrategy shooterLimelightStrategy, ShootingStrategy odometryStrategy,
            ShootingStrategy chassisLimelight2dStrategy, SlewRateLimiter xLimiter, SlewRateLimiter yLimiter,
            SlewRateLimiter thetaLimiter, SubAutos autos, RobotContainer robotContainer) {
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
        return pivot.runToClimbCommand().alongWith(whippy.whippyWheelsCommand(VBusConstants.WHIPPY_VBUS))
                .alongWith(shooter.setSlotCommand(Shooter.Slots.AMP).andThen(shooter.runShotCommand(ShotSpeeds.AMP)));
    }

    // ==============================
    // SECTION: Driving
    // ==============================

    /** Set Snap Direction Toggle */
    public Command snapCommand(SnapDirection direction) {
        return drivetrain.applyRequest(() -> snapDrive.withVelocityX(robotContainer.getXSpeed(true))
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
        return shooter.spinMotorLeftCommand(VBusConstants.SHOOTER_BACKOUT_VBUS).repeatedly()
                .raceWith(conveyor.runXRotations(rotations).withTimeout(timeout)).alongWith(infeed.runMotorCommand(0.))
                .andThen(shooter.stopCommand());
    }

    /** Special Note Fix */
    public Command coolNoteFixCommand(double timeout) {
        return shooter.spinMotorLeftCommand(VBusConstants.SHOOTER_BACKOUT_VBUS).repeatedly()
                .alongWith(infeed.runMotorCommand(0.)).alongWith(conveyor.runMotorCommand(-0.2)).withTimeout(timeout)
                .andThen(shooter.stopCommand()).andThen(conveyor.brakeStopCommand());
    }

    /** Convey the Note */
    public Command conveyCommand() {
        return conveyor.runXRotations(20.).alongWith(infeed.runMotorCommand(VBusConstants.SLOW_INFEED_VBUS));
    }

    /** Smart Infeed Command Generator */
    public Command smartInfeedCommand() {
        return runBoth(true, VBusConstants.SLOW_CONVEYOR_VBUS, VBusConstants.INFEED_VBUS)
                .until(noteSensing.hasInfedSupplier()).andThen(runBoth(true, 0., 0.).withTimeout(0.1))
                .andThen(conveyBackCommand(-4.0, 0.5)).finallyDo(shooter::stop);
    }

    public Command smartInfeedAutoCommand() {
        return runBoth(true, VBusConstants.SLOW_CONVEYOR_VBUS, VBusConstants.INFEED_VBUS)
                .until(noteSensing.hasInfedSupplier()).andThen(runBoth(true, 0., 0.).withTimeout(0.1))
                .andThen(conveyBackCommand(-4.0, 0.1)).finallyDo(shooter::stop);
    }

    /** Run both Conveyor and Infeed */
    public Command runBoth(boolean stopShooter, double conveyorVbus, double infeedVbus) {
        return shooter.brakeStopCommand().onlyIf(() -> stopShooter).alongWith(
                infeed.runMotorCommand(infeedVbus).alongWith(conveyor.runMotorCommand(conveyorVbus)).repeatedly());
    }

    /** Run Conveyor, Infeed, and shooter if backwards */
    public Command runThree(Supplier<Double> conveyorVbus, Supplier<Double> infeedVbus, Supplier<Double> shooterVbus) {
        return new FunctionalCommand(() -> {
        }, () -> {
            infeed.runMotor(infeedVbus.get());
            conveyor.runMotor(conveyorVbus.get());
            shooter.spinMotorRight(0.3 * shooterVbus.get());
            shooter.spinMotorLeft(0.3 * shooterVbus.get());
        }, (z) -> shooter.stop(), () -> false, infeed, conveyor);
    }

    // ==============================
    // SECTION: Stop/Zero
    // ==============================

    /** Stop all motors and zero everything */
    public Command stopAllCommand(boolean switchCamera) {
        return Commands.parallel(infeed.stopCommand(), conveyor.stopCommand(),
                shooter.stopCommand().andThen(shooter.setSlotCommand(Shooter.Slots.FAST)), pivot.runToHomeCommand(),
                fan.stopCommand(), fanPivot.runToHomeCommand(), whippy.stopCommand(),
                driverCamera.setInfeedCameraCommand().onlyIf(() -> switchCamera),
                Commands.runOnce(() -> currentSequence = ClimbSequence.Default));
    }

    /** That but don't reset the camera */
    public Command stopAllCommand() {
        return stopAllCommand(false);
    }

    /** Zeroing Command */
    public Command zeroCommand() {
        return pivot.zeroCommand().alongWith(fanPivot.runToHomeCommand());// .alongWith(climber.zeroCommand());
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
     *            The {@link ShootingStrategy} to use.
     */
    public Command magicLockCommand(Supplier<ShootingStrategy> strategy) {
        return fixNoteCommand().unless(noteSensing.conveyorSeesNoteSupplier())
                .andThen(driverCamera.setShooterCameraCommand())
                .andThen(drivetrain
                        .speakerLock(() -> robotContainer.getXSpeed(true), () -> robotContainer.getYSpeed(true),
                                strategy)
                        .alongWith(runEntryCommand(() -> strategy.get().getTargetEntry(true), () -> ShotSpeeds.FAST)
                                .repeatedly()))
                .finallyDo(driverCamera::setInfeedCamera);
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
                        drivetrain
                                .applyRequest(() -> snapDrive
                                        .withTargetDirection(BeakUtils
                                                .passingTranslation(
                                                        odometryStrategy.getDrivetrainFieldTranslation(true))
                                                .getAngle())
                                        .withVelocityX(robotContainer.getXSpeed(true))
                                        .withVelocityY(robotContainer.getYSpeed(true)))
                                .alongWith(runEntryCommand(
                                        () -> ShooterTable.calcShuttleTableEntry(Meters
                                                .of(odometryStrategy.getDrivetrainGoalTranslation(true).getNorm())),
                                        () -> ShotSpeeds.FAST))
                                .repeatedly());
    }

    public Command shuttleShortCommand() {
        return updateDrivePoseMT2Command()
                .andThen(
                        drivetrain
                                .applyRequest(
                                        () -> snapDrive
                                                .withTargetDirection(BeakUtils
                                                        .passingTranslation(
                                                                odometryStrategy.getDrivetrainFieldTranslation(true))
                                                        .getAngle()
                                                        .rotateBy(BeakUtils.allianceIsBlue()
                                                                ? Constants.SHUTTLE_SHORT_OFFSET_BLUE
                                                                : Constants.SHUTTLE_SHORT_OFFSET_RED))
                                                .withVelocityX(robotContainer.getXSpeed(true))
                                                .withVelocityY(robotContainer.getYSpeed(true)))
                                .alongWith(runEntryCommand(
                                        () -> ShooterTable.calcShortShuttleTableEntry(Meters
                                                .of(odometryStrategy.getDrivetrainGoalTranslation(true).getNorm())),
                                        () -> ShotSpeeds.FAST))
                                .repeatedly());
    }

    /**
     * Generate a command to run the shooter, convey, & stop everything thereafter.
     * 
     * @param entry
     *            The entry to run.
     */
    public Command shootCommand(Supplier<ShooterTableEntry> entry) {
        return driverCamera.setShooterCameraCommand().andThen(runEntryCommand(entry, () -> ShotSpeeds.FAST))
                .andThen(Commands.waitUntil(shooterAndPivotReady())).andThen(conveyCommand().withTimeout(0.75))
                .finallyDo(() -> {
                    shooter.stop();
                    pivot.runToPosition(Pivot.HOLD_POSITION);
                    setCameraWithWait();
                });
    }

    /**
     * Generate a command to shoot based on a target distance.
     * 
     * @param distance
     *            The target distance, in feet.
     */
    public Command shootCommand(double distance) {
        return shootCommand(() -> ShooterTable.calcShooterTableEntry(Feet.of(distance)));
    }

    /**
     * Generate a command to use the specified entry to run a magic shot.
     * 
     * @param entry
     *            The entry to run.
     * @param strategy
     *            The strategy to use for rotation.
     * @param lock
     *            Whether or not to align the drivetrain.
     */
    public Command magicShootCommand(Supplier<ShooterTableEntry> entry, Supplier<ShootingStrategy> strategy,
            boolean lock, Rotation2d offset) {
        return Commands.runOnce(() -> entryToRun = entry.get())
                .andThen(
                        fixNoteCommand().unless(noteSensing.conveyorSeesNoteSupplier()))
                .andThen(driverCamera.setShooterCameraCommand())
                .andThen(runEntryCommand(() -> entryToRun, () -> ShotSpeeds.FAST)
                        .alongWith(drivetrain.speakerAlign(strategy, offset).withTimeout(0.5)
                                .unless(() -> Math.abs(strategy.get().getTargetOffset().getDegrees()) < 0.25))
                        .onlyIf(() -> lock))
                .andThen(shootCommand(() -> entryToRun));
    }

    /**
     * Generate a command to use the specified entry to run a magic shot.
     * 
     * @param entry
     *            The entry to run.
     * @param strategy
     *            The strategy to use for rotation.
     * @param lock
     *            Whether or not to align the drivetrain.
     */
    public Command magicShootCommand(Supplier<ShooterTableEntry> entry, Supplier<ShootingStrategy> strategy,
            boolean lock) {
        return magicShootCommand(entry, strategy, lock, ShootingStrategy.OFFSET);
    }

    /**
     * Generate a command to use the specified distance to run a magic shot.
     * 
     * @param distance
     *            The shot to run.
     * @param strategy
     *            The strategy to use for rotation.
     * @param lock
     *            Whether or not to align the drivetrain.
     */
    public Command magicShootCommand(double distance, Supplier<ShootingStrategy> strategy, boolean lock) {
        return magicShootCommand(() -> ShooterTable.calcShooterTableEntry(Feet.of(distance)), strategy, lock);
    }

    /**
     * Generate a command to use the specified strategy to run a magic shot.
     * 
     * @param strategy
     *            The {@link ShootingStrategy} to use.
     * @param lock
     *            Whether or not to align the drivetrain.
     */
    public Command magicShootCommand(Supplier<ShootingStrategy> strategy, boolean lock, Rotation2d offset) {
        return magicShootCommand(() -> strategy.get().getTargetEntry(), strategy, lock, offset);
    }

    /**
     * Generate a command to use the specified strategy to run a magic shot.
     * 
     * @param strategy
     *            The {@link ShootingStrategy} to use.
     * @param lock
     *            Whether or not to align the drivetrain.
     */
    public Command magicShootCommand(Supplier<ShootingStrategy> strategy, boolean lock) {
        return magicShootCommand(() -> strategy.get().getTargetEntry(), strategy, lock, ShootingStrategy.OFFSET);
    }

    /**
     * Generate a command to use the specified strategy to run a magic, aligning
     * shot.
     * 
     * @param strategy
     *            The {@link ShootingStrategy} to use.
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
        return shooter.runEntryCommand(entry, speed).alongWith(pivot.runToPositionCommand(() -> entry.get().Angle))
                .alongWith(Commands.runOnce(() -> lastShot = entry.get().Distance.in(Feet))
                        .onlyIf(() -> entry.get().Distance != null));
    }

    /** Shooter & Pivot Both Ready */
    public BooleanSupplier shooterAndPivotReady() {
        return () -> shooter.isReady() && pivot.inPosition();
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
                .alongWith(pivot.runToPositionCommand(pivotAngle))
                .alongWith(Commands.either(drivetrain.mirrorablePathFindCommand(target, scale, endVelocity),
                        drivetrain.mirrorablePathFindCommand(redPose, scale, endVelocity), BeakUtils::allianceIsBlue));
    }

    /** Pathfinding Auton Shot */
    public Command pathfindingShotCommand(double targetDistance, Pose2d target, double scale, double endVelocity) {
        return drivetrain.mirrorablePathFindCommand(target, scale, endVelocity).deadlineWith(smartInfeedCommand()
                .withTimeout(0.6).andThen(coolNoteFixCommand(0.2)).andThen(shooter.runShotCommand(ShotSpeeds.FAST)))
                .andThen(shootCommand(targetDistance));
    }

    // ==============================
    // SECTION: Climbing
    // ==============================

    /** Only run climb command if pivot good */
    public Command safeClimbCommand(Command command) {
        return command.onlyIf(() -> pivot.getPosition() > PIVOT_UP_THRESHOLD);
    }

    public Command safePivotCommand(Command command) {
        return command.onlyIf(climber::reverseLimitOn);
    }

    /** Iterate Climb Sequence */
    private void incrementSequence() {
        ClimbSequence seq = ClimbSequence.Default;

        switch (currentSequence) {
            case Default :
                seq = enableTrap || enableClimber ? ClimbSequence.Prep : ClimbSequence.Default;
                break;
            case Prep :
                seq = ClimbSequence.Fan;
                break;
            case Fan :
                seq = enableTrap ? ClimbSequence.Shoot : ClimbSequence.End;
                break;
            case Shoot :
                seq = ClimbSequence.End;
                break;
            case End :
                seq = enableClimber ? ClimbSequence.Climb : ClimbSequence.Default;
                break;
            default :
                seq = ClimbSequence.Default;
                break;
        }

        currentSequence = seq;
    }

    /** Climb Preparation */
    public Command prepClimbCommand() {
        // Pivot => Trap Position
        // Shooter => Trap Mode
        return pivot.runToTrapCommand().alongWith(shooter.setSlotCommand(Slots.TRAP));
    }

    /** Prime the fan & shooter */
    public Command fanReadyCommand() {
        // Fan Pivot Up
        // Shooting Routine
        // Climber Up
        return fanPivot.runToTrapCommand()
                .alongWith(fan.runMotorCommand(VBusConstants.FAN_VBUS)
                        .andThen(BeakCommands.repeatCommand(fixNoteCommand(), 2))
                        .andThen(shooter.runShotCommand(ShotSpeeds.TRAP)).onlyIf(() -> enableTrap))
                .alongWith(climber.runToPositionCommand(VBusConstants.CLIMBER_VBUS, ClimberPositions.READY, false)
                        .onlyIf(() -> enableClimber && pivot.getPosition() > PIVOT_UP_THRESHOLD));
    }

    public Command dumbCancelClimbCommand() {

        return Commands.runOnce(() -> {
            currentSequence = ClimbSequence.Default;
            cancelClimbRequestShooterDown = true;
        }).andThen(fanPivot.runToTrapCommand(), Commands.waitSeconds(0.2),
                safeClimbCommand(climber.zeroCommand().until(climber.reverseLimitOnSupplier()).alongWith(
                        Commands.waitSeconds(1).andThen(fanPivot.runToHomeCommand()), fan.stopCommand(),
                        shooter.stopCommand())));
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
        return fanPivot.runToHomeCommand().alongWith(fan.stopCommand())
                .alongWith(shooter.stopCommand().andThen(shooter.setSlotCommand(Slots.FAST)))
                .alongWith(safePivotCommand(pivot.runToHomeCommand()).unless(() -> enableClimber));
    }

    /** Climb!!!!! */
    public Command climbCommand() {
        // Climber Down
        return safeClimbCommand(climber.runToPositionCommand(VBusConstants.CLIMBER_VBUS, ClimberPositions.CLIMB, true));
    }

    /** Update Sequence */
    public Command sequenceCommand() {
        return Commands.runOnce(this::incrementSequence)
                .andThen(Commands.select(sequenceCommandMap, () -> currentSequence));
    }

    public Command toggleTrapCommand() {
        return Commands.either(pivot.runToTrapCommand(),
                safeClimbCommand(climber.zeroCommand().until(climber::reverseLimitOn))
                        .andThen(pivot.runToHomeCommand()),
                pivot::getIsAtHome);
    }

    // ==============================
    // SECTION: Vision
    // ==============================

    /** Set infeed camera asynchronously */
    public void setCameraWithWait() {
        Commands.waitSeconds(VisionConstants.CAMERA_SWITCH_TIMEOUT).andThen(driverCamera.setInfeedCameraCommand())
                .schedule();
    }

    public void updateMTRot() {
        chassisLimelight.setRobotRotationMT2(drivetrain.getRotation().getDegrees());
        infeedLimelight3G.setRobotRotationMT2(drivetrain.getRotation().getDegrees());

    }

    public void updateDrivePoseMT2() {
        updateMTRot();

        if (!useMT2)
            return;

        // Apply Chassis Limelight
        var visionResult = chassisLimelight.getBotposeEstimateMT2();
        var visionStdDevs = chassisLimelight.getSTDevsXY(drivetrain);
        if (visionStdDevs.isPresent()) {
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
        chassisLimelight.setPipeline(VisionConstants.MEGATAG_PIPELINE);
        selectedStrategy = odometryStrategy;
    }

    public void setChassisPipeline() {
        selectedStrategy = chassisLimelight2dStrategy;
        chassisLimelight.setPipeline(VisionConstants.TY_PIPELINE);
    }

    public void setTeleopMT2RotationThresholds() {
        chassisLimelight.setTeleopMT2Threshold();
        infeedLimelight3G.setTeleopMT2Threshold();
    }

    public void setAutonMT2RotationThresholds() {
        chassisLimelight.setAutonMT2Threshold();
        infeedLimelight3G.setAutonMT2Threshold();
    }

    // ==============================
    // SECTION: Misc.
    // ==============================

    /** Stop Shooter */
    public void stopShooter() {
        shooter.stop();
    }

    /** Push limelight data to the CANdle */
    public Command encodeLimelights() {
        return candle.encodeLimelights(chassisLimelight, chassisLimelight, infeedLimelight3G);
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
