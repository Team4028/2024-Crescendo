package frc.robot.subsystems;

import java.lang.reflect.Field;
import java.util.HashMap;
import java.util.Map;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Function;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;

import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.units.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog.State;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.SwerveVoltRequest;
import frc.robot.generated.TunerConstants;
import frc.robot.utils.LogStore;
import frc.robot.utils.ShootingStrategy;
import frc.robot.utils.SignalStore;
import frc.robot.utils.VisionSystem;

// TODO: general clean up

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements
 * subsystem
 * so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    private static final PIDConstants AUTON_LINEAR_PID = new PIDConstants(10, 0, 0);
    private static final PIDConstants AUTON_ANGULAR_PID = new PIDConstants(5, 0, 0);

    private static final class PathFindPlannerConstants {
        private static final double kMaxSpeed = TunerConstants.kSpeedAt12VoltsMps;
        private static final double kMaxAccel = TunerConstants.kSpeedAt12VoltsMps;
        private static final double kMaxAngSpeed = 1.5 * Math.PI;
        private static final double kMaxAngAccel = 1.5 * Math.PI;
    }

    private BaseStatusSignal currentFL, currentFR, currentBL, currentBR,
            velocityFL, velocityFR, velocityBL, velocityBR,
            vbusFL, vbusFR, vbusBL, vbusBR;

    private final SwerveRequest.ApplyChassisSpeeds autoRequest = new SwerveRequest.ApplyChassisSpeeds();
    private final SwerveRequest.FieldCentric speakerLockDrive = new SwerveRequest.FieldCentric();
    private final SwerveRequest.RobotCentric staticAlignDrive = new SwerveRequest.RobotCentric();
    private final SwerveRequest.RobotCentric targetAcquireDrive = new SwerveRequest.RobotCentric();

    private Rotation2d alignmentTarget = new Rotation2d();

    private static final double STATIC_ALIGN_kP = 8.0;
    private static final double STATIC_ALIGN_VELOCITY = 6.0;
    private static final double STATIC_ALIGN_ACCELERATION = 12.0;

    private static final double LOCK_ALIGN_kP = 10.0;

    private static final double TARGET_ACQUIRE_THRESHOLD = 1.0;
    private static final double TARGET_ACQUIRE_kP = 6.0;
    private static final double TARGET_ACQUIRE_kD = 0.5;

    private static final ProfiledPIDController STATIC_ALIGN_CONTROLLER = new ProfiledPIDController(
            // The PID gains
            STATIC_ALIGN_kP,
            0.0,
            0.0,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(
                    STATIC_ALIGN_VELOCITY,
                    STATIC_ALIGN_ACCELERATION));

    private static final PIDController LOCK_ALIGN_CONTROLLER = new PIDController(
            LOCK_ALIGN_kP,
            0.0,
            0.0);

    private static final PIDController TARGET_ACQUIRE_CONTROLLER = new PIDController(
            TARGET_ACQUIRE_kP, 0.0, TARGET_ACQUIRE_kD);

    private final SwerveVoltRequest voltRequest = new SwerveVoltRequest();

    private final SysIdRoutine sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(null, null, null, this::logSysIdState),
            new SysIdRoutine.Mechanism(
                    (Measure<Voltage> volts) -> setControl(voltRequest.withVoltage(volts.in(Units.Volts))),
                    null,
                    this));

    private void logSysIdState(State state) {
        SignalLogger.writeString("test-mode", state.toString());
    }

    {
        initSwerve();
    }

    // public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants,
    // double OdometryUpdateFrequency,
    // SwerveModuleConstants... modules) {
    // super(driveTrainConstants, OdometryUpdateFrequency, modules);
    // if (Utils.isSimulation()) {
    // startSimThread();
    // }

    // ppTargetPose = new DoubleArrayLogEntry(log, "Pathplanner Target Pose");

    // configModules(modules);
    // }

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants drivetrainConstants, double OdometryUpdateFrequency,
            SwerveModuleConstants... modules) {
        super(drivetrainConstants, OdometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }

        STATIC_ALIGN_CONTROLLER.enableContinuousInput(-Math.PI, Math.PI);
        TARGET_ACQUIRE_CONTROLLER.enableContinuousInput(-Math.PI, Math.PI);
        LOCK_ALIGN_CONTROLLER.enableContinuousInput(-Math.PI, Math.PI);

        configModules(modules);
        configLogging();
    }

    // public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants,
    // SwerveModuleConstants... modules) {
    // super(driveTrainConstants, modules);
    // if (Utils.isSimulation()) {
    // startSimThread();
    // }

    // ppTargetPose = new DoubleArrayLogEntry(log, "Pathplanner Target Pose");

    // configModules(modules);
    // }

    private void configModules(SwerveModuleConstants... modules) {
        int i = 0;
        for (SwerveModuleConstants module : modules) {
            try {
                Thread.sleep(250);
            } catch (InterruptedException e) {
                System.err.println("bad" + e.getMessage());
            }

            TalonFXConfiguration talonConfigs = new TalonFXConfiguration();

            talonConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

            talonConfigs.Slot0 = module.DriveMotorGains;
            talonConfigs.TorqueCurrent.PeakForwardTorqueCurrent = module.SlipCurrent;
            talonConfigs.TorqueCurrent.PeakReverseTorqueCurrent = -module.SlipCurrent;
            talonConfigs.CurrentLimits.StatorCurrentLimit = module.SlipCurrent;
            talonConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
            talonConfigs.CurrentLimits.SupplyCurrentLimit = 60.;
            talonConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;

            talonConfigs.MotorOutput.Inverted = module.DriveMotorInverted ? InvertedValue.Clockwise_Positive
                    : InvertedValue.CounterClockwise_Positive;
            StatusCode response = Modules[i].getDriveMotor().getConfigurator().apply(talonConfigs);
            if (!response.isOK()) {
                System.out.println(
                        "TalonFX ID " + Modules[i].getDriveMotor().getDeviceID()
                                + " failed config with error "
                                + response.toString());
            }
            try {
                Thread.sleep(250);
            } catch (InterruptedException e) {
                System.err.println("bad" + e.getMessage());
            }

            talonConfigs = new TalonFXConfiguration();

            talonConfigs.Slot0 = module.SteerMotorGains;
            // Modify configuration to use remote CANcoder fused
            talonConfigs.Feedback.FeedbackRemoteSensorID = module.CANcoderId;
            switch (module.FeedbackSource) {
                case RemoteCANcoder:
                    talonConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
                    break;
                case FusedCANcoder:
                    talonConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
                    break;
                case SyncCANcoder:
                    talonConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.SyncCANcoder;
                    break;
            }
            talonConfigs.Feedback.RotorToSensorRatio = module.SteerMotorGearRatio;

            talonConfigs.MotionMagic.MotionMagicCruiseVelocity = 100.0 / module.SteerMotorGearRatio;
            talonConfigs.MotionMagic.MotionMagicAcceleration = talonConfigs.MotionMagic.MotionMagicCruiseVelocity
                    / 0.100;
            talonConfigs.MotionMagic.MotionMagicExpo_kV = 0.12 * module.SteerMotorGearRatio;
            talonConfigs.MotionMagic.MotionMagicExpo_kA = 0.1;

            talonConfigs.ClosedLoopGeneral.ContinuousWrap = true; // Enable continuous wrap for swerve
                                                                  // modules

            talonConfigs.MotorOutput.Inverted = module.SteerMotorInverted
                    ? InvertedValue.Clockwise_Positive
                    : InvertedValue.CounterClockwise_Positive;
            response = Modules[i].getSteerMotor().getConfigurator().apply(talonConfigs);
            if (!response.isOK()) {
                System.out.println(
                        "TalonFX ID " + Modules[i].getSteerMotor() + " failed config with error "
                                + response.toString());
            }

            i++;
        }
    }

    private void configLogging() {
        HashMap<String, Integer> idMap = new HashMap<>(Map.of(
                "FL", 0,
                "FR", 1,
                "BL", 2,
                "BR", 3));

        HashMap<String, Function<SwerveModule, BaseStatusSignal>> signalNameMap = new HashMap<>(Map.of(
                "current", (module) -> module.getDriveMotor().getStatorCurrent(),
                "velocity", (module) -> module.getDriveMotor().getVelocity(),
                "vbus", (module) -> module.getDriveMotor().getMotorVoltage()));

        for (Field field : this.getClass().getDeclaredFields()) {
            if (field.getName().matches("[cv][a-z]{3,8}[FB][RL]")) {
                System.out.println(field.getName());
                String name = field.getName();
                String[] split = name.split("(?=[BF])");
                String signal = split[0];

                // Capitalize first letter for logging
                String signalLog = signal.substring(0, 1).toUpperCase() + signal.substring(1);

                String location = split[1];

                SwerveModule module = getModule(idMap.get(location));

                BaseStatusSignal statusSignal = signalNameMap.get(signal).apply(module);

                try {
                    field.set(this, statusSignal);
                } catch (IllegalAccessException e) {
                    System.out.println(e.getMessage());
                }

                statusSignal.setUpdateFrequency(UpdateFrequency);

                LogStore.add("/Drive/" + location + "/" + signalLog,
                        () -> statusSignal.getValueAsDouble());
                SignalStore.add(statusSignal);
            }
        }
    }

    public Command pathFindCommand(Pose2d desiredPose, double scale, double endVel) {
        PathConstraints constraints = new PathConstraints(
                PathFindPlannerConstants.kMaxSpeed * scale,
                PathFindPlannerConstants.kMaxAccel * scale,
                PathFindPlannerConstants.kMaxAngSpeed,
                PathFindPlannerConstants.kMaxAngAccel);

        return AutoBuilder.pathfindToPose(
                desiredPose,
                constraints,
                endVel);
    }

    public Command mirrorablePathFindCommand(Pose2d desiredPose, double scale, double endVel) { // may end up being
                                                                                                // pathFindCommand
        PathConstraints constraints = new PathConstraints(
                PathFindPlannerConstants.kMaxSpeed * scale,
                PathFindPlannerConstants.kMaxAccel * scale,
                PathFindPlannerConstants.kMaxAngSpeed,
                PathFindPlannerConstants.kMaxAngAccel);

        return AutoBuilder.pathfindToPoseFlipped(desiredPose, constraints, endVel);
    }

    public Command pathFindThenFollowCommand(PathPlannerPath path, double scale) {
        PathConstraints constraints = new PathConstraints(
                TunerConstants.kSpeedAt12VoltsMps * scale,
                TunerConstants.kSpeedAt12VoltsMps * scale,
                1.5 * Math.PI,
                1.5 * Math.PI);

        return AutoBuilder.pathfindThenFollowPath(
                path,
                constraints);
    }

    public Command addMeasurementCommand(Supplier<Pose2d> measurement, Supplier<Double> timeStamp) {
        return runOnce(() -> {
            Pose2d pose = measurement.get();
            if (pose == new Pose2d())
                return;

            addVisionMeasurement(pose, timeStamp.get());
        });
    }

    public Command addMeasurementCommand(Supplier<Optional<EstimatedRobotPose>> pose) {
        return runOnce(() -> {
            Optional<EstimatedRobotPose> estPose = pose.get();
            if (estPose.isPresent()) {
                addVisionMeasurement(estPose.get().estimatedPose.toPose2d(), estPose.get().timestampSeconds);
            }
        });
    }

    public ChassisSpeeds getCurrentRobotChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(getState().ModuleStates);
    }

    public Command runQuasiTest(SysIdRoutine.Direction dir) {
        return sysIdRoutine.quasistatic(dir);
    }

    public Command runDynamTest(SysIdRoutine.Direction dir) {
        return sysIdRoutine.dynamic(dir);
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    public SwerveRequest getCurrentRequest() {
        return m_requestToApply;
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    private void initSwerve() {
        configPPLib();
        configModules();
    }

    private void configModules() {
        for (var mod : Modules) {
            mod.getDriveMotor().setNeutralMode(NeutralModeValue.Brake);
            mod.getSteerMotor().setNeutralMode(NeutralModeValue.Brake);
            mod.getDriveMotor().optimizeBusUtilization();
            mod.getSteerMotor().optimizeBusUtilization();
        }
    }

    private void configPPLib() {
        double driveBaseRadius = 0;
        for (var modLocation : m_moduleLocations)
            driveBaseRadius = Math.max(driveBaseRadius, modLocation.getNorm());

        AutoBuilder.configureHolonomic(
                () -> getPose(),
                this::seedFieldRelative,
                this::getCurrentRobotChassisSpeeds,
                (speeds) -> this.setControl(autoRequest.withSpeeds(speeds)),
                new HolonomicPathFollowerConfig(
                        AUTON_LINEAR_PID,
                        AUTON_ANGULAR_PID,
                        TunerConstants.kSpeedAt12VoltsMps,
                        driveBaseRadius,
                        new ReplanningConfig()),
                () -> DriverStation.getAlliance().get() == Alliance.Red,
                this);
    }

    public ChassisSpeeds getFieldRelativeChassisSpeeds() {
        ChassisSpeeds speeds = getCurrentRobotChassisSpeeds();
        Rotation2d rotation = getRotation();

        return new ChassisSpeeds(
                speeds.vxMetersPerSecond * rotation.getCos()
                        - speeds.vyMetersPerSecond * rotation.getSin(),
                speeds.vyMetersPerSecond * rotation.getCos()
                        + speeds.vxMetersPerSecond * rotation.getSin(),
                speeds.omegaRadiansPerSecond);
    }

    public Pose2d getPose() {
        return getState().Pose;
    }

    public Rotation2d getRotation() {
        return getPose().getRotation();
    }

    public Translation2d getTranslation() {
        return getPose().getTranslation();
    }

    public Command speakerLock(DoubleSupplier xSpeed, DoubleSupplier ySpeed,
            Supplier<ShootingStrategy> strategy) {
        return new PIDCommand(
                // The ProfiledPIDController used by the command
                LOCK_ALIGN_CONTROLLER,
                () -> getRotation().getRadians(),
                // This should return the goal (can also be a constant)
                () -> getRotation().plus(strategy.get().getTargetOffset()).getRadians(),
                // This uses the output
                (output) -> {
                    setControl(speakerLockDrive.withRotationalRate(output)
                            .withVelocityX(xSpeed.getAsDouble()).withVelocityY(ySpeed.getAsDouble()));
                }, this);
    }

    public Command speakerAlign(Supplier<ShootingStrategy> strategy) {
        return staticAlign(() -> alignmentTarget).beforeStarting(() -> {
            alignmentTarget = getRotation().plus(strategy.get().getTargetOffset());
        });
    }

    public Command speakerAlign(Supplier<ShootingStrategy> strategy, Rotation2d offset) {
        return staticAlign(() -> alignmentTarget).beforeStarting(() -> {
            alignmentTarget = getRotation().plus(strategy.get().getTargetOffset(offset, false));
        });
    }

    public Command staticAlign(Supplier<Rotation2d> target, Supplier<Rotation2d> measurement) {
        return new ProfiledPIDCommand(
                // The controller that the command will use
                STATIC_ALIGN_CONTROLLER,
                // This should return the measurement
                () -> measurement.get().getRadians(),
                // This should return the setpoint (can also be a constant)
                () -> target.get().getRadians(),
                // This uses the output
                (output, setpoint) -> {
                    setControl(staticAlignDrive.withRotationalRate(output + setpoint.velocity));
                }, this);
    }

    public Command staticAlign(Supplier<Rotation2d> target) {
        return staticAlign(target, this::getRotation);
    }

    public Command targetAcquire(DoubleSupplier forwardVelocity, VisionSystem vision) {
        return new PIDCommand(
                TARGET_ACQUIRE_CONTROLLER,
                () -> {
                    Optional<Rotation2d> target = vision.getTargetX();
                    return target.isPresent() ? target.get().getRadians() : 0.0;
                },
                () -> 0.0,
                (output) -> {
                    Optional<Rotation2d> tx = vision.getTargetX();

                    boolean useRotation = tx.isPresent() && Math.abs(tx.get().getDegrees()) < TARGET_ACQUIRE_THRESHOLD;
                    boolean useDrive = vision.getHasTarget();

                    setControl(targetAcquireDrive.withVelocityX(
                            vision.getHasTarget() && useDrive ? forwardVelocity.getAsDouble() : 0.0)
                            .withRotationalRate(useRotation ? output : 0.0));
                }, this);
    }
}
