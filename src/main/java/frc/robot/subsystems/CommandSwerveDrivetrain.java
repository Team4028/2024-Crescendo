package frc.robot.subsystems;

import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.units.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog.State;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.SwerveVoltRequest;
import frc.robot.generated.TunerConstants;

// FIXME: extract out magic numbers & fixup logging
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
    private DataLog log;

    private static final PIDConstants AUTON_LINEAR_PID = new PIDConstants(10, 0, 0);
    private static final PIDConstants AUTON_ANGULAR_PID = new PIDConstants(5, 0, 0);

    private static final class PathFindPlannerConstants {
        private static final double kMaxSpeed = TunerConstants.kSpeedAt12VoltsMps;
        private static final double kMaxAccel = TunerConstants.kSpeedAt12VoltsMps;
        private static final double kMaxAngSpeed = 1.5 * Math.PI;
        private static final double kMaxAngAccel = 1.5 * Math.PI;
    }

    public DoubleLogEntry currentFL, currentFR, currentBL, currentBR, velocityFL, velocityFR, velocityBL, velocityBR, vBusFL, vBusFR, vBusBL,
            vBusBR;

    private final SwerveRequest.ApplyChassisSpeeds autoRequest = new SwerveRequest.ApplyChassisSpeeds();

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

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency,
            SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    @Override
    public void seedFieldRelative(Pose2d location) {
        // getPigeon2().setYaw(0);
        super.seedFieldRelative(location);
    }

    @Override
    public void seedFieldRelative() {
        // getPigeon2().setYaw(0);
        try {
            m_stateLock.writeLock().lock();
            m_fieldRelativeOffset = getState().Pose.getRotation();
        } finally {
            m_stateLock.writeLock().unlock();
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

    public void logValues() {
        // for (var mod : Modules) {
        //     TalonFX motor = mod.getDriveMotor();
        //     switch (motor.getDeviceID()) {
        //         case TunerConstants.kFrontLeftDriveMotorId:
        //             currentFL.append(motor.getSupplyCurrent().getValueAsDouble());
        //             velocityFL.append(motor.getVelocity().getValueAsDouble());
        //             vBusFL.append(motor.getMotorVoltage().getValueAsDouble() / RobotController.getBatteryVoltage());
        //             break;
        //         case TunerConstants.kFrontRightDriveMotorId:
        //             currentFR.append(motor.getSupplyCurrent().getValueAsDouble());
        //             velocityFR.append(motor.getVelocity().getValueAsDouble());
        //             vBusFR.append(motor.getMotorVoltage().getValueAsDouble() / RobotController.getBatteryVoltage());
        //             break;
        //         case TunerConstants.kBackLeftDriveMotorId:
        //             currentBL.append(motor.getSupplyCurrent().getValueAsDouble());
        //             velocityBL.append(motor.getVelocity().getValueAsDouble());
        //             vBusBL.append(motor.getMotorVoltage().getValueAsDouble() / RobotController.getBatteryVoltage());
        //             break;
        //         case TunerConstants.kBackRightDriveMotorId:
        //             currentBR.append(motor.getSupplyCurrent().getValueAsDouble());
        //             velocityBR.append(motor.getVelocity().getValueAsDouble());
        //             vBusBR.append(motor.getMotorVoltage().getValueAsDouble() / RobotController.getBatteryVoltage());
        //             break;
        //     }
        // }
    }

    private void initSwerve() {
        log = DataLogManager.getLog();

        configLoggersProcedural();
        configPPLib();
        configModules();
    }

    private void configModules() {
        for (var mod : Modules) {
            mod.getDriveMotor().setNeutralMode(NeutralModeValue.Brake);
            mod.getSteerMotor().setNeutralMode(NeutralModeValue.Brake);
        }
    }

    // private void configLoggersMan() {
    // currentFL = new DoubleLogEntry(log, "/FL/current");
    // currentFR = new DoubleLogEntry(log, "/FR/current");
    // currentBL = new DoubleLogEntry(log, "/BL/current");
    // currentBR = new DoubleLogEntry(log, "/BR/current");
    // velFL = new DoubleLogEntry(log, "/FL/velocity");
    // velFR = new DoubleLogEntry(log, "/FR/velocity");
    // velBL = new DoubleLogEntry(log, "/BL/velocity");
    // velBR = new DoubleLogEntry(log, "/BR/velocity");
    // vbFL = new DoubleLogEntry(log, "/FL/vBus");
    // vbFR = new DoubleLogEntry(log, "/FR/vBus");
    // vbBL = new DoubleLogEntry(log, "/BL/vBus");
    // vbBR = new DoubleLogEntry(log, "/BR/vBus");
    // }

    private void configLoggersProcedural() {
        try {
            for (var field : getClass().getFields()) {
                if (field.getName().contains("currentF") || field.getName().contains("currentB"))
                    field.set(this, new DoubleLogEntry(log, "/" + field.getName().replace("current", "") + "/current"));
                else if (field.getName().contains("velocityF") || field.getName().contains("velocityB"))
                    field.set(this, new DoubleLogEntry(log, "/" + field.getName().replace("velocity", "") + "/velocity"));
                else if (field.getName().contains("vBusF") || field.getName().contains("vBusB"))
                    field.set(this, new DoubleLogEntry(log, "/" + field.getName().replace("vBus", "") + "/vBus"));
            }
        } catch (IllegalAccessException e) {
            e.printStackTrace();
        }
    }

    private void configPPLib() {
        double driveBaseRadius = 0;
        for (var modLocation : m_moduleLocations)
            driveBaseRadius = Math.max(driveBaseRadius, modLocation.getNorm());

        AutoBuilder.configureHolonomic(
                () -> getState().Pose,
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
}
