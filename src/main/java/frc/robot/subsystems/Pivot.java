package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class Pivot extends SubsystemBase {
    private final TalonFX motor;

    private final DataLog log;
    private final DoubleLogEntry pivotCurrent, pivotVelocity, pivotVoltage, pivotVolt2, pivotPosition;

    private final Timer zeroTimer;
    private final double ZERO_TIMER_THRESHOLD = 0.14; // 7 scans
    private final double ZERO_VELOCITY_THRESHOLD = 0.2;

    // private final MotionMagicTorqueCurrentFOC motionMagicRequest = new MotionMagicTorqueCurrentFOC(0.);
    private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0.);

    // private final Slot0Configs pidConfigs = new Slot0Configs().withKP(320.);
    private final Slot0Configs pidConfigs = new Slot0Configs()
        .withKP(14.0)
        .withKS(0.12);
    
    private static final int SUPPLY_CURRENT_LIMIT = 80;
    private static final int STATOR_CURRENT_LIMIT = 100;

    private static final int PIVOT_CAN_ID = 13;

    private static final double CRUISE_VELOCITY = 24.;
    private static final double ACCELERATION = 36.;
    private static final double JERK = 360.;

    public static final double MIN_VAL = 0.5;
    
    public static final double MAX_VAL = 12.5;
    public static final double TRAP_POSITION = 7.3;

    private double targetPosition;

    public Pivot() {
        /* ======== */
        /* MOTAHHHH */
        /* ======== */
        motor = new TalonFX(PIVOT_CAN_ID);

        motor.setInverted(true);
        motor.setNeutralMode(NeutralModeValue.Brake);

        /* === */
        /* PID */
        /* === */

        motor.getConfigurator().apply(pidConfigs);

        // current limits
        motor.getConfigurator().apply(
                new CurrentLimitsConfigs().withSupplyCurrentLimit(SUPPLY_CURRENT_LIMIT)
                        .withStatorCurrentLimit(STATOR_CURRENT_LIMIT));
    

        /* ============ */
        /* MOTION MAGIC */
        /* ============ */
        MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
        motionMagicConfigs.MotionMagicCruiseVelocity = CRUISE_VELOCITY;
        motionMagicConfigs.MotionMagicAcceleration = ACCELERATION;
        motionMagicConfigs.MotionMagicJerk = JERK;

        motor.getConfigurator().apply(motionMagicConfigs);

        targetPosition = MIN_VAL;

        /* ===== */
        /* TIMER */
        /* ===== */
        zeroTimer = new Timer();

        // Logging *Wo-HO!!
        log = DataLogManager.getLog();
        pivotCurrent = new DoubleLogEntry(log, "/Pivot/Current");
        pivotVoltage = new DoubleLogEntry(log, "/Pivot/Voltage");
        pivotVelocity = new DoubleLogEntry(log, "/Pivot/Velocity");
        pivotVolt2 = new DoubleLogEntry(log, "/Pivot/Voltage2");
        pivotPosition = new DoubleLogEntry(log, "/Pivot/Position");
    }
    // ==================================
    // PIVOT COMMANDS
    // ==================================

    public double getPosition() {
        return motor.getPosition().getValueAsDouble();
    }

    public void runMotor(double vBus) {
        motor.set(vBus);
    }

    public Command runMotorCommand(double vBus) {
        return runOnce(() -> runMotor(vBus));
    }

    public void runToPosition(double position) {
        targetPosition = position;
        motor.setControl(
                motionMagicRequest.withPosition(MathUtil.clamp(position, MIN_VAL, MAX_VAL)));
    }

    public Command runToPositionCommand(double position) {
        return runOnce(() -> runToPosition(position));
    }

    public void holdPosition() {
        runToPosition(motor.getPosition().getValueAsDouble());
    }

    public Command holdPositionCommand() {
        return runOnce(this::holdPosition).andThen(Commands.idle());
    }

    public Command zeroCommand() {
        return runOnce(() -> {
            zeroTimer.restart();
        })
                .andThen(runMotorCommand(-0.025).repeatedly()
                        .until(() -> zeroTimer.get() >= ZERO_TIMER_THRESHOLD
                                && Math.abs(motor.getVelocity().getValueAsDouble()) < ZERO_VELOCITY_THRESHOLD))
                .andThen(runMotorCommand(0.).alongWith(Commands.runOnce(() -> zeroTimer.stop())))
                .andThen(runOnce(() -> motor.setPosition(0.)))
                .andThen(new WaitCommand(0.25).andThen(runToPositionCommand(MIN_VAL)));

    }

    // public boolean inPosition() {
    //     return Math.abs(motor.getPosition() - targetPosition) < 1.0;
    // }

    public BooleanSupplier inPosition() {
        return () -> (Math.abs(motor.getPosition().getValueAsDouble() - targetPosition) < 1.0);
    }

    public void logValues() {
        pivotCurrent.append(motor.getStatorCurrent().getValueAsDouble());
        pivotVoltage.append(motor.getMotorVoltage().getValueAsDouble());
        pivotVelocity.append(motor.getVelocity().getValueAsDouble());
        pivotVolt2.append(motor.getSupplyVoltage().getValueAsDouble());
        pivotPosition.append(motor.getPosition().getValueAsDouble());
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Pivot pos", motor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Pivot Current", motor.getStatorCurrent().getValueAsDouble());
    }
}
