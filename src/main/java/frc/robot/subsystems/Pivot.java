package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
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
    private final DoubleLogEntry pivotCurrent, pivotVelocity, pivotVoltage;

    private final Timer zeroTimer;
    private final double ZERO_TIMER_THRESHOLD = 0.14; // 7 scans
    private final double ZERO_VELOCITY_THRESHOLD = 0.2;

    private final MotionMagicTorqueCurrentFOC motionMagicRequest = new MotionMagicTorqueCurrentFOC(
            0.);

    private final Slot0Configs pidConfigs = new Slot0Configs().withKP(0.01);

    private static final int PIVOT_CAN_ID = 13;

    private static final double CRUISE_VELOCITY = 2.;
    private static final double ACCELERATION = 4.;
    private static final double JERK = 40.;

    public static final double MIN_VAL = 1.;
    public static final double MAX_VAL = 17.5;

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

        /* ============ */
        /* MOTION MAGIC */
        /* ============ */
        MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
        motionMagicConfigs.MotionMagicCruiseVelocity = CRUISE_VELOCITY;
        motionMagicConfigs.MotionMagicAcceleration = ACCELERATION;
        motionMagicConfigs.MotionMagicJerk = JERK;

        motor.getConfigurator().apply(motionMagicConfigs);

        /* ===== */
        /* TIMER */
        /* ===== */
        zeroTimer = new Timer();

        // Logging *Wo-HO!!
        log = DataLogManager.getLog();
        pivotCurrent = new DoubleLogEntry(log, "/Pivot/Current");
        pivotVoltage = new DoubleLogEntry(log, "/Pivot/Voltage");
        pivotVelocity = new DoubleLogEntry(log, "/Pivot/Velocity");
    }
    // ==================================
    // PIVOT COMMANDS
    // ==================================

    public void runMotor(double vBus) {
        motor.set(vBus);
    }

    public Command runMotorCommand(double vBus) {
        return runOnce(() -> runMotor(vBus));
    }

    public void runToPosition(double position) {
        motor.setControl(
                motionMagicRequest.withPosition(MathUtil.clamp(position, MIN_VAL, MAX_VAL)));
    }

    public Command runToPositionCommand(double position) {
        return runOnce(() -> runToPosition(position));
    }

    public Command zeroCommand() {
        return runOnce(() -> {
            zeroTimer.restart();
        })
                .andThen(runMotorCommand(-0.1).repeatedly()
                        .until(() -> zeroTimer.get() >= ZERO_TIMER_THRESHOLD
                                && Math.abs(motor.getVelocity().getValueAsDouble()) < ZERO_VELOCITY_THRESHOLD))
                .andThen(runMotorCommand(0.).alongWith(Commands.runOnce(() -> zeroTimer.stop())))
                .andThen(runOnce(() -> motor.setPosition(0.)))
                .andThen(new WaitCommand(0.25).andThen(runToPositionCommand(MIN_VAL)));

    }

    public void logValues() {
        pivotCurrent.append(motor.getStatorCurrent().getValueAsDouble());
        pivotVoltage.append(motor.getDutyCycle().getValueAsDouble());
        pivotVelocity.append(motor.getVelocity().getValueAsDouble());
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Pivot pos", motor.getPosition().getValueAsDouble());
    }
}
