package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
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
    private final DoubleLogEntry currentLog, velocityLog, voltageLog, positionLog;

    private final Timer zeroTimer;
    private final double ZERO_TIMER_THRESHOLD = 0.14; // 7 scans
    private final double ZERO_VELOCITY_THRESHOLD = 0.2;

    private static final int CAN_ID = 13;
    public final static double CLIMB_POSITION = 12.5;
    public final static double HOLD_POSITION = 1.;

    private final PositionDutyCycle positionRequest = new PositionDutyCycle(0.);

    /* Configs */
    private final Slot0Configs pidConfigs = new Slot0Configs()
            .withKP(14.0)
            .withKS(0.12);

    private final MotorOutputConfigs outputConfigs = new MotorOutputConfigs()
            // check pls
            .withInverted(InvertedValue.Clockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake)
            .withPeakForwardDutyCycle(0.25)
            .withPeakReverseDutyCycle(-0.2);

    private final CurrentLimitsConfigs currentConfigs = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(100.)
            .withSupplyCurrentLimit(80.);

    private final ClosedLoopRampsConfigs rampConfigs = new ClosedLoopRampsConfigs()
            .withDutyCycleClosedLoopRampPeriod(1.0);

    private final SoftwareLimitSwitchConfigs limitConfigs = new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitThreshold(12.5);

    private double targetPosition;

    public Pivot() {
        /* ======== */
        /* MOTAHHHH */
        /* ======== */
        motor = new TalonFX(CAN_ID);

        /* ======= */
        /* CONFIGS */
        /* ======= */

        motor.getConfigurator().apply(pidConfigs);
        motor.getConfigurator().apply(currentConfigs);
        motor.getConfigurator().apply(outputConfigs);
        motor.getConfigurator().apply(rampConfigs);
        motor.getConfigurator().apply(limitConfigs);

        targetPosition = 1.;

        /* ===== */
        /* TIMER */
        /* ===== */
        zeroTimer = new Timer();

        // Logging *Wo-HO!!
        log = DataLogManager.getLog();
        currentLog = new DoubleLogEntry(log, "/Pivot/Current");
        voltageLog = new DoubleLogEntry(log, "/Pivot/Voltage");
        velocityLog = new DoubleLogEntry(log, "/Pivot/Velocity");
        positionLog = new DoubleLogEntry(log, "/Pivot/Position");
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
        targetPosition = position;
        motor.setControl(positionRequest.withPosition(position));
    }

    public Command runToPositionCommand(double position) {
        return runOnce(() -> runToPosition(position));
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
                .andThen(new WaitCommand(0.25).andThen(runToPositionCommand(HOLD_POSITION)));

    }

    // public boolean inPosition() {
    // return Math.abs(motor.getPosition() - targetPosition) < 1.0;
    // }

    public BooleanSupplier inPositionSupplier() {
        return () -> (Math.abs(motor.getPosition().getValueAsDouble() - targetPosition) < 1.0);
    }

    public void logValues() {
        currentLog.append(motor.getStatorCurrent().getValueAsDouble());
        voltageLog.append(motor.getMotorVoltage().getValueAsDouble());
        velocityLog.append(motor.getVelocity().getValueAsDouble());
        positionLog.append(motor.getPosition().getValueAsDouble());
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Pivot Position", motor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Pivot Current", motor.getStatorCurrent().getValueAsDouble());
    }
}
