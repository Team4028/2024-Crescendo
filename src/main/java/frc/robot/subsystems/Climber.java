// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.DashboardStore;

public class Climber extends SubsystemBase {
    private final TalonFX motor;

    private final DataLog log;
    private final DoubleLogEntry vbusLog, currentLog, positionLog, velocityLog;

    private static final double ZERO_CURRENT_THRESHOLD = 7;
    private static final double ZERO_VBUS = -0.05;
    private static final double ZERO_TIMER_OFFSET = 0.1;

    private final Timer m_zeroTimer;

    private static final int CAN_ID = 15;

    /* Configs */
    private final CurrentLimitsConfigs currentConfigs = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(100.)
            .withSupplyCurrentLimit(80.);

    private final HardwareLimitSwitchConfigs limitSwitchConfigs = new HardwareLimitSwitchConfigs()
    
            /* Forward */
            .withForwardLimitAutosetPositionEnable(true)
            .withForwardLimitAutosetPositionValue(140.) // tune

            .withForwardLimitEnable(true)
            .withForwardLimitRemoteSensorID(9)

            .withForwardLimitSource(ForwardLimitSourceValue.LimitSwitchPin)
            .withForwardLimitType(ForwardLimitTypeValue.NormallyClosed)

            /* Reverse */
            .withReverseLimitAutosetPositionEnable(true)
            .withReverseLimitAutosetPositionValue(0.) // tune

            .withReverseLimitEnable(true)
            .withReverseLimitRemoteSensorID(8)

            .withReverseLimitSource(ReverseLimitSourceValue.LimitSwitchPin)
            .withReverseLimitType(ReverseLimitTypeValue.NormallyClosed);

    /* Requests */
    private final DutyCycleOut m_focRequest = new DutyCycleOut(0.)
            .withEnableFOC(true);

    private double m_target = 0.;
    private double m_targetSign = 1;

    public enum ClimberPositions {
        CLIMB(8.),
        READY(135.);

        public double Position;

        private ClimberPositions(double position) {
            this.Position = position;
        }
    }

    public Climber() {
        /* Setup */
        motor = new TalonFX(CAN_ID);

        motor.setNeutralMode(NeutralModeValue.Brake);
        motor.setInverted(false);

        /* ======= */
        /* CONFIGS */
        /* ======= */
        motor.getConfigurator().apply(currentConfigs);
        // motor.getConfigurator().apply(limitSwitchConfigs);

        /* CAN Bus */
        motor.getVelocity().setUpdateFrequency(20.);
        motor.getPosition().setUpdateFrequency(20.);
        motor.getStatorCurrent().setUpdateFrequency(20.);
        motor.getDutyCycle().setUpdateFrequency(20.);
        motor.optimizeBusUtilization();

        /* Logs */
        log = DataLogManager.getLog();
        vbusLog = new DoubleLogEntry(log, "/Climber/Vbus");
        currentLog = new DoubleLogEntry(log, "/Climber/Current");
        positionLog = new DoubleLogEntry(log, "/Climber/Position");
        velocityLog = new DoubleLogEntry(log, "/Climber/Velocity");

        /* Dashboard */
        DashboardStore.add("Climber Position", () -> motor.getPosition().getValueAsDouble());
        DashboardStore.add("Climber Current", () -> motor.getStatorCurrent().getValueAsDouble());
        DashboardStore.add("Climber Velocity", () -> motor.getVelocity().getValueAsDouble());

        /* Timer */
        m_zeroTimer = new Timer();
    }

    public void runMotor(double vBus, boolean useFoc) {
        if (useFoc) {
            motor.setControl(m_focRequest.withOutput(vBus));
        } else {
            motor.set(vBus);
        }
    }

    public Command runMotorCommand(double vBus, boolean useFoc) {
        return runOnce(() -> runMotor(vBus, useFoc));
    }

    public void stop() {
        runMotor(0.0, false);
    }

    public Command stopCommand() {
        return runMotorCommand(0.0, false);
    }

    public void zeroEncoder() {
        motor.setPosition(0.0);
    }

    public Command zeroEncoderCommand() {
        return runOnce(this::zeroEncoder);
    }

    /* Position Stuff */
    public double getError() {
        return m_target - motor.getPosition().getValueAsDouble();
    }

    public Command runToPositionCommand(double output, double position) {
        return runOnce(() -> {
            m_target = position;
            m_targetSign = Math.signum(getError());
        })
                .andThen(runOnce(() -> runMotor(m_targetSign * Math.abs(output), true)))
                // make sure overdrives aren't (generally) possible
                .andThen(Commands.waitUntil(() -> m_targetSign == -1 ? //
                        getError() > -2.0 : getError() < 2.0))
                .andThen(stopCommand());
    }

    public Command runToPositionCommand(double output, ClimberPositions position) {
        return runToPositionCommand(output, position.Position);
    }

    public void logValues() {
        vbusLog.append(motor.get());
        currentLog.append(motor.getStatorCurrent().getValueAsDouble());
        positionLog.append(motor.getPosition().getValueAsDouble());
        velocityLog.append(motor.getVelocity().getValueAsDouble());
    }

    public Command zeroCommand() {
        return runOnce(m_zeroTimer::restart)
                .andThen(runMotorCommand(ZERO_VBUS, false))
                .andThen(Commands.waitUntil(() -> motor.getStatorCurrent().getValueAsDouble() >= ZERO_CURRENT_THRESHOLD
                        && m_zeroTimer.get() > ZERO_TIMER_OFFSET))
                .andThen(zeroEncoderCommand())
                .andThen(stopCommand())
                .andThen(runOnce(m_zeroTimer::stop));
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
