// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalInput;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.utils.DashboardStore;

public class Climber extends SubsystemBase {
    private final TalonFX motor;

    private final StatusSignal<Double> position, current, velocity;

    private final DigitalInput forwardLimitSwitch, reverseLimitSwitch;

    private final DataLog log;
    private final DoubleLogEntry currentLog, positionLog, velocityLog;

    private static final double ZERO_VBUS = -0.05;

    private static final int CAN_ID = 15;

    private static final int FORWARD_LIMIT_SWITCH_PIN = 8;
    private static final int REVERSE_LIMIT_SWITCH_PIN = 9;

    private static final double ZERO_POSITION = 0.0;
    private static final double UP_POSITION = 129.0;

    /* Configs */
    private final CurrentLimitsConfigs currentConfigs = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(100.)
            .withSupplyCurrentLimit(80.);

    /* Requests */
    private final DutyCycleOut m_focRequest = new DutyCycleOut(0.)
            .withEnableFOC(true);

    private double m_target = 0.;
    private double m_targetSign = 1;

    public enum ClimberPositions {
        CLIMB(8.),
        READY(125.);

        public double Position;

        private ClimberPositions(double position) {
            this.Position = position;
        }
    }

    public Climber() {
        /* Setup */
        motor = new TalonFX(CAN_ID);

        position = motor.getPosition();
        velocity = motor.getVelocity();
        current = motor.getStatorCurrent();

        forwardLimitSwitch = new DigitalInput(FORWARD_LIMIT_SWITCH_PIN);
        reverseLimitSwitch = new DigitalInput(REVERSE_LIMIT_SWITCH_PIN);

        motor.setNeutralMode(NeutralModeValue.Brake);
        motor.setInverted(false);

        /* ======= */
        /* CONFIGS */
        /* ======= */
        motor.getConfigurator().apply(currentConfigs);

        /* CAN Bus */
        BaseStatusSignal.setUpdateFrequencyForAll(20.0, velocity, position, current);

        motor.optimizeBusUtilization();

        /* Logs */
        log = DataLogManager.getLog();

        currentLog = new DoubleLogEntry(log, "/Climber/Current");
        positionLog = new DoubleLogEntry(log, "/Climber/Position");
        velocityLog = new DoubleLogEntry(log, "/Climber/Velocity");

        /* Dashboard */
        DashboardStore.add("Climber Position", () -> position.getValueAsDouble());
        DashboardStore.add("Climber Current", () -> current.getValueAsDouble());
        DashboardStore.add("Climber Velocity", () -> velocity.getValueAsDouble());

        DashboardStore.add("Forward", forwardLimitSwitch::get);
        DashboardStore.add("Reverse", reverseLimitSwitch::get);
    }

    public void runMotor(double vBus, boolean useFoc) {
        if (useFoc) {
            motor.setControl(m_focRequest.withOutput(vBus));
        } else {
            motor.set(vBus);
        }
    }

    public Command runMotorCommand(double vBus, boolean useFoc) {
        return runOnce(() -> runMotor(vBus, useFoc)).unless(() -> {
            if (vBus > 0.)
                return forwardLimitSwitch.get();
            if (vBus < 0.)
                return reverseLimitSwitch.get();
            return false;
        });
    }

    public boolean forwardLimit() {
        return forwardLimitSwitch.get() && motor.getMotorVoltage().getValueAsDouble() > 0.2;
    }

    public boolean reverseLimit() {
        return reverseLimitSwitch.get() && motor.getMotorVoltage().getValueAsDouble() < -0.2;
    }

    public void stop() {
        runMotor(0.0, false);
    }

    public Command stopCommand() {
        return runMotorCommand(0.0, false);
    }

    public void setEncoderPosition(double position) {
        motor.setPosition(position);
    }

    public Command setEncoderPositionCommand(double position) {
        return runOnce(() -> setEncoderPosition(position));
    }

    public void zeroEncoder() {
        setEncoderPosition(0.0);
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

    public Command hitForwardLimitCommand() {
        return setEncoderPositionCommand(UP_POSITION)
        .andThen(runToPositionCommand(ZERO_VBUS, UP_POSITION - 4.));
    }

    public Command hitReverseLimitCommand() {
        return setEncoderPositionCommand(ZERO_POSITION)
        .andThen(runToPositionCommand(ZERO_VBUS, ZERO_POSITION + 7.));
    }

    public void logValues() {
        BaseStatusSignal.refreshAll(velocity, current, position);

        currentLog.append(current.getValueAsDouble());
        positionLog.append(position.getValueAsDouble());
        velocityLog.append(velocity.getValueAsDouble());
    }

    public Command zeroCommand() {
        return runMotorCommand(ZERO_VBUS, false);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
