// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.robot.utils.DashboardStore;
import frc.robot.utils.LogStore;
import frc.robot.utils.SignalStore;

public class Climber extends SubsystemBase {
    private final TalonFX motor;

    private final StatusSignal<Double> position, current, velocity, voltage;

    private final DigitalInput forwardLimitSwitch, reverseLimitSwitch;

    private static final double ZERO_VBUS = -0.50;

    private static final int CAN_ID = 15;

    private static final int FORWARD_LIMIT_SWITCH_PIN = 9;
    private static final int REVERSE_LIMIT_SWITCH_PIN = 8;

    private static final double ZERO_POSITION = 0.0;
    private static final double UP_POSITION = 118.0;

    /* Configs */
    private final CurrentLimitsConfigs currentConfigs = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(100.)
            .withSupplyCurrentLimit(80.);

    private final Slot0Configs m_pidConfigs = new Slot0Configs()
            // .withKP(0.4);
            .withKP(0.2);

    /* Requests */
    private final DutyCycleOut m_focRequest = new DutyCycleOut(0.)
            .withEnableFOC(true);

    private final PositionDutyCycle m_positionRequest = new PositionDutyCycle(0.)
            .withEnableFOC(true);
    // .withSlot(0);

    private double m_target = 0.;
    private double m_targetSign = 1;

    public enum ClimberPositions {
        CLIMB(0.0),
        HOLD(0.0),
        DISENGAGE(70.),
        READY(111.0);

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
        voltage = motor.getMotorVoltage();

        forwardLimitSwitch = new DigitalInput(FORWARD_LIMIT_SWITCH_PIN);
        reverseLimitSwitch = new DigitalInput(REVERSE_LIMIT_SWITCH_PIN);

        motor.setNeutralMode(NeutralModeValue.Brake);
        motor.setInverted(true);

        /* ======= */
        /* CONFIGS */
        /* ======= */
        motor.getConfigurator().apply(m_pidConfigs);
        motor.getConfigurator().apply(currentConfigs);

        /* CAN Bus */
        BaseStatusSignal.setUpdateFrequencyForAll(20.0, velocity, position, current, voltage);

        motor.optimizeBusUtilization();

        SignalStore.add(velocity, position, current, voltage);

        /* Logs */
        LogStore.add("/Climber/Current", current::getValueAsDouble);
        LogStore.add("/Climber/Position", position::getValueAsDouble);
        LogStore.add("/Climber/Velocity", velocity::getValueAsDouble);
        LogStore.add("/Climber/Voltage", voltage::getValueAsDouble);

        LogStore.add("/Climber/Forward Limit", forwardLimitSwitch::get);
        LogStore.add("/Climber/Reverse Limit", reverseLimitSwitch::get);

        /* Dashboard */
        DashboardStore.add("Climber Position", position::getValueAsDouble);
        DashboardStore.add("Climber Current", current::getValueAsDouble);
        DashboardStore.add("Climber Velocity", velocity::getValueAsDouble);

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
        return forwardLimitSwitch.get() && voltage.getValueAsDouble() > 0.2;
    }

    public boolean reverseLimit() {
        return reverseLimitSwitch.get() && voltage.getValueAsDouble() < -0.2;
    }

    public void stop() {
        runMotor(0.0, false);
    }

    public Command stopCommand() {
        return runOnce(() -> motor.stopMotor());
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

    public Command runToPositionCommand(double output, double position, boolean hold) {
        return runOnce(() -> {
            m_target = position;
            m_targetSign = Math.signum(getError());
        })
                .andThen(runOnce(() -> runMotor(m_targetSign * Math.abs(output), true)))
                // make sure overdrives aren't (generally) possible
                .andThen(Commands.waitUntil(() -> m_targetSign == -1 ? //
                        getError() > -2.0 : getError() < 2.0))
                .andThen(stopCommand())
                .andThen(holdCommand().onlyIf(() -> hold));
    }

    public Command runToPositionCommand(double output, ClimberPositions position, boolean hold) {
        return runToPositionCommand(output, position.Position, hold);
    }

    public Command hitForwardLimitCommand() {
        return stopCommand().andThen(setEncoderPositionCommand(UP_POSITION));
    }

    public Command hitReverseLimitCommand() {
        return stopCommand().andThen(setEncoderPositionCommand(ZERO_POSITION))
                .andThen(holdCommand());
    }

    public Command holdCurrentPositionCommand() {
        return run(() -> motor.setControl(m_positionRequest.withPosition(position.getValueAsDouble())));
    }

    public Command holdCommand() {
        return run(() -> motor.setControl(m_positionRequest.withPosition(ClimberPositions.HOLD.Position)))
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }

    public Command zeroCommand() {
        return runMotorCommand(ZERO_VBUS, false).repeatedly();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // motor.set(SmartDashboard.getNumber("Climber VBus", ));
    }
}
