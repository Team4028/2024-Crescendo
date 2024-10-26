// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.LogStore;
import frc.robot.utils.SignalStore;

public class Infeed extends SubsystemBase {
    private final TalonFX motor;

    private final StatusSignal<Double> current, velocity;

    private static final int CAN_ID = 18;

    private final CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs().withSupplyCurrentLimit(50.)
            .withStatorCurrentLimit(90.);

    private final MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Brake).withInverted(InvertedValue.Clockwise_Positive);

    /** Creates a new SensorMotor. */
    public Infeed() {
        motor = new TalonFX(CAN_ID);

        current = motor.getStatorCurrent();
        velocity = motor.getVelocity();

        motor.getConfigurator().apply(currentLimitsConfigs);
        motor.getConfigurator().apply(motorOutputConfigs);

        /* CAN Bus */
        BaseStatusSignal.setUpdateFrequencyForAll(10.0, velocity, current);

        motor.optimizeBusUtilization();

        SignalStore.add(current, velocity);

        /* Logs */
        LogStore.add("/Infeed/Current", current::getValueAsDouble);
        LogStore.add("/Infeed/Velocity", velocity::getValueAsDouble);
    }

    // ==================================
    // REAL STUFF
    // ==================================

    public void runMotor(double vBus) {
        motor.set(vBus);
    }

    public Command runMotorCommand(double vBus) {
        return runOnce(() -> runMotor(vBus));
    }

    public void stop() {
        runMotor(0.);
    }

    public Command stopCommand() {
        return runMotorCommand(0.);
    }

    @Override
    public void periodic() {
    }
}
