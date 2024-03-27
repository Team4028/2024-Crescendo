// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.DashboardStore;

public class Infeed extends SubsystemBase {
    private final TalonFX motor;

    private final DataLog log;
    private final DoubleLogEntry currentLog, velocityLog;

    private static final int CAN_ID = 18;

    private final CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(50.)
            .withStatorCurrentLimit(90.);

    private final MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Brake)
            .withInverted(InvertedValue.Clockwise_Positive);

    /** Creates a new SensorMotor. */
    public Infeed() {
        motor = new TalonFX(CAN_ID);

        motor.getConfigurator().apply(currentLimitsConfigs);
        motor.getConfigurator().apply(motorOutputConfigs);

        /* CAN Bus */
        motor.getDutyCycle().setUpdateFrequency(50.);
        motor.getVelocity().setUpdateFrequency(10.);
        motor.getStatorCurrent().setUpdateFrequency(10.);
        motor.optimizeBusUtilization();

        log = DataLogManager.getLog();
        currentLog = new DoubleLogEntry(log, "/Infeed/Current");
        velocityLog = new DoubleLogEntry(log, "/Infeed/Velocity");

        /* Dashboard */
        DashboardStore.add("Running/Infeed", this::isRunning);
    }

    // ==================================
    // REAL STUFF
    // ==================================

    /* Check if shooter is running */
    public boolean isRunning() {
        return Math.abs(motor.getMotorVoltage().getValueAsDouble()) > 0.2;
    }

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

    public void logValues() {
        currentLog.append(motor.getStatorCurrent().getValueAsDouble());
        velocityLog.append(motor.getVelocity().getValueAsDouble());
    }

    @Override
    public void periodic() {
    }
}
