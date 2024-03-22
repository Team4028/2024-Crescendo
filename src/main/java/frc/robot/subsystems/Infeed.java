// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.DashboardStore;

public class Infeed extends SubsystemBase {
    private final TimeOfFlight tofSensor;
    private final TalonFX motor;

    private final DataLog log;
    private final DoubleLogEntry currentLog, velocityLog;

    private static final double RANGE_THRESHOLD = 200;
    private static final int CAN_ID = 18;
    private static final int TOF_CAN_ID = 1;

    private static final double TOF_SAMPLE_TIME = 24.0;
    private static final int[] TOF_ROI = new int[] { 4, 4, 11, 11 };

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

        tofSensor = new TimeOfFlight(TOF_CAN_ID);

        tofSensor.setRangingMode(RangingMode.Short, TOF_SAMPLE_TIME);
        tofSensor.setRangeOfInterest(TOF_ROI[0], TOF_ROI[1], TOF_ROI[2], TOF_ROI[3]);

        log = DataLogManager.getLog();
        currentLog = new DoubleLogEntry(log, "/Infeed/Current");
        velocityLog = new DoubleLogEntry(log, "/Infeed/Velocity");

        /* Dashboard */
        DashboardStore.add("Infeed Tof", () -> tofSensor.getRange());

        DashboardStore.add("Running/Infeed", this::isRunning);
    }

    // ==================================
    // REAL STUFF
    // ==================================

    /* Check if shooter is running */
    public boolean isRunning() {
        return Math.abs(motor.getMotorVoltage().getValueAsDouble()) > 0.2;
    }

    public boolean hasGamePiece() {
        return tofSensor.getRange() < RANGE_THRESHOLD;
    }

    public BooleanSupplier hasGamePieceSupplier() {
        return this::hasGamePiece;
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
