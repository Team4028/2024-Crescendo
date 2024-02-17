// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Infeed extends SubsystemBase {
    private static Infeed instance;
    private final CANSparkFlex infeedMotor;
    private final RelativeEncoder infeedEncoder;

    private final DataLog log;
    public DoubleLogEntry infeedMotorCurrent, infeedMotorVelocity;

    /** Creates a new SensorMotor. */
    private Infeed() {
        log = DataLogManager.getLog();
        configureLogs();
        infeedMotor = new CANSparkFlex(18, MotorType.kBrushless);
        infeedEncoder = infeedMotor.getEncoder();
        infeedMotor.setInverted(true);
        infeedMotor.setSmartCurrentLimit(60);

        infeedEncoder.setMeasurementPeriod(16);
        infeedEncoder.setAverageDepth(2);
    }

    private void runInfeedMotor(double vBus) {
        infeedMotor.set(vBus);
    }

    public Command runInfeedMotorCommand(double vBus) {
        return runOnce(() -> runInfeedMotor(vBus));
    }

    private void configureLogs() {
        infeedMotorCurrent = new DoubleLogEntry(log, "Current");
        infeedMotorVelocity = new DoubleLogEntry(log, "Velocity");
    }

    public void logValues() {
        infeedMotorCurrent.append(infeedMotor.getOutputCurrent());
        infeedMotorVelocity.append(infeedEncoder.getVelocity());
    }

    @Override
    public void periodic() {
    }

    public static Infeed getInstance() {
        if (instance == null)
            instance = new Infeed();
        return instance;
    }
}
