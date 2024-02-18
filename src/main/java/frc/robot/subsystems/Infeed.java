// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import java.util.function.BooleanSupplier;

import com.playingwithfusion.TimeOfFlight;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Infeed extends SubsystemBase {
    private static Infeed instance;
    private final TimeOfFlight tofSensor;
    private final CANSparkFlex infeedMotor;
    private final RelativeEncoder infeedEncoder;

    private final DataLog log;
    public DoubleLogEntry infeedMotorCurrent, infeedMotorVelocity;

    private final double RANGE_THRESH = 200;

    /** Creates a new SensorMotor. */
    private Infeed() {
        log = DataLogManager.getLog();
        configureLogs();
        tofSensor = new TimeOfFlight(1);
        infeedMotor = new CANSparkFlex(18, MotorType.kBrushless);
        infeedEncoder = infeedMotor.getEncoder();
        infeedMotor.setInverted(true);
        infeedMotor.setSmartCurrentLimit(60);
        infeedMotor.setIdleMode(IdleMode.kBrake);

        infeedEncoder.setMeasurementPeriod(16);
        infeedEncoder.setAverageDepth(2);

        tofSensor.setRangeOfInterest(4, 4, 11, 11);
    }

    public boolean hasGamePiece() {
        return tofSensor.getRange() < RANGE_THRESH;
    }

    public BooleanSupplier hasGamePieceSupplier() {
        return this::hasGamePiece;
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
        SmartDashboard.putNumber("Infeed Tof", tofSensor.getRange());
    }

    public static Infeed getInstance() {
        if (instance == null)
            instance = new Infeed();
        return instance;
    }
}
