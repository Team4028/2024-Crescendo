// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import java.util.function.BooleanSupplier;
import com.playingwithfusion.TimeOfFlight;

public class Feeder extends SubsystemBase {
    private final CANSparkFlex feederMotor; // TODO: log this
    private final RelativeEncoder feederEncoder;
    private final TimeOfFlight tofSensor;
    private final double RANGE_THRESH = 65;
    private final SparkPIDController pid;
    private double setPos = 0;
    private final DataLog log;
    private final DoubleLogEntry fCurrent, fVBus, fPosition, fVelocity;

    public Feeder() {
        feederMotor = new CANSparkFlex(11, MotorType.kBrushless);
        feederEncoder = feederMotor.getEncoder();
        tofSensor = new TimeOfFlight(21);
        log = DataLogManager.getLog();

        fCurrent = new DoubleLogEntry(log, "/Feeder Motor/Current");
        fVBus = new DoubleLogEntry(log, "/Feeder Motor/vBus");
        fPosition = new DoubleLogEntry(log, "/Feeder Motor/Position");
        fVelocity = new DoubleLogEntry(log, "/Feeder Motor/Velocity");

        pid = feederMotor.getPIDController();
        pid.setP(0);
        pid.setI(0);
        pid.setD(0);
        pid.setIZone(0);
        pid.setOutputRange(-.9, .9);
    }

    public boolean hasGamePiece() {
        return tofSensor.getRange() < RANGE_THRESH;
    }

    public Command runXRotations(double x) {
        return run(() -> pid.setReference(feederMotor.getEncoder().getPosition() + x, ControlType.kPosition));
    }

    public Command runXRotationsNoPID(double x) {
        return runOnce(() -> setPos = feederEncoder.getPosition() + x)
                .andThen(runFeederMotorCommand(0.2 * Math.signum(x)).repeatedly()
                        .until(() -> Math.signum(x) == 1 ? feederEncoder.getPosition() > setPos
                                : feederEncoder.getPosition() < setPos));
    }

    public BooleanSupplier hasGamePieceSupplier() {
        return this::hasGamePiece;
    }

    public final void runFeederMotor(double vBus) {
        feederMotor.set(vBus);
    }

    public Command runFeederMotorCommand(double vBus) {
        return runOnce(() -> runFeederMotor(vBus));
    }

    public void logFeeder() {
        fCurrent.append(feederMotor.getOutputCurrent());
        fVBus.append(feederMotor.getAppliedOutput());
        fPosition.append(feederEncoder.getPosition());
        fVelocity.append(feederEncoder.getVelocity());
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("TOF FEEDEDE", tofSensor.getRange());
    }
}
