// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
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

    private final static class PIDConstants {
        private static final double kP = 0.4;
        private static final double kI = 0.0;
        private static final double kD = 0.0;
    }

    private double target;

    public Feeder() {
        feederMotor = new CANSparkFlex(11, MotorType.kBrushless);
        feederMotor.setIdleMode(IdleMode.kBrake);
        feederMotor.setClosedLoopRampRate(.1);
        feederEncoder = feederMotor.getEncoder();
        tofSensor = new TimeOfFlight(21);
        log = DataLogManager.getLog();

        fCurrent = new DoubleLogEntry(log, "/Feeder/Current");
        fVBus = new DoubleLogEntry(log, "/Feeder/vBus");
        fPosition = new DoubleLogEntry(log, "/Feeder/Position");
        fVelocity = new DoubleLogEntry(log, "/Feeder/Velocity");

        pid = feederMotor.getPIDController();
        pid.setP(PIDConstants.kP);
        pid.setI(PIDConstants.kI);
        pid.setD(PIDConstants.kD);
        pid.setIZone(0);
        pid.setOutputRange(-.8, .8);
    }

    public boolean hasGamePiece() {
        return tofSensor.getRange() < RANGE_THRESH;
    }

    public Command runXRotations(double x) {
        return runOnce(() -> {
            target = feederEncoder.getPosition() + x;
            pid.setReference(target, ControlType.kPosition);
        }).andThen(Commands.idle(this)).until(() -> Math.abs(target - feederEncoder.getPosition()) < 0.06);
    }

    public Command runXRotationsNoPID(double x) {
        return runOnce(() -> setPos = feederEncoder.getPosition() + x)
                .andThen(runFeederMotorCommand(0.2 * Math.signum(x)).repeatedly()
                        .until(() -> Math.signum(x) == 1 ? feederEncoder.getPosition() > setPos
                                : feederEncoder.getPosition() < setPos));
        // .finallyDo(() -> runFeederMotor(0.));
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

    public void logValues() {
        fCurrent.append(feederMotor.getOutputCurrent());
        fVBus.append(feederMotor.getAppliedOutput());
        fPosition.append(feederEncoder.getPosition());
        fVelocity.append(feederEncoder.getVelocity());
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("Feeder ToF Sensor", tofSensor.getRange());
        SmartDashboard.putNumber("Feeder Position", feederEncoder.getPosition());
    }
}
