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

public class Conveyor extends SubsystemBase {
    private final CANSparkFlex motor;
    private final RelativeEncoder encoder;
    private final TimeOfFlight tofSensor;
    private final SparkPIDController pid;
    private final DataLog log;
    private final DoubleLogEntry current, vBus, position, velocity;

    private final static class PIDConstants {
        private static final double kP = 0.8;
        private static final double kI = 0.0;
        private static final double kD = 0.0;
    }

    private final double RANGE_THRESH = 100;
    private final double OTHER_RANGE_THRESH = 75;
    private final double TOLERANCE = 10;
    private boolean infedFlag = false;

    private double target;
    private double setPos = 0;

    public Conveyor() {
        motor = new CANSparkFlex(11, MotorType.kBrushless);
        motor.setIdleMode(IdleMode.kCoast);
        motor.setClosedLoopRampRate(.1);
        encoder = motor.getEncoder();
        tofSensor = new TimeOfFlight(21);
        log = DataLogManager.getLog();

        current = new DoubleLogEntry(log, "/Conveyor/Current");
        vBus = new DoubleLogEntry(log, "/Conveyor/vBus");
        position = new DoubleLogEntry(log, "/Conveyor/Position");
        velocity = new DoubleLogEntry(log, "/Conveyor/Velocity");

        pid = motor.getPIDController();
        pid.setP(PIDConstants.kP);
        pid.setI(PIDConstants.kI);
        pid.setD(PIDConstants.kD);
        pid.setIZone(0);
        pid.setOutputRange(-.3, .3);

        tofSensor.setRangeOfInterest(4, 4, 11, 11);
    }

    public boolean hasInfed() {
        if (tofSensor.getRange() < OTHER_RANGE_THRESH) {
            infedFlag = true;
        }

        if (infedFlag && Math.abs(tofSensor.getRange() - RANGE_THRESH) <= TOLERANCE) {
            infedFlag = false;
            return true;
        }

        return false;
    }

    public BooleanSupplier hasInfedSupplier() {
        return this::hasInfed;
    }

    public boolean hasGamePiece() {
        return tofSensor.getRange() < RANGE_THRESH;
    }

    public Command runXRotations(double x) {
        return runOnce(() -> {
            target = encoder.getPosition() + x;
            pid.setReference(target, ControlType.kPosition);
        }).andThen(Commands.idle(this)).until(() -> Math.abs(target - encoder.getPosition()) < 0.06);
    }

    public Command runXRotationsNoPID(double x) {
        return runOnce(() -> setPos = encoder.getPosition() + x)
                .andThen(runMotorCommand(0.2 * Math.signum(x)).repeatedly()
                        .until(() -> Math.signum(x) == 1 ? encoder.getPosition() > setPos
                                : encoder.getPosition() < setPos));
    }

    public BooleanSupplier hasGamePieceSupplier() {
        return this::hasGamePiece;
    }

    public final void runMotor(double vBus) {
        motor.set(vBus);
    }

    public Command runMotorCommand(double vBus) {
        return runOnce(() -> runMotor(vBus));
    }

    public void logValues() {
        current.append(motor.getOutputCurrent());
        vBus.append(motor.getAppliedOutput());
        position.append(encoder.getPosition());
        velocity.append(encoder.getVelocity());
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("ToF Sensor", tofSensor.getRange());
    }
}
