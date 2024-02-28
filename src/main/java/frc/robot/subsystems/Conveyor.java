// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.units.Per;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
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
import com.playingwithfusion.TimeOfFlight.RangingMode;

public class Conveyor extends SubsystemBase {
    private final CANSparkFlex motor;
    private final RelativeEncoder encoder;
    private final TimeOfFlight tofSensor;
    private final SparkPIDController pid;
    private final DataLog log;
    private final DoubleLogEntry currentLog, vbusLog, positionLog, velocityLog;

    private final static class PIDConstants {
        private static final double kP = 0.8;
        private static final double kI = 0.0;
        private static final double kD = 0.0;
        private static final double[] kOutputRange = new double[] { -0.3, 0.3 };

        private static final double COMMAND_ESCAPE_THRESHOLD = 0.06;
    }

    private static final double RANGE_THRESHOLD = 70;
    private static final double OTHER_RANGE_THRESHOLD = 60.;
    private static final double TOLERANCE = 10;
    private static final double NO_PID_ROT_VBUS = 0.2;

    private static final int CAN_ID = 11;
    private static final int TOF_CAN_ID = 21;

    private boolean hasInfed = false;
    private double target;
    private Timer timer;
    private double setPos = 0;

    public Conveyor() {
        timer = new Timer();
        timer.reset();
        motor = new CANSparkFlex(CAN_ID, MotorType.kBrushless);
        motor.setIdleMode(IdleMode.kCoast);
        motor.setInverted(true);
        motor.setClosedLoopRampRate(.1);
        encoder = motor.getEncoder();

        pid = motor.getPIDController();
        pid.setP(PIDConstants.kP);
        pid.setI(PIDConstants.kI);
        pid.setD(PIDConstants.kD);
        pid.setIZone(0);
        pid.setOutputRange(PIDConstants.kOutputRange[0], PIDConstants.kOutputRange[1]);

        motor.burnFlash();

        tofSensor = new TimeOfFlight(TOF_CAN_ID);
        tofSensor.setRangingMode(RangingMode.Short, 24.0);
        tofSensor.setRangeOfInterest(4, 4, 11, 11);

        log = DataLogManager.getLog();

        currentLog = new DoubleLogEntry(log, "/Conveyor/Current");
        vbusLog = new DoubleLogEntry(log, "/Conveyor/Vbus");
        positionLog = new DoubleLogEntry(log, "/Conveyor/Position");
        velocityLog = new DoubleLogEntry(log, "/Conveyor/Velocity");
    }

    public boolean getHasInfed() {
        if (tofSensor.getRange() < OTHER_RANGE_THRESHOLD) {
            hasInfed = true;
            timer.start();
        }

        if (hasInfed && ((tofSensor.getRange() >= RANGE_THRESHOLD) || (timer.get() >= 0.75))) {
            hasInfed = false;
            timer.stop();
            timer.reset();
            return true;
        }

        return false;
    }

    public BooleanSupplier hasInfedSupplier() {
        return this::getHasInfed;
    }

    public boolean hasGamePiece() {
        return tofSensor.getRange() < RANGE_THRESHOLD;
    }

    public Command runXRotations(double x) {
        return runOnce(() -> {
            target = encoder.getPosition() + x;
            pid.setReference(target, ControlType.kPosition);
        }).andThen(Commands.idle(this)).until(() -> Math.abs(target - encoder.getPosition()) < 0.06);
    }

    public Command runXRotationsNoPID(double x) {
        return runOnce(() -> setPos = encoder.getPosition() + x)
                .andThen(runMotorCommand(NO_PID_ROT_VBUS * Math.signum(x)).repeatedly()
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
        currentLog.append(motor.getOutputCurrent());
        vbusLog.append(motor.getAppliedOutput());
        positionLog.append(encoder.getPosition());
        velocityLog.append(encoder.getVelocity());
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("ToF Sensor", tofSensor.getRange());
    }
}
