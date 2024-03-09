// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

import java.util.function.BooleanSupplier;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

public class Conveyor extends SubsystemBase {
    private final CANSparkFlex motor;
    private final RelativeEncoder encoder;
    private final TimeOfFlight tofSensor;
    private final TimeOfFlight infeedTof;
    private final SparkPIDController pid;
    private final DataLog log;
    private final DoubleLogEntry currentLog, vbusLog, positionLog, velocityLog;

    private final static class PIDConstants {
        private static final double kP = 0.8;
        private static final double kI = 0.0;
        private static final double kD = 0.0;
        private static final double[] kOutputRange = new double[] { -0.3, 0.6 };
    }

    private static final double NOTE_HELD_RANGE_THRESHOLD = 70.;
    private static final double CONVEYOR_INITIAL_DETECTION_RANGE_THRESHOLD = 60.;
    private static final double CONVEYOR_BACKDRIVE_TIMER = 0.75;
    private static final double INFEED_BACKDRIVE_TIMER = 2.00;
    private static final double INFEED_INITIAL_DETECTION_RANGE_THRESHOLD = 60.;

    private static final int CAN_ID = 11;
    private static final int TOF_CAN_ID = 21;
    private static final int INFEED_TOF_ID = 1;

    private boolean hasInfed = false;
    private double target;
    private final Timer conveyorTimer;
    private final Timer infeedTimer;

    public Conveyor() {
        conveyorTimer = new Timer();
        conveyorTimer.reset();
        infeedTimer = new Timer();
        infeedTimer.reset();

        motor = new CANSparkFlex(CAN_ID, MotorType.kBrushless);

        motor.restoreFactoryDefaults();

        motor.setIdleMode(IdleMode.kCoast);
        motor.setInverted(true);
        motor.setClosedLoopRampRate(.1);

        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20);
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 101);
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 102);
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 103);
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 104);
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus7, 106);
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

        infeedTof = new TimeOfFlight(INFEED_TOF_ID);
        tofSensor.setRangingMode(RangingMode.Short, 24.0);
        tofSensor.setRangeOfInterest(4, 4, 11, 11);

        log = DataLogManager.getLog();

        currentLog = new DoubleLogEntry(log, "/Conveyor/Current");
        vbusLog = new DoubleLogEntry(log, "/Conveyor/Vbus");
        positionLog = new DoubleLogEntry(log, "/Conveyor/Position");
        velocityLog = new DoubleLogEntry(log, "/Conveyor/Velocity");
    }

    public boolean getHasInfed() {
        if (infeedTof.getRange() < INFEED_INITIAL_DETECTION_RANGE_THRESHOLD) {
            infeedTimer.start();
        }

        if (tofSensor.getRange() < CONVEYOR_INITIAL_DETECTION_RANGE_THRESHOLD) {
            hasInfed = true;
            conveyorTimer.start();
        }

        if ((hasInfed && ((tofSensor.getRange() >= NOTE_HELD_RANGE_THRESHOLD)
                || (conveyorTimer.get() >= CONVEYOR_BACKDRIVE_TIMER)))
                /*|| infeedTimer.get() >= INFEED_BACKDRIVE_TIMER*/) {
            hasInfed = false;
            conveyorTimer.stop();
            conveyorTimer.reset();
            infeedTimer.stop();
            infeedTimer.reset();
            return true;
        }

        return false;
    }

    public BooleanSupplier hasInfedSupplier() {
        return this::getHasInfed;
    }

    public boolean hasGamePiece() {
        return tofSensor.getRange() < NOTE_HELD_RANGE_THRESHOLD;
    }

    public Command runXRotations(double x) {
        return runOnce(() -> {
            target = encoder.getPosition() + x;
            pid.setReference(target, ControlType.kPosition);
        }).andThen(Commands.idle(this)).until(() -> Math.abs(target - encoder.getPosition()) < 0.06);
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
