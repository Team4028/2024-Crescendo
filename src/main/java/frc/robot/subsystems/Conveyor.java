// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.DashboardStore;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

public class Conveyor extends SubsystemBase {
    private final CANSparkFlex motor;
    private final RelativeEncoder encoder;

    private final SparkPIDController pid;
    private final DataLog log;
    private final DoubleLogEntry currentLog, vbusLog, positionLog, velocityLog;

    private final static class PIDConstants {
        private static final double kP = 0.2;
        private static final double kI = 0.0;
        private static final double kD = 0.0;
        private static final double[] kOutputRange = new double[] { -0.3, 0.6 };
    }

    private static final int CAN_ID = 11;

    private double target;

    public Conveyor() {
        /* Main Setup */
        motor = new CANSparkFlex(CAN_ID, MotorType.kBrushless);
        motor.restoreFactoryDefaults();

        motor.setIdleMode(IdleMode.kCoast);
        motor.setInverted(true);
        motor.setClosedLoopRampRate(.1);

        /* CAN Bus */
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

        log = DataLogManager.getLog();

        currentLog = new DoubleLogEntry(log, "/Conveyor/Current");
        vbusLog = new DoubleLogEntry(log, "/Conveyor/Vbus");
        positionLog = new DoubleLogEntry(log, "/Conveyor/Position");
        velocityLog = new DoubleLogEntry(log, "/Conveyor/Velocity");

        /* Dashboard */
        DashboardStore.add("Running/Conveyor", this::isRunning);
    }

    // ==================================
    // SHOOTER COMMANDS
    // ==================================

    /* Check if shooter is running */
    public boolean isRunning() {
        return Math.abs(motor.getAppliedOutput()) > 0.1;
    }

    public Command runXRotations(double x) {
        return runOnce(() -> {
            target = encoder.getPosition() + x;
            pid.setReference(target, ControlType.kPosition);
        }).andThen(Commands.idle(this)).until(() -> Math.abs(target - encoder.getPosition()) < 0.06);
    }

    public final void runMotor(double vBus) {
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

    public Command brakeStopCommand() {
        return runOnce(() -> pid.setReference(encoder.getPosition(), ControlType.kPosition))
                .andThen(Commands.waitSeconds(0.2))
                .andThen(stopCommand());
    }

    public void logValues() {
        currentLog.append(motor.getOutputCurrent());
        vbusLog.append(motor.getAppliedOutput());
        positionLog.append(encoder.getPosition());
        velocityLog.append(encoder.getVelocity());
    }

    @Override
    public void periodic() {}
}
