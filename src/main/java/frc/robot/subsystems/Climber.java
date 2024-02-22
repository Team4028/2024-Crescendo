// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    /** Creates a new Climber. */
    private final CANSparkMax m_motor;
    private final RelativeEncoder m_encoder;
    private final SparkPIDController m_pid;
    private final DataLog m_log;
    private final DoubleLogEntry m_vbusLog, m_currentLog, m_positionLog, m_velocityLog;

    private final Timer m_zeroTimer;

    private final double ZERO_TIMER_THRESHOLD = 0.1; // 5 scans
    private final double ZERO_VELOCITY_THRESHOLD = 5;

    private static final int CAN_ID = 15;
    private static final int CURRENT_LIMIT = 100;

    public enum ClimberPositions {
        HOME(1.),
        DOWN_ONE(50.),
        DOWN_TWO(40.),
        READY(65.);

        double Position;

        private ClimberPositions(double position) {
            Position = position;
        }
    }

    private final class PIDConstants {
        private static final double kP = 0.045;
        private static final double kI = 0.0;
        private static final double kD = 0.0;
        private static final double MAX_OUTPUT = 0.95;
    }

    public Climber() {
        m_motor = new CANSparkMax(CAN_ID, MotorType.kBrushless);
        m_motor.setIdleMode(IdleMode.kBrake);
        m_motor.setInverted(false);
        m_motor.setSmartCurrentLimit(CURRENT_LIMIT);

        m_encoder = m_motor.getEncoder();

        // PID
        m_pid = m_motor.getPIDController();
        m_pid.setFeedbackDevice(m_encoder);

        m_pid.setP(PIDConstants.kP);
        m_pid.setI(PIDConstants.kI);
        m_pid.setD(PIDConstants.kD);
        m_pid.setOutputRange(-PIDConstants.MAX_OUTPUT, PIDConstants.MAX_OUTPUT);

        m_motor.burnFlash();

        m_zeroTimer = new Timer();

        m_log = DataLogManager.getLog();
        m_vbusLog = new DoubleLogEntry(m_log, "/Climber/Vbus");
        m_currentLog = new DoubleLogEntry(m_log, "/Climber/Current");
        m_positionLog = new DoubleLogEntry(m_log, "/Climber/Position");
        m_velocityLog = new DoubleLogEntry(m_log, "/Climber/Velocity");
    }

    public void runMotor(double vBus) {
        m_motor.set(vBus);
    }

    public Command runMotorCommand(double vBus) {
        return runOnce(() -> runMotor(vBus));
    }

    public Command zeroCommand() {
        return runOnce(() -> {
            m_zeroTimer.restart();
        })
                .andThen(runMotorCommand(-0.1).repeatedly()
                        .until(() -> m_zeroTimer.get() >= ZERO_TIMER_THRESHOLD
                                && Math.abs(m_encoder.getVelocity()) < ZERO_VELOCITY_THRESHOLD))
                .andThen(runMotorCommand(0.),
                        Commands.runOnce(() -> m_zeroTimer.stop()),
                        Commands.runOnce(() -> m_encoder.setPosition(0.)));
    }

    public void runToPosition(double position) {
        m_pid.setReference(position, ControlType.kPosition);
    }

    public Command runToPositionCommand(double position) {
        return runOnce(() -> runToPosition(position));
    }

    public Command runToPositionCommand(ClimberPositions position) {
        return runToPositionCommand(position.Position);
    }

    public void logValues() {
        m_vbusLog.append(m_motor.get());
        m_currentLog.append(m_motor.getOutputCurrent());
        m_positionLog.append(m_encoder.getPosition());
        m_velocityLog.append(m_encoder.getVelocity());
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("Climber Position", m_encoder.getPosition());
        SmartDashboard.putNumber("Climber Current", m_motor.getOutputCurrent());
    }
}
