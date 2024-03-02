// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Fan extends SubsystemBase {
    private final CANSparkFlex m_motor;
    private final RelativeEncoder m_encoder;

    private static final int CAN_ID = 14;

    private final DataLog m_log;
    private final DoubleLogEntry m_vbusLog, m_currentLog, m_velocityLog;

    /** Creates a new Fan. */
    public Fan() {
        m_motor = new CANSparkFlex(CAN_ID, MotorType.kBrushless);
        m_encoder = m_motor.getEncoder();

        m_log = DataLogManager.getLog();
        m_vbusLog = new DoubleLogEntry(m_log, "/Fan/Vbus");
        m_currentLog = new DoubleLogEntry(m_log, "/Fan/Current");
        m_velocityLog = new DoubleLogEntry(m_log, "/Fan/Velocity");
    }

    public void runMotor(double vbus) {
        m_motor.set(vbus);
    }

    public Command runMotorCommand(double vbus) {
        return startEnd(
                () -> runMotor(vbus),
                () -> runMotor(0.));
    }

    public void stop() {
        runMotor(0.);
    }

    public Command stopCommand() {
        return runOnce(this::stop);
    }

    public void logValues() {
        m_vbusLog.append(m_motor.getAppliedOutput());
        m_currentLog.append(m_motor.getOutputCurrent());
        m_velocityLog.append(m_encoder.getVelocity());
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("Fan RPM", m_encoder.getVelocity());
    }
}
