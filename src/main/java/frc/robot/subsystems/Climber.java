// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    /** Creates a new Climber. */
    private final CANSparkMax m_motor;
    private final RelativeEncoder m_encoder;
    private final DataLog m_log;
    private final DoubleLogEntry m_vbusLog, m_currentLog, m_positionLog, m_velocityLog;

    private final Timer m_zeroTimer;

    private Climber() {
        m_motor = new CANSparkMax(15, MotorType.kBrushless);

        m_zeroTimer = new Timer();

        m_encoder = m_motor.getEncoder();

        m_log = DataLogManager.getLog();
        m_vbusLog = new DoubleLogEntry(m_log, "Climber/VBus");
        m_currentLog = new DoubleLogEntry(m_log, "Climber/Current");
        m_positionLog = new DoubleLogEntry(m_log, "Climber/Position");
        m_velocityLog = new DoubleLogEntry(m_log, "Climber/Velocity");
    }

    public void runMotor(double vBus) {
        m_motor.set(vBus);
    }

    public Command runMotorCommand(double vBus) {
        return runOnce(() -> runMotor(vBus));
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
    }
}
