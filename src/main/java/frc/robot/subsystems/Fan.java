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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.DashboardStore;

public class Fan extends SubsystemBase {
    private final CANSparkFlex motor;
    private final RelativeEncoder encoder;

    private static final int FAN_CAN_ID = 14;

    private final DataLog m_log;
    private final DoubleLogEntry m_vbusLog, m_currentLog, m_velocityLog;

    /** Creates a new Fan. */
    public Fan() {
        motor = new CANSparkFlex(FAN_CAN_ID, MotorType.kBrushless);
        encoder = motor.getEncoder();

        m_log = DataLogManager.getLog();
        m_vbusLog = new DoubleLogEntry(m_log, "/Fan/Vbus");
        m_currentLog = new DoubleLogEntry(m_log, "/Fan/Current");
        m_velocityLog = new DoubleLogEntry(m_log, "/Fan/Velocity");

        DashboardStore.add("Running/Fan", this::isRunning);
        DashboardStore.add("Fan RPM", () -> encoder.getVelocity());
    }

    // ==================================
    // FAN STUFF
    // ==================================

    /* Check if fan is running */
    public boolean isRunning() {
        return Math.abs(encoder.getVelocity()) > 200.;
    }

    // Fan Motor Controls //
    public void runMotor(double vbus) {
        motor.set(vbus);
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
        m_vbusLog.append(motor.getAppliedOutput());
        m_currentLog.append(motor.getOutputCurrent());
        m_velocityLog.append(encoder.getVelocity());
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
