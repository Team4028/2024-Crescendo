// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.DashboardStore;

public class Whippy extends SubsystemBase {
    private final CANSparkFlex motor;

    private static final int CAN_ID = 16;
    private static final int CURRENT_LIMIT = 40;

    private final DataLog log;
    private final DoubleLogEntry velocityLog;

    /** Creates a new WhippyWheels. */

    public Whippy() {
        motor = new CANSparkFlex(CAN_ID, MotorType.kBrushless);
        motor.setInverted(true);
        motor.setSmartCurrentLimit(CURRENT_LIMIT);

        /* Logging */
        log = DataLogManager.getLog();
        
        velocityLog = new DoubleLogEntry(log, "Whippy/Velocity");

        /* Dashboard */
        DashboardStore.add("Running/Whippy", this::isRunning);
    }

    // ==================================
    // SHOOTER COMMANDS
    // ==================================

    /* Check if shooter is running */
    public boolean isRunning() {
        return Math.abs(motor.getEncoder().getVelocity()) > 50;
    }

    public void whippyWheels(double vbus) {
        motor.set(vbus);
    }

    public Command whippyWheelsCommand(double vbus) {
        return runOnce(() -> whippyWheels(vbus));
    }

    public void stop() {
        whippyWheels(0.);
    }

    public Command stopCommand() {
        return whippyWheelsCommand(0.);
    }

    public void logValues() {
        velocityLog.append(motor.getEncoder().getVelocity());
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler ru

    }
}
