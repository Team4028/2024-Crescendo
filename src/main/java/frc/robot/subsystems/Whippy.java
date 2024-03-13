// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.DashboardStore;

public class Whippy extends SubsystemBase {
    private final CANSparkFlex motor;

    private static final int CAN_ID = 16;
    private static final int CURRENT_LIMIT = 40;

    /** Creates a new WhippyWheels. */

    public Whippy() {
        motor = new CANSparkFlex(CAN_ID, MotorType.kBrushless);
        motor.setInverted(true);
        motor.setSmartCurrentLimit(CURRENT_LIMIT);

        /* Dashboard */
        DashboardStore.add("Running/Whippy", this::isRunning);
    }

    // ==================================
    // SHOOTER COMMANDS
    // ==================================

    /* Check if shooter is running */
    public boolean isRunning() {
        return Math.abs(motor.getAppliedOutput()) > 0.1;
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

    @Override
    public void periodic() {
        // This method will be called once per scheduler ru

    }
}
