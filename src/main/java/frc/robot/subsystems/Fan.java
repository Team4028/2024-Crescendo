// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.DashboardStore;
import frc.robot.utils.LogStore;

public class Fan extends SubsystemBase {
    private final CANSparkFlex motor;
    private final RelativeEncoder encoder;

    private static final int CAN_ID = 14;

    /** Creates a new Fan. */
    public Fan() {
        /* Fan Setup */
        motor = new CANSparkFlex(CAN_ID, MotorType.kBrushless);
        encoder = motor.getEncoder();

        /* CAN Bus */
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 50);
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 51);
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 52);
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 101);
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 102);
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 103);
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 104);
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus7, 106);

        /* Logs */
        LogStore.add("/Fan/Vbus", motor::getAppliedOutput);
        LogStore.add("/Fan/Current", motor::getOutputCurrent);
        LogStore.add("/Fan/Velocity", encoder::getVelocity);

        /* Dashboard */
        DashboardStore.add("Running/Fan", this::isRunning);
        DashboardStore.add("Fan RPM", encoder::getVelocity);
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
        return runOnce(() -> runMotor(vbus));
    }

    public void stop() {
        runMotor(0.);
    }

    public Command stopCommand() {
        return runOnce(this::stop);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
