// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Whippy extends SubsystemBase {
    private final CANSparkFlex motor;

    /** Creates a new WhippyWheels. */

    public Whippy() {
        motor = new CANSparkFlex(16, MotorType.kBrushless);
    }

    public void whippyWheels(double vbus) {
        motor.set(vbus);
    }

    public Command whippyWheelsCommand(double vbus) {
        return runOnce(() -> whippyWheels(vbus));
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler ru

    }
}
