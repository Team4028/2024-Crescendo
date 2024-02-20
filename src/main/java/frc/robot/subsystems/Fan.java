// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Fan extends SubsystemBase {
    private final CANSparkFlex m_motor;
    private final RelativeEncoder m_encoder;

    /** Creates a new Fan. */
    public Fan() {
        m_motor = new CANSparkFlex(14, MotorType.kBrushless);
        m_encoder = m_motor.getEncoder();
    }

    public void runMotor(double vbus) {
        m_motor.set(vbus);
    }

    public Command runMotorCommand(double vbus) {
        return startEnd(
                () -> runMotor(vbus),
                () -> runMotor(0.));
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("Fan RPM", m_encoder.getVelocity());
    }
}
