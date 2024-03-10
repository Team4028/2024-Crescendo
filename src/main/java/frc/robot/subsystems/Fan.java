// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Fan extends SubsystemBase {
    private final CANSparkFlex motor;
    private final RelativeEncoder encoder;
    private final TalonFX fanPivot;

    private static final int fanMotorCAN_ID = 14;


    private static final int pivotMotorCAN_ID = 16; //? Actual id?
    double targetPosition = 0;
    double trapPosition = 13; // Find out the actual position for the trap angle.

    private final DataLog m_log;
    private final DoubleLogEntry m_vbusLog, m_currentLog, m_velocityLog;

    /** Creates a new Fan. */
    public Fan() {
        motor = new CANSparkFlex(fanMotorCAN_ID, MotorType.kBrushless);
        encoder = motor.getEncoder();

        m_log = DataLogManager.getLog();
        m_vbusLog = new DoubleLogEntry(m_log, "/Fan/Vbus");
        m_currentLog = new DoubleLogEntry(m_log, "/Fan/Current");
        m_velocityLog = new DoubleLogEntry(m_log, "/Fan/Velocity");

        // Creates a new pivot
        fanPivot = new TalonFX(pivotMotorCAN_ID);
        fanPivot.getConfigurator().apply(pidConfigs); // See Pivot Pid and configs below
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

    // FAN PIVOT COMMANDS AND PID CONTROLS //

    /* Requests */
    private final PositionVoltage positionRequest = new PositionVoltage(0.)
            .withEnableFOC(true)
            .withOverrideBrakeDurNeutral(true);

    private final Slot0Configs pidConfigs = new Slot0Configs()
            .withKP(0.0)
            .withKI(0.0)
            .withKD(0.0); // needs to be testing

    public void runToPosition(double position) {
        targetPosition = position;
        fanPivot.setControl(positionRequest.withPosition(position)); 
    }

    public Command runToPositionCommand(double position) {
        return runOnce(() -> runToPosition(position));
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
