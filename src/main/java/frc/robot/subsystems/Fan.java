// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
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
    private final TalonFX pivot;

    private static final int FAN_CAN_ID = 14;
    private static final int PIVOT_CAN_ID = 17; // ? Actual id?

    private static final double TRAP_POSITION = 0.46; // Find out the actual position for the trap angle.

    /* Requests */
    private final PositionDutyCycle positionRequest = new PositionDutyCycle(0.)
            .withOverrideBrakeDurNeutral(true);

    private final Slot0Configs pidConfigs = new Slot0Configs()
            .withKP(0.8)
            .withKI(0.0)
            .withKD(0.0); // needs to be testing
    
    private final ClosedLoopRampsConfigs closedLoopRampsConfigs = new ClosedLoopRampsConfigs()
    .withDutyCycleClosedLoopRampPeriod(0.25);

    double targetPosition = 0;

    private final DataLog m_log;
    private final DoubleLogEntry m_vbusLog, m_currentLog, m_velocityLog, m_pivotVelocityLog, m_pivotPositionLog;

    /** Creates a new Fan. */
    public Fan() {
        motor = new CANSparkFlex(FAN_CAN_ID, MotorType.kBrushless);
        encoder = motor.getEncoder();

        m_log = DataLogManager.getLog();
        m_vbusLog = new DoubleLogEntry(m_log, "/Fan/Vbus");
        m_currentLog = new DoubleLogEntry(m_log, "/Fan/Current");
        m_velocityLog = new DoubleLogEntry(m_log, "/Fan/Velocity");

        // Creates a new pivot
        pivot = new TalonFX(PIVOT_CAN_ID);
        pivot.getConfigurator().apply(pidConfigs); // See Pivot Pid and configs below
        pivot.getConfigurator().apply(closedLoopRampsConfigs);
        pivot.setPosition(0.);

        // LOGGGGGGGGGGGGGGGGG DA PIVOTTTTT
        m_pivotPositionLog = new DoubleLogEntry(m_log, "/Fan/Pivot/Position");
        m_pivotVelocityLog = new DoubleLogEntry(m_log, "/Fan/Pivot/Velocity");

        DashboardStore.add("Fan Pivot Position", () -> pivot.getPosition().getValueAsDouble());
        DashboardStore.add("Fan Pivot Current", () -> pivot.getStatorCurrent().getValueAsDouble());
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

        m_pivotPositionLog.append(pivot.getPosition().getValueAsDouble());
        m_pivotVelocityLog.append(pivot.getVelocity().getValueAsDouble());
    }

    public void runToPosition(double position) {
        targetPosition = position;
        pivot.setControl(positionRequest.withPosition(position));
    }

    public Command runToPositionCommand(double position) {
        return runOnce(() -> runToPosition(position));
    }

    public Command runToTrapCommand() {
        return runToPositionCommand(TRAP_POSITION);
    }

    public void runPivot(double vbus) {
        pivot.set(vbus);
    }

    public Command runPivotCommand(double vbus) {
        return startEnd(
                () -> runPivot(vbus),
                () -> runPivot(0.));
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
