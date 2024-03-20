// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.DashboardStore;

public class FanPivot extends SubsystemBase {
    private final TalonFX motor;

    private static final int PIVOT_CAN_ID = 17; // ? Actual id?

    private static final double TRAP_POSITION = 0.57; // Find out the actual position for the trap angle.

    /* Requests */
    private final PositionDutyCycle positionRequest = new PositionDutyCycle(0.)
            .withOverrideBrakeDurNeutral(true);

    private final Slot0Configs pidConfigs = new Slot0Configs()
            .withKP(0.4)
            .withKI(0.0)
            .withKD(0.0);

    private final ClosedLoopRampsConfigs closedLoopRampsConfigs = new ClosedLoopRampsConfigs()
            .withDutyCycleClosedLoopRampPeriod(0.25);

    double targetPosition = 0;

    private final DataLog m_log;
    private final DoubleLogEntry m_pivotVelocityLog, m_pivotPositionLog;

    /** Creates a new FanPivot. */
    public FanPivot() {
        // Creates a new pivot
        motor = new TalonFX(PIVOT_CAN_ID);
        motor.getConfigurator().apply(pidConfigs);
        motor.getConfigurator().apply(closedLoopRampsConfigs);
        motor.setPosition(0.);

        // LOGGGGGGGGGGGGGGGGG DA PIVOTTTTT
        m_log = DataLogManager.getLog();
        m_pivotPositionLog = new DoubleLogEntry(m_log, "/Fan/Pivot/Position");
        m_pivotVelocityLog = new DoubleLogEntry(m_log, "/Fan/Pivot/Velocity");

        DashboardStore.add("Fan Pivot Position", () -> motor.getPosition().getValueAsDouble());
        DashboardStore.add("Fan Pivot Current", () -> motor.getStatorCurrent().getValueAsDouble());
    }

    public void logValues() {
        m_pivotPositionLog.append(motor.getPosition().getValueAsDouble());
        m_pivotVelocityLog.append(motor.getVelocity().getValueAsDouble());
    }
    
    public void runToPosition(double position) {
        targetPosition = position;
        motor.setControl(positionRequest.withPosition(position));
    }

    public Command runToPositionCommand(double position) {
        return runOnce(() -> runToPosition(position));
    }

    public Command runToTrapCommand() {
        return runToPositionCommand(TRAP_POSITION);
    }

    public void runMotor(double vbus) {
        motor.set(vbus);
    }

    public Command runMotorCommand(double vbus) {
        return startEnd(
                () -> runMotor(vbus),
                () -> runMotor(0.));
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
