// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.DashboardStore;
import frc.robot.utils.LogStore;
import frc.robot.utils.SignalStore;

public class FanPivot extends SubsystemBase {
    private final TalonFX motor;

    private final StatusSignal<Double> position, current;

    private static final int CAN_ID = 17;

    public static final double TRAP_POSITION = 0.57;

    /* Requests */
    private final PositionDutyCycle positionRequest = new PositionDutyCycle(0.).withOverrideBrakeDurNeutral(true);

    private final Slot0Configs trapConfigs = new Slot0Configs().withKP(0.4).withKI(0.0).withKD(0.0);

    private final Slot1Configs holdConfigs = new Slot1Configs().withKP(0.8);

    private final ClosedLoopRampsConfigs closedLoopRampsConfigs = new ClosedLoopRampsConfigs()
            .withDutyCycleClosedLoopRampPeriod(0.25);

    double targetPosition = 0;

    /** Creates a new FanPivot. */
    public FanPivot() {
        /* Setup */
        motor = new TalonFX(CAN_ID);
        motor.setNeutralMode(NeutralModeValue.Brake);

        position = motor.getPosition();
        current = motor.getStatorCurrent();

        motor.getConfigurator().apply(trapConfigs);
        motor.getConfigurator().apply(holdConfigs);
        motor.getConfigurator().apply(closedLoopRampsConfigs);
        motor.setPosition(0.);

        /* CAN Bus */
        BaseStatusSignal.setUpdateFrequencyForAll(20.0, current, position);
        motor.optimizeBusUtilization();

        SignalStore.add(current, position);

        /* Logs */
        LogStore.add("/Fan/Pivot/Position", position::getValueAsDouble);
        LogStore.add("/Fan/Pivot/Current", current::getValueAsDouble);

        DashboardStore.add("Fan Pivot Position", position::getValueAsDouble);
    }

    public void runToTrap() {
        targetPosition = TRAP_POSITION;
        motor.setControl(positionRequest.withPosition(targetPosition).withSlot(0));
    }

    public Command runToTrapCommand() {
        return runOnce(this::runToTrap);
    }

    public void hold() {
        targetPosition = 0.0;
        motor.setControl(positionRequest.withPosition(targetPosition).withSlot(1));
    }

    public Command runToHomeCommand() {
        return runOnce(this::hold);
    }

    public void runMotor(double vbus) {
        motor.set(vbus);
    }

    public Command runMotorCommand(double vbus) {
        return startEnd(() -> runMotor(vbus), () -> runMotor(0.));
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
