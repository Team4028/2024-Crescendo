// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.DashboardStore;
import frc.robot.utils.LogStore;
import frc.robot.utils.SignalStore;
import frc.robot.utils.ShooterTable.ShooterTableEntry;

public class Shooter extends SubsystemBase {
    private final TalonFX leftMotor, rightMotor;

    private final StatusSignal<Double> rightCurrent, leftCurrent,
            rightVelocity, leftVelocity,
            rightVoltage, leftVoltage;

    private int slot = 0;

    private double rightTarget, leftTarget;

    private static final int LEFT_CAN_ID = 10;
    private static final int RIGHT_CAN_ID = 9;

    /* Configs */
    private final VelocityVoltage leftVelocityRequest = new VelocityVoltage(0.)
            .withEnableFOC(true);

    private final VelocityVoltage rightVelocityRequest = new VelocityVoltage(0.)
            .withEnableFOC(true);

    private final CurrentLimitsConfigs leftCurrentLimitsConfigs = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(120.)
            .withSupplyCurrentLimit(80.);

    private final CurrentLimitsConfigs rightCurrentLimitsConfigs = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(100.)
            .withSupplyCurrentLimit(60.);

    private final ClosedLoopRampsConfigs leftRampConfigs = new ClosedLoopRampsConfigs()
            .withVoltageClosedLoopRampPeriod(0.1);

    private final ClosedLoopRampsConfigs rightRampConfigs = new ClosedLoopRampsConfigs()
            .withVoltageClosedLoopRampPeriod(0.1);

    public enum ShotSpeeds {

        FAST(4800, 3400),
        MEDIUM(3840, 2700),
        TRAP(1370, 1370),
        AMP(677., 677.);

        public final double RightRPM;
        public final double LeftRPM;

        private ShotSpeeds(double leftRPM, double rightRPM) {
            this.LeftRPM = leftRPM;
            this.RightRPM = rightRPM;
        }
    }

    public final class Slots {
        public static final int TRAP = 2;
        public static final int AMP = 1;
        public static final int FAST = 0;
    }

    private final class PIDConstants {
        private static class Right {
            private static double kFF = 0.136;

            private static final Slot2Configs Trap = new Slot2Configs()
                    .withKP(0.02)
                    .withKV(kFF); // 1300

            private static final Slot1Configs Amp = new Slot1Configs()
                    .withKP(0.02)
                    .withKV(kFF); // 690

            private static final Slot0Configs Fast = new Slot0Configs()
                    .withKP(0.1)
                    .withKV(kFF); // 100%
        }

        private static class Left {
            private static double kFF = 0.132;

            private static final Slot2Configs Trap = new Slot2Configs()
                    .withKP(0.01)
                    .withKV(kFF); // 1300

            private static final Slot1Configs Amp = new Slot1Configs()
                    .withKP(0.01)
                    .withKV(kFF); // 690

            private static final Slot0Configs Fast = new Slot0Configs()
                    .withKP(0.05)
                    .withKV(kFF); // 100%
        }
    }

    public Shooter() {
        // ==================================
        // SHOOTER WHEELS
        // ==================================
        leftMotor = new TalonFX(LEFT_CAN_ID);
        rightMotor = new TalonFX(RIGHT_CAN_ID);

        rightCurrent = rightMotor.getStatorCurrent();
        leftCurrent = leftMotor.getStatorCurrent();

        rightVelocity = rightMotor.getVelocity();
        leftVelocity = leftMotor.getVelocity();

        rightVoltage = rightMotor.getMotorVoltage();
        leftVoltage = leftMotor.getMotorVoltage();

        leftMotor.setInverted(true);

        leftMotor.setNeutralMode(NeutralModeValue.Coast);
        rightMotor.setNeutralMode(NeutralModeValue.Coast);

        leftMotor.getConfigurator().apply(leftCurrentLimitsConfigs);
        rightMotor.getConfigurator().apply(rightCurrentLimitsConfigs);

        leftMotor.getConfigurator().apply(leftRampConfigs);
        rightMotor.getConfigurator().apply(rightRampConfigs);

        /* CAN */
        BaseStatusSignal.setUpdateFrequencyForAll(20.,
                rightCurrent, leftCurrent,
                rightVelocity, leftVelocity,
                rightVoltage, leftVoltage);

        leftMotor.optimizeBusUtilization();
        rightMotor.optimizeBusUtilization();

        SignalStore.add(rightCurrent, leftCurrent,
                rightVelocity, leftVelocity,
                rightVoltage, leftVoltage);

        // ==================================
        // SHOOTER PID
        // ==================================

        leftMotor.getConfigurator().apply(PIDConstants.Left.Trap);
        leftMotor.getConfigurator().apply(PIDConstants.Left.Amp);
        leftMotor.getConfigurator().apply(PIDConstants.Left.Fast);

        rightMotor.getConfigurator().apply(PIDConstants.Right.Trap);
        rightMotor.getConfigurator().apply(PIDConstants.Right.Amp);
        rightMotor.getConfigurator().apply(PIDConstants.Right.Fast);

        // ==================================
        // LOGS
        // ==================================
        LogStore.add("/Shooter/Right/Current", rightCurrent::getValueAsDouble);
        LogStore.add("/Shooter/Left/Current", leftCurrent::getValueAsDouble);
        LogStore.add("/Shooter/Right/Velocity", rightVelocity::getValueAsDouble);
        LogStore.add("/Shooter/Left/Velocity", leftVelocity::getValueAsDouble);
        LogStore.add("/Shooter/Right/Voltage", rightVoltage::getValueAsDouble);
        LogStore.add("/Shooter/Left/Voltage", leftVoltage::getValueAsDouble);

        /* Dashboard */
        DashboardStore.add("Left RPM", () -> leftVelocity.getValueAsDouble() * 60.);
        DashboardStore.add("Right RPM", () -> rightVelocity.getValueAsDouble() * 60.);

        DashboardStore.add("Left Target", () -> leftTarget);
        DashboardStore.add("Right Target", () -> rightTarget);

        DashboardStore.add("Running/Shooter", this::isRunning);
        DashboardStore.add("At Target/Shooter", this::isReady);
    }

    // ==================================
    // SHOOTER COMMANDS
    // ==================================

    /* Check if shooter is running */
    public BooleanSupplier isRunningSupplier() {
        return this::isRunning;
    }

    public boolean isRunning() {
        return leftMotor.getAppliedControl().getName() == "VelocityVoltage" &&
                rightMotor.getAppliedControl().getName() == "VelocityVoltage";
    }

    /* Check if shooter is spinned up */
    public BooleanSupplier isReadySupplier() {
        return () -> isReady();
    }

    public boolean isReady() {
        return Math.abs(leftMotor.getVelocity().getValueAsDouble() * 60. - leftTarget) < 130.
                && Math.abs(rightMotor.getVelocity().getValueAsDouble() * 60. - rightTarget) < 130.;
    }

    public BooleanSupplier isWorkingSupplier() {
        return this::isWorking;
    }

    private boolean isWorking() {
        return leftMotor.isAlive() && rightMotor.isAlive();
    }

    /**
     * Run motors at the velocity input from the dashboard.
     * 
     * @return A {@link Command} that runs both motors at their desired input
     *         velocities.
     */
    public Command runVelocityCommand() {
        return startEnd(
                () -> {
                    setLeftToVel(leftTarget);
                    setRightToVel(rightTarget);
                },
                () -> stop());
    }

    /* Run Based on Shooter Table Entry */
    public void runEntry(ShooterTableEntry entry, ShotSpeeds shotSpeed) {
        setLeftToVel(entry.Percent * shotSpeed.LeftRPM);
        setRightToVel(entry.Percent * shotSpeed.RightRPM);
    }

    public Command runEntryCommand(Supplier<ShooterTableEntry> entry, Supplier<ShotSpeeds> shotSpeed) {
        return runOnce(() -> runEntry(entry.get(), shotSpeed.get()));
    }

    public void stop() {
        leftMotor.stopMotor();
        rightMotor.stopMotor();
    }

    public Command stopCommand() {
        return runOnce(this::stop);
    }

    public Command brakeStopCommand() {
        return runShotCommand(ShotSpeeds.AMP, 0.0).andThen(
                Commands.waitSeconds(0.2),
                stopCommand());
    }

    public void setSlot(int slot) {
        this.slot = MathUtil.clamp(slot, 0, 3);
    }

    public Command setSlotCommand(int slot) {
        return runOnce(() -> setSlot(slot));
    }

    // =============================
    // SHOTS
    // =============================

    public void runShot(ShotSpeeds shot, double scale) {
        setLeftToVel(shot.LeftRPM * scale);
        setRightToVel(shot.RightRPM * scale);
    }

    public Command runShotCommand(ShotSpeeds shot, double scale) {
        return runOnce(() -> runShot(shot, scale));
    }

    public Command runShotCommand(ShotSpeeds shot) {
        return runShotCommand(shot, 1.0);
    }

    public void setLeftToVel(double velRPM) {
        leftTarget = velRPM;

        leftMotor.setControl(leftVelocityRequest
                .withVelocity(velRPM / 60.)
                .withSlot(slot));
    }

    public void setRightToVel(double velRPM) {
        rightTarget = velRPM;

        rightMotor.setControl(rightVelocityRequest
                .withVelocity(velRPM / 60.)
                .withSlot(slot));
    }

    public void spinMotorLeft(double vBus) {
        leftMotor.set(vBus);
    }

    public void spinMotorRight(double vBus) {
        rightMotor.set(vBus);
    }

    public Command spinMotorLeftCommand(double vBus) {
        return runOnce(() -> spinMotorLeft(vBus));
    }

    public Command spinMotorRightCommand(double vBus) {
        return runOnce(() -> spinMotorRight(vBus));
    }

    public Command spinBothCommand(double vBus) {
        return spinMotorRightCommand(vBus).andThen(spinMotorLeftCommand(vBus));
    }

    @Override
    public void periodic() {

    }
}
