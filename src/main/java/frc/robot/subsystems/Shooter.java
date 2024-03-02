// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.revrobotics.CANSparkFlex;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.ShooterTable.ShooterTableEntry;

public class Shooter extends SubsystemBase {
    private final CANSparkFlex leftMotor, rightMotor;

    private final RelativeEncoder leftEncoder, rightEncoder;
    private final SparkPIDController leftPid, rightPid;

    private final DataLog log;
    private final DoubleLogEntry rightCurrent, leftCurrent,
            rightVelocity, leftVelocity, leftVoltage,
            rightVoltage;
    private int slot = 0;

    private double rightTarget, leftTarget;

    private static final int LEFT_CAN_ID = 9;
    private static final int RIGHT_CAN_ID = 10;

    public enum ShotSpeeds {

        FAST(3400, 2500),
        MEDIUM(2700, 2000),
        TRAP(1300, 1300),
        AMP(677., 677.);

        public final double RightRPM;
        public final double LeftRPM;

        private ShotSpeeds(double leftRPM, double rightRPM) {
            this.LeftRPM = leftRPM;
            this.RightRPM = rightRPM;
        }
    }

    public final class Slots {
        public static final int TRAP = 3;
        public static final int AMP = 2;
        public static final int MEDIUM = 1;
        public static final int LONG = 0;
    }

    private static class PIDVFConstants {
        public final double kP;
        public final double kI;
        public final double kD;
        public final double kFF;

        public PIDVFConstants(double kP, double kI, double kD, double kFF) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
            this.kFF = kFF;
        }

        public PIDVFConstants(double kP, double kFF) {
            this(kP, 0., 0., kFF);
        }
    }

    private final class PIDConstants {
        private static class Right {
            private static double kFF = 0.00019;

            private static final PIDVFConstants Trap = new PIDVFConstants(0.0002, kFF); // 1300
            private static final PIDVFConstants Long = new PIDVFConstants(0.001, kFF); // 100%
            private static final PIDVFConstants Medium = new PIDVFConstants(0.001, kFF); // 80%
            private static final PIDVFConstants Amp = new PIDVFConstants(0.00025, kFF); // 690
        }

        private static class Left {
            private static double kFF = 0.00022;

            private static final PIDVFConstants Trap = new PIDVFConstants(0.001, kFF); // 1300
            private static final PIDVFConstants Long = new PIDVFConstants(0.002, kFF); // 100%
            private static final PIDVFConstants Medium = new PIDVFConstants(0.002, kFF); // 80%
            private static final PIDVFConstants Amp = new PIDVFConstants(0.0005, kFF); // 690
        }
    }

    public Shooter() {
        // ==================================
        // SHOOTER WHEELS
        // ==================================
        leftMotor = new CANSparkFlex(LEFT_CAN_ID, MotorType.kBrushless);
        rightMotor = new CANSparkFlex(RIGHT_CAN_ID, MotorType.kBrushless);

        leftMotor.setInverted(false);

        leftMotor.setIdleMode(IdleMode.kBrake);
        rightMotor.setIdleMode(IdleMode.kBrake);

        leftEncoder = leftMotor.getEncoder();
        rightEncoder = rightMotor.getEncoder();

        leftMotor.setSmartCurrentLimit(80);
        rightMotor.setSmartCurrentLimit(60);

        leftEncoder.setMeasurementPeriod(16);
        leftEncoder.setAverageDepth(2);

        rightEncoder.setMeasurementPeriod(16);
        rightEncoder.setAverageDepth(2);

        leftMotor.setClosedLoopRampRate(0.1);
        rightMotor.setClosedLoopRampRate(0.1);

        leftPid = leftMotor.getPIDController();
        rightPid = rightMotor.getPIDController();

        leftPid.setFeedbackDevice(leftEncoder);
        rightPid.setFeedbackDevice(rightEncoder);

        leftMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20);
        leftMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
        leftMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);
        leftMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 101);
        leftMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 102);
        leftMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 103);
        leftMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 104);
        leftMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus7, 106);
        rightMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20);
        rightMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
        rightMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);
        rightMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 101);
        rightMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 102);
        rightMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 103);
        rightMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 104);
        rightMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus7, 106);

        // ==================================
        // SHOOTER PID
        // ==================================
        leftPid.setOutputRange(-1, 1);
        rightPid.setOutputRange(-1, 1);

        // TRAP //
        int slot = Slots.TRAP;

        configPid(leftPid, slot, PIDConstants.Left.Trap);
        configPid(rightPid, slot, PIDConstants.Right.Trap);

        // SHORT //
        slot = Slots.AMP;

        configPid(leftPid, slot, PIDConstants.Left.Amp);
        configPid(rightPid, slot, PIDConstants.Right.Amp);

        // MEDIUM //
        slot = Slots.MEDIUM;

        configPid(leftPid, slot, PIDConstants.Left.Medium);
        configPid(rightPid, slot, PIDConstants.Right.Medium);

        // SHORT //
        slot = Slots.LONG;

        configPid(leftPid, slot, PIDConstants.Left.Long);
        configPid(rightPid, slot, PIDConstants.Right.Long);

        longMode();

        // leftMotor.burnFlash();
        // rightMotor.burnFlash();

        // ==================================
        // LOGS
        // ==================================
        log = DataLogManager.getLog();
        rightCurrent = new DoubleLogEntry(log, "/Shooter/right/Current");
        leftCurrent = new DoubleLogEntry(log, "/Shooter/left/Current");
        rightVelocity = new DoubleLogEntry(log, "/Shooter/right/Velocity");
        leftVelocity = new DoubleLogEntry(log, "/Shooter/left/Velocity");
        rightVoltage = new DoubleLogEntry(log, "/Shooter/right/Voltage");
        leftVoltage = new DoubleLogEntry(log, "/Shooter/left/Voltage");

    }

    // ==================================
    // FUNNY PID THING
    // ==================================
    private void configPid(SparkPIDController controller, int slot, PIDVFConstants constants) {
        controller.setP(constants.kP, slot);
        controller.setI(constants.kI, slot);
        controller.setD(constants.kD, slot);
        controller.setFF(constants.kFF, slot);
    }

    // ==================================
    // SHOOTER COMMANDS
    // ==================================

    /* Check if shooter is spinned up */
    public BooleanSupplier isReady() {
        return () -> Math.abs(leftEncoder.getVelocity() - leftTarget) < 20.
                && Math.abs(rightEncoder.getVelocity() - rightTarget) < 20.;
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
        setLeftToVel(entry.percent * shotSpeed.LeftRPM);
        setRightToVel(entry.percent * shotSpeed.RightRPM);
    }

    public Command runEntryCommand(Supplier<ShooterTableEntry> entry, Supplier<ShotSpeeds> shotSpeed) {
        return startEnd(
                () -> runEntry(entry.get(), shotSpeed.get()),
                () -> stop());
    }

    public void stop() {
        leftMotor.stopMotor();
        rightMotor.stopMotor();
    }

    public Command stopCommand() {
        return runOnce(this::stop);
    }

    public void setSlot(int slot) {
        this.slot = MathUtil.clamp(slot, 0, 3);

        switch (slot) {
            case Slots.TRAP:
                trapMode();
                break;
            case Slots.AMP:
                ampMode();
                break;
            case Slots.MEDIUM:
                mediumMode();
                break;
            case Slots.LONG:
                longMode();
                break;
            default:
                break;
        }
    }

    public Command setSlotCommand(int slot) {
        return runOnce(() -> setSlot(slot));
    }

    // =============================
    // MODES
    // =============================

    private void putConstants(PIDVFConstants left, PIDVFConstants right, ShotSpeeds speeds, String modeString) {
        SmartDashboard.putNumber("Left P Gain", left.kP);
        SmartDashboard.putNumber("Left I Gain", left.kI);
        SmartDashboard.putNumber("Left D Gain", left.kD);
        SmartDashboard.putNumber("Left Feed Forward", left.kFF);

        SmartDashboard.putNumber("Right P Gain", right.kP);
        SmartDashboard.putNumber("Right I Gain", right.kI);
        SmartDashboard.putNumber("Right D Gain", right.kD);
        SmartDashboard.putNumber("Right Feed Forward", right.kFF);

        SmartDashboard.putNumber("Left Velocity Target", speeds.LeftRPM);
        SmartDashboard.putNumber("Right Velocity Target", speeds.RightRPM);

        SmartDashboard.putString("Shooter Mode", modeString);

        rightTarget = speeds.RightRPM;
        leftTarget = speeds.LeftRPM;
    }

    private void trapMode() {
        putConstants(PIDConstants.Left.Trap, PIDConstants.Right.Trap, ShotSpeeds.TRAP, "Trap");
    }

    private void longMode() {
        putConstants(PIDConstants.Left.Long, PIDConstants.Right.Long, ShotSpeeds.FAST, "Long");
    }

    private void mediumMode() {
        putConstants(PIDConstants.Left.Medium, PIDConstants.Right.Medium, ShotSpeeds.MEDIUM, "Medium");
    }

    private void ampMode() {
        putConstants(PIDConstants.Left.Amp, PIDConstants.Right.Amp, ShotSpeeds.AMP, "Amp");
    }

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
        leftPid.setReference(velRPM, ControlType.kVelocity, slot);
    }

    public void setRightToVel(double velRPM) {
        rightTarget = velRPM;
        rightPid.setReference(velRPM, ControlType.kVelocity, slot);
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

    public void logValues() {
        leftCurrent.append(leftMotor.getOutputCurrent());
        rightCurrent.append(rightMotor.getOutputCurrent());

        leftVelocity.append(leftMotor.getEncoder().getVelocity());
        rightVelocity.append(rightMotor.getEncoder().getVelocity());

        leftVoltage.append(leftMotor.getAppliedOutput() *
                leftMotor.getBusVoltage());
        rightVoltage.append(rightMotor.getAppliedOutput() *
                rightMotor.getBusVoltage());

    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Left Shooter Speed", leftEncoder.getVelocity());
        SmartDashboard.putNumber("Right Shooter Speed", rightEncoder.getVelocity());

        SmartDashboard.putNumber("Left Shooter Target", leftTarget);
        SmartDashboard.putNumber("Right Shooter Target", rightTarget);
    }
}
