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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.ShooterTable.ShooterTableEntry;

public class Shooter extends SubsystemBase {
    private CANSparkFlex rightMotor, leftMotor;

    private RelativeEncoder rightEncoder, leftEncoder;
    private SparkPIDController rightPid, leftPid;

    private final DataLog log;
    private final DoubleLogEntry rightCurrent, leftCurrent,
            rightVelocity, leftVelocity, leftVoltage,
            rightVoltage;
    private int slot = 0;

    private double leftTarget, rightTarget;

    private static final int RIGHT_CAN_ID = 9;
    private static final int LEFT_CAN_ID = 10;

    public enum ShotSpeeds {

        FAST(3400, 2500),
        MEDIUM(2700, 2000),
        TRAP(1300, 1300),
        AMP(677., 677.);

        public final double LeftRPM;
        public final double RightRPM;

        private ShotSpeeds(double leftRPM, double rightRPM) {
            this.LeftRPM = leftRPM;
            this.RightRPM = rightRPM;
        }
    }

    private final class Slots {
        private static final int TRAP = 3;
        private static final int AMP = 2;
        private static final int MEDIUM = 1;
        private static final int LONG = 0;
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
        private static class Left {
            private static double kFF = 0.00019;

            private static final PIDVFConstants Trap = new PIDVFConstants(0.0002, kFF); // 1300
            private static final PIDVFConstants Long = new PIDVFConstants(0.001, kFF); // 100%
            private static final PIDVFConstants Medium = new PIDVFConstants(0.001, kFF); // 80%
            private static final PIDVFConstants Amp = new PIDVFConstants(0.00025, kFF); // 690
        }

        private static class Right {
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
        rightMotor = new CANSparkFlex(RIGHT_CAN_ID, MotorType.kBrushless);
        leftMotor = new CANSparkFlex(LEFT_CAN_ID, MotorType.kBrushless);

        rightMotor.setInverted(false);

        rightMotor.setIdleMode(IdleMode.kBrake);
        leftMotor.setIdleMode(IdleMode.kBrake);

        rightEncoder = rightMotor.getEncoder();
        leftEncoder = leftMotor.getEncoder();

        rightMotor.setSmartCurrentLimit(80);
        leftMotor.setSmartCurrentLimit(60);

        rightEncoder.setMeasurementPeriod(16);
        rightEncoder.setAverageDepth(2);

        leftEncoder.setMeasurementPeriod(16);
        leftEncoder.setAverageDepth(2);

        rightMotor.setClosedLoopRampRate(0.1);
        leftMotor.setClosedLoopRampRate(0.1);

        rightPid = rightMotor.getPIDController();
        leftPid = leftMotor.getPIDController();

        rightPid.setFeedbackDevice(rightEncoder);
        leftPid.setFeedbackDevice(leftEncoder);

        // ==================================
        // SHOOTER PID
        // ==================================
        rightPid.setOutputRange(-1, 1);
        leftPid.setOutputRange(-1, 1);

        // TRAP //
        int slot = Slots.TRAP;

        configPid(rightPid, slot, PIDConstants.Right.Trap);
        configPid(leftPid, slot, PIDConstants.Left.Trap);

        // SHORT //
        slot = Slots.AMP;

        configPid(rightPid, slot, PIDConstants.Right.Amp);
        configPid(leftPid, slot, PIDConstants.Left.Amp);

        // MEDIUM //
        slot = Slots.MEDIUM;

        configPid(rightPid, slot, PIDConstants.Right.Medium);
        configPid(leftPid, slot, PIDConstants.Left.Medium);

        // SHORT //
        slot = Slots.LONG;

        configPid(rightPid, slot, PIDConstants.Right.Long);
        configPid(leftPid, slot, PIDConstants.Left.Long);

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
        return () -> Math.abs(rightEncoder.getVelocity() - rightTarget) < 20.
                && Math.abs(leftEncoder.getVelocity() - leftTarget) < 20.;
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
                    setRightToVel(rightTarget);
                    setLeftToVel(leftTarget);
                },
                () -> stop());
    }

    /* Run Based on Shooter Table Entry */
    public void runEntry(ShooterTableEntry entry, ShotSpeeds shotSpeed) {
        setRightToVel(entry.percent * shotSpeed.RightRPM);
        setLeftToVel(entry.percent * shotSpeed.LeftRPM);
    }

    public Command runEntryCommand(Supplier<ShooterTableEntry> entry, Supplier<ShotSpeeds> shotSpeed) {
        return startEnd(
                () -> runEntry(entry.get(), shotSpeed.get()),
                () -> stop());
    }

    public void stop() {
        rightMotor.stopMotor();
        leftMotor.stopMotor();
    }

    public Command stopCommand() {
        return runOnce(this::stop);
    }

    public Command cycleUpCommand() {
        return runOnce(() -> setSlot(slot + 1));
    }

    public Command cycleDownCommand() {
        return runOnce(() -> setSlot(slot - 1));
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

    private void putConstants(PIDVFConstants right, PIDVFConstants left, ShotSpeeds speeds, String modeString) {
        SmartDashboard.putNumber("Left P Gain", left.kP);
        SmartDashboard.putNumber("Left I Gain", left.kI);
        SmartDashboard.putNumber("Left D Gain", left.kD);
        SmartDashboard.putNumber("Left Feed Forward", left.kFF);

        SmartDashboard.putNumber("Right P Gain", right.kP);
        SmartDashboard.putNumber("Right I Gain", right.kI);
        SmartDashboard.putNumber("Right D Gain", right.kD);
        SmartDashboard.putNumber("Right Feed Forward", right.kFF);

        SmartDashboard.putNumber("Left Velocity", speeds.LeftRPM);
        SmartDashboard.putNumber("Right Velocity", speeds.RightRPM);

        SmartDashboard.putString("Shooter Mode", modeString);

        leftTarget = speeds.LeftRPM;
        rightTarget = speeds.RightRPM;
    }

    private void trapMode() {
        putConstants(PIDConstants.Right.Trap, PIDConstants.Left.Trap, ShotSpeeds.TRAP, "Trap");
    }

    private void longMode() {
        putConstants(PIDConstants.Right.Long, PIDConstants.Left.Long, ShotSpeeds.FAST, "Long");
    }

    private void mediumMode() {
        putConstants(PIDConstants.Right.Medium, PIDConstants.Left.Medium, ShotSpeeds.MEDIUM, "Medium");
    }

    private void ampMode() {
        putConstants(PIDConstants.Right.Amp, PIDConstants.Left.Amp, ShotSpeeds.AMP, "Amp");
    }

    public void setRightToVel(double velRPM) {
        rightTarget = velRPM;
        rightPid.setReference(velRPM, ControlType.kVelocity, slot);
    }

    public void setLeftToVel(double velRPM) {
        leftTarget = velRPM;
        leftPid.setReference(velRPM, ControlType.kVelocity, slot);
    }

    public void spinMotorRight(double vBus) {
        rightMotor.set(vBus);
    }

    public void spinMotorLeft(double vBus) {
        leftMotor.set(vBus);
    }

    public Command spinMotorRightCommand(double vBus) {
        return runOnce(() -> spinMotorRight(vBus));
    }

    public Command spinMotorLeftCommand(double vBus) {
        return runOnce(() -> spinMotorLeft(vBus));
    }

    public Command spinBothCommand(double vBus) {
        return spinMotorLeftCommand(vBus).andThen(spinMotorRightCommand(vBus));
    }

    public void logValues() {
        rightCurrent.append(rightMotor.getOutputCurrent());
        leftCurrent.append(leftMotor.getOutputCurrent());

        rightVelocity.append(rightMotor.getEncoder().getVelocity());
        leftVelocity.append(leftMotor.getEncoder().getVelocity());

        rightVoltage.append(rightMotor.getAppliedOutput() *
                rightMotor.getBusVoltage());
        leftVoltage.append(leftMotor.getAppliedOutput() *
                leftMotor.getBusVoltage());

    }

    @Override
    public void periodic() {
    }
}
