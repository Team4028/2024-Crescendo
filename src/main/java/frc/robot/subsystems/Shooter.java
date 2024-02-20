// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    private CANSparkFlex rightMotor, leftMotor;
    private CANSparkMax pivotMotor;
    private RelativeEncoder rightEncoder, leftEncoder, pivotEncoder;
    private SparkPIDController rightPid, leftPid, pivotPid;

    private final DataLog log;
    private final DoubleLogEntry rightCurrent, leftCurrent,
            rightVelocity, leftVelocity, leftVoltage,
            rightVoltage, pivotCurrent, pivotVelocity, pivotVoltage;

    private int slot = 0;

    private double leftTarget, rightTarget;

    private final Timer zeroTimer;

    private final double ZERO_TIMER_THRESHOLD = 0.06; // 3 scans
    private final double ZERO_VELOCITY_THRESHOLD = 0.2;

    private static final int RIGHT_CAN_ID = 9;
    private static final int LEFT_CAN_ID = 10;
    private static final int PIVOT_CAN_ID = 13;

    private final class Slots {
        private static final int TRAP = 3;
        private static final int SHORT = 2;
        private static final int MEDIUM = 1;
        private static final int LONG = 0;
    }

    private static class PIDVFConstants {
        public final double kP;
        public final double kI;
        public final double kD;
        public final double kFF;
        public final double Velocity;

        public PIDVFConstants(double kP, double kI, double kD, double kFF, double velocity) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
            this.kFF = kFF;
            this.Velocity = velocity;
        }

        public PIDVFConstants(double kP, double kFF, double velocity) {
            this(kP, 0., 0., kFF, velocity);
        }
    }

    private final class PIDConstants {
        private static final double MAX_LEFT = 2500;
        private static final double MAX_RIGHT = 3400;

        private static class Left {
            private static double kFF = 0.00019;

            private static final PIDVFConstants Trap = new PIDVFConstants(0.0002, kFF, 1300);
            private static final PIDVFConstants Long = new PIDVFConstants(0.001, kFF, MAX_LEFT);
            private static final PIDVFConstants Medium = new PIDVFConstants(0.001, kFF, MAX_LEFT * 0.8);
            private static final PIDVFConstants Short = new PIDVFConstants(0.000125, kFF, 1000.);
        }

        private static class Right {
            private static double kFF = 0.00022;

            private static final PIDVFConstants Trap = new PIDVFConstants(0.001, kFF, 1300);
            private static final PIDVFConstants Long = new PIDVFConstants(0.002, kFF, MAX_RIGHT);
            private static final PIDVFConstants Medium = new PIDVFConstants(0.002, kFF, MAX_RIGHT * 0.8);
            private static final PIDVFConstants Short = new PIDVFConstants(0.0000625, kFF, 100.);
        }

        private static class Pivot {
            private static double kFF = 0.0;

            private static final PIDVFConstants PID = new PIDVFConstants(0.16, kFF, 0.);

            private static double LONG_POSITION = 2.5;
            private static double MEDIUM_POSITION = 26.75;
            private static double SHORT_POSITION = 45.;

            private static final Map<Integer, Double> POSITION_MAP = Map.of(
                    Slots.LONG, LONG_POSITION,
                    Slots.MEDIUM, MEDIUM_POSITION,
                    Slots.SHORT, SHORT_POSITION,
                    Slots.TRAP, SHORT_POSITION);

            private static double MIN_VAL = 0.;
            private static double MAX_VAL = 53.0;
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
        slot = Slots.SHORT;

        configPid(rightPid, slot, PIDConstants.Right.Short);
        configPid(leftPid, slot, PIDConstants.Left.Short);

        // MEDIUM //
        slot = Slots.MEDIUM;

        configPid(rightPid, slot, PIDConstants.Right.Medium);
        configPid(leftPid, slot, PIDConstants.Left.Medium);

        // SHORT //
        slot = Slots.SHORT;

        configPid(rightPid, slot, PIDConstants.Right.Long);
        configPid(leftPid, slot, PIDConstants.Left.Long);

        longMode();

        leftMotor.burnFlash();
        rightMotor.burnFlash();

        // ==================================
        // PIVOT
        // ==================================
        pivotMotor = new CANSparkMax(PIVOT_CAN_ID, MotorType.kBrushless);

        pivotMotor.setInverted(true);
        pivotMotor.setIdleMode(IdleMode.kBrake);

        pivotEncoder = pivotMotor.getEncoder();
        pivotPid = pivotMotor.getPIDController();

        // ==================================
        // PIVOT PID
        // ==================================
        configPid(pivotPid, 0, PIDConstants.Pivot.PID);

        pivotMotor.burnFlash();

        // ==================================
        // TIMER
        // ==================================
        zeroTimer = new Timer();

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
        pivotCurrent = new DoubleLogEntry(log, "/Pivot/Current");
        pivotVoltage = new DoubleLogEntry(log, "/Pivot/Voltage");
        pivotVelocity = new DoubleLogEntry(log, "/Pivot/Velocity");

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
    // PIVOT COMMANDS
    // ==================================

    public void runPivotMotor(double vBus) {
        pivotMotor.set(vBus);
    }

    public Command runPivotCommand(double vBus) {
        return runOnce(() -> runPivotMotor(vBus));
    }

    public void runPivotToPosition(double position) {
        pivotPid.setReference(MathUtil.clamp(position, PIDConstants.Pivot.MIN_VAL, PIDConstants.Pivot.MAX_VAL),
                ControlType.kPosition);
    }

    public Command pivotZeroCommand() {
        return runOnce(() -> {
            zeroTimer.restart();
        })
                .andThen(runPivotCommand(-0.1).repeatedly()
                        .until(() -> zeroTimer.get() >= ZERO_TIMER_THRESHOLD
                                && Math.abs(pivotEncoder.getVelocity()) < ZERO_VELOCITY_THRESHOLD))
                .andThen(runPivotCommand(0.).alongWith(Commands.runOnce(() -> zeroTimer.stop())))
                .andThen(runOnce(() -> pivotEncoder.setPosition(0.)));
    }

    public double getPivotPosition() {
        return PIDConstants.Pivot.POSITION_MAP.getOrDefault(slot, PIDConstants.Pivot.MEDIUM_POSITION);
    }

    // ==================================
    // SHOOTER COMMANDS
    // ==================================

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
            case Slots.SHORT:
                shortMode();
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

    public Command setSlotComand(int slot) {
        return runOnce(() -> setSlot(slot));
    }

    // =============================
    // MODES
    // =============================

    private void putConstants(PIDVFConstants right, PIDVFConstants left, String modeString) {
        SmartDashboard.putNumber("Left P Gain", right.kP);
        SmartDashboard.putNumber("Left I Gain", right.kI);
        SmartDashboard.putNumber("Left D Gain", right.kD);
        SmartDashboard.putNumber("Left Feed Forward", left.kFF);

        SmartDashboard.putNumber("Right P Gain", right.kP);
        SmartDashboard.putNumber("Right I Gain", right.kI);
        SmartDashboard.putNumber("Right D Gain", right.kD);
        SmartDashboard.putNumber("Right Feed Forward", right.kFF);

        SmartDashboard.putNumber("Left Velocity", right.Velocity);
        SmartDashboard.putNumber("Right Velocity", right.Velocity);

        SmartDashboard.putString("Shooter Mode", modeString);

        leftTarget = right.Velocity;
        rightTarget = right.Velocity;

    }

    public void trapMode() {
        putConstants(PIDConstants.Right.Trap, PIDConstants.Left.Trap, "Trap");
    }

    public void longMode() {
        putConstants(PIDConstants.Right.Long, PIDConstants.Left.Long, "Long");
    }

    public void mediumMode() {
        putConstants(PIDConstants.Right.Medium, PIDConstants.Left.Medium, "Medium");
    }

    public void shortMode() {
        putConstants(PIDConstants.Right.Short, PIDConstants.Left.Short, "Short");
    }

    public void setRightToVel(double velRPM) {
        rightPid.setReference(velRPM, ControlType.kVelocity);
    }

    public void setLeftToVel(double velRPM) {
        leftPid.setReference(velRPM, ControlType.kVelocity);
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

    public void logValues() {
        rightCurrent.append(rightMotor.getOutputCurrent());
        leftCurrent.append(leftMotor.getOutputCurrent());

        rightVelocity.append(rightMotor.getEncoder().getVelocity());
        leftVelocity.append(leftMotor.getEncoder().getVelocity());

        rightVoltage.append(rightMotor.getAppliedOutput() *
                rightMotor.getBusVoltage());
        leftVoltage.append(leftMotor.getAppliedOutput() *
                leftMotor.getBusVoltage());

        pivotCurrent.append(pivotMotor.getOutputCurrent());
        pivotVoltage.append(pivotMotor.getBusVoltage() * pivotMotor.getAppliedOutput());
        pivotVelocity.append(pivotEncoder.getVelocity());
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Pivot pos", pivotEncoder.getPosition());
    }
}
