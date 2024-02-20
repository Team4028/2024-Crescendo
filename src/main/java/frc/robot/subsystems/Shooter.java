// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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

    private final class PIDConstants {
        private static final double MAX_LEFT = 2500;
        private static final double MAX_RIGHT = 3400;

        private static class Left {
            private static double kFF = 0.00019;

            private final class Trap {
                private static double velocity = 1300;
                private static double kP = 0.0002;
                private static double kI = 0.0;
                private static double kD = 0.0;
            }

            private final class Long {
                private static double velocity = MAX_LEFT * 1.0;;
                private static double kP = 0.001;
                private static double kI = 0.0;
                private static double kD = 0.0;
            }

            private final class Medium {
                private static double velocity = MAX_LEFT * 0.8;
                private static double kP = 0.001;
                private static double kI = 0.0;
                private static double kD = 0.0;
            }

            private final class Short {
                private static double velocity = 1000;
                private static double kP = 0.000125;
                private static double kI = 0.0;
                private static double kD = 0.0;
            }
        }

        private static class Right {
            private static double kFF = 0.00022;

            private final class Trap {
                private static double velocity = 1300;
                private static double kP = 0.001;
                private static double kI = 0.0;
                private static double kD = 0.0;
            }

            private final class Long {
                private static double velocity = MAX_RIGHT * 1.0;
                private static double kP = 0.002;
                private static double kI = 0.0;
                private static double kD = 0.0;
            }

            private final class Medium {
                private static double velocity = MAX_RIGHT * 0.8;
                private static double kP = 0.002;
                private static double kI = 0.0;
                private static double kD = 0.0;
            }

            private final class Short {
                private static double velocity = 100;
                private static double kP = 0.0000625;
                private static double kI = 0.0;
                private static double kD = 0.0;
            }
        }

        private static class Pivot {
            private static double kFF = 0.0;

            private static double kP = 0.16;
            private static double kI = 0.0;
            private static double kD = 0.0;

            //@formatter:off
            private static double LONG_POSITION = 2.5; // was 10.            ===================================================
            private static double MEDIUM_POSITION = 26.75; // was 107.     ||  **DIVIDED BY 4 BC TOOK A STAGE OFF OF MPLANETARY ||
                                                           //              ||    (16:1 -> 4:1)**                                ||
            private static double SHORT_POSITION = 45.; // was 108.          ===================================================
            //@formatter:on

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

        configPid(rightPid, slot, PIDConstants.Right.Trap.class, PIDConstants.Right.kFF);
        configPid(leftPid, slot, PIDConstants.Left.Trap.class, PIDConstants.Left.kFF);

        // SHORT //
        slot = Slots.SHORT;

        configPid(rightPid, slot, PIDConstants.Right.Short.class, PIDConstants.Right.kFF);
        configPid(leftPid, slot, PIDConstants.Left.Short.class, PIDConstants.Left.kFF);

        // MEDIUM //
        slot = Slots.MEDIUM;

        configPid(rightPid, slot, PIDConstants.Right.Medium.class, PIDConstants.Right.kFF);
        configPid(leftPid, slot, PIDConstants.Left.Medium.class, PIDConstants.Left.kFF);

        // SHORT //
        slot = Slots.SHORT;

        configPid(rightPid, slot, PIDConstants.Right.Long.class, PIDConstants.Right.kFF);
        configPid(leftPid, slot, PIDConstants.Left.Long.class, PIDConstants.Left.kFF);

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
        configPid(pivotPid, 0, PIDConstants.Pivot.class, PIDConstants.Pivot.kFF);

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
    private void configPid(SparkPIDController controller, int slot, Class constants, double kFF) {
        try {
            controller.setP(constants.getField("kP").getDouble(constants), slot);
            controller.setI(constants.getField("kI").getDouble(constants), slot);
            controller.setD(constants.getField("kD").getDouble(constants), slot);
            controller.setFF(kFF, slot);
        } catch (Throwable e) {
            System.err.println(e.getMessage());
        }
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
        switch (slot) {
            case Slots.TRAP:
            case Slots.SHORT:
                return PIDConstants.Pivot.SHORT_POSITION;
            case Slots.MEDIUM:
                return PIDConstants.Pivot.MEDIUM_POSITION;
            case Slots.LONG:
                return PIDConstants.Pivot.LONG_POSITION;
            default:
                return PIDConstants.Pivot.MEDIUM_POSITION;
        }
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
                    // runPivotToPosition(getPivotPosition());
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

    public void setMode() {
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

    public Command cycleUpCommand() {
        return runOnce(() -> {
            slot += 1;

            if (slot > 3)
                slot = 3;

            setMode();
        });
    }

    public Command cycleDownCommand() {
        return runOnce(() -> {
            slot -= 1;

            if (slot < 0)
                slot = 0;

            setMode();
        });
    }

    public Command setSlot(int slot) {
        return runOnce(() -> {
            this.slot = MathUtil.clamp(slot, 0, 3);
            setMode();
        });
    }

    public void trapMode() {
        slot = Slots.TRAP;

        SmartDashboard.putNumber("Left P Gain", PIDConstants.Left.Trap.kP);
        SmartDashboard.putNumber("Left I Gain", PIDConstants.Left.Trap.kI);
        SmartDashboard.putNumber("Left D Gain", PIDConstants.Left.Trap.kD);
        SmartDashboard.putNumber("Left Feed Forward", PIDConstants.Left.kFF);

        SmartDashboard.putNumber("Right P Gain", PIDConstants.Right.Trap.kP);
        SmartDashboard.putNumber("Right I Gain", PIDConstants.Right.Trap.kI);
        SmartDashboard.putNumber("Right D Gain", PIDConstants.Right.Trap.kD);
        SmartDashboard.putNumber("Right Feed Forward", PIDConstants.Right.kFF);

        SmartDashboard.putNumber("Left Velocity", PIDConstants.Left.Trap.velocity);
        SmartDashboard.putNumber("Right Velocity", PIDConstants.Right.Trap.velocity);

        SmartDashboard.putString("Shooter Mode", "Trap");

        leftTarget = PIDConstants.Left.Trap.velocity;
        rightTarget = PIDConstants.Right.Trap.velocity;
    }

    public void longMode() {
        slot = Slots.LONG;

        SmartDashboard.putNumber("Left P Gain", PIDConstants.Left.Long.kP);
        SmartDashboard.putNumber("Left I Gain", PIDConstants.Left.Long.kI);
        SmartDashboard.putNumber("Left D Gain", PIDConstants.Left.Long.kD);
        SmartDashboard.putNumber("Left Feed Forward", PIDConstants.Left.kFF);

        SmartDashboard.putNumber("Right P Gain", PIDConstants.Right.Long.kP);
        SmartDashboard.putNumber("Right I Gain", PIDConstants.Right.Long.kI);
        SmartDashboard.putNumber("Right D Gain", PIDConstants.Right.Long.kD);
        SmartDashboard.putNumber("Right Feed Forward", PIDConstants.Right.kFF);

        SmartDashboard.putNumber("Left Velocity", PIDConstants.Left.Long.velocity);
        SmartDashboard.putNumber("Right Velocity", PIDConstants.Right.Long.velocity);

        SmartDashboard.putString("Shooter Mode", "Long");

        leftTarget = PIDConstants.Left.Long.velocity;
        rightTarget = PIDConstants.Right.Long.velocity;
    }

    public void mediumMode() {
        slot = Slots.MEDIUM;

        SmartDashboard.putNumber("Left P Gain", PIDConstants.Left.Medium.kP);
        SmartDashboard.putNumber("Left I Gain", PIDConstants.Left.Medium.kI);
        SmartDashboard.putNumber("Left D Gain", PIDConstants.Left.Medium.kD);
        SmartDashboard.putNumber("Left Feed Forward", PIDConstants.Left.kFF);

        SmartDashboard.putNumber("Right P Gain", PIDConstants.Right.Medium.kP);
        SmartDashboard.putNumber("Right I Gain", PIDConstants.Right.Medium.kI);
        SmartDashboard.putNumber("Right D Gain", PIDConstants.Right.Medium.kD);
        SmartDashboard.putNumber("Right Feed Forward", PIDConstants.Right.kFF);

        SmartDashboard.putNumber("Left Velocity", PIDConstants.Left.Medium.velocity);
        SmartDashboard.putNumber("Right Velocity", PIDConstants.Right.Medium.velocity);

        SmartDashboard.putString("Shooter Mode", "Medium");

        leftTarget = PIDConstants.Left.Medium.velocity;
        rightTarget = PIDConstants.Right.Medium.velocity;
    }

    public void shortMode() {
        slot = Slots.SHORT;

        SmartDashboard.putNumber("Left P Gain", PIDConstants.Left.Short.kP);
        SmartDashboard.putNumber("Left I Gain", PIDConstants.Left.Short.kI);
        SmartDashboard.putNumber("Left D Gain", PIDConstants.Left.Short.kD);
        SmartDashboard.putNumber("Left Feed Forward", PIDConstants.Left.kFF);

        SmartDashboard.putNumber("Right P Gain", PIDConstants.Right.Short.kP);
        SmartDashboard.putNumber("Right I Gain", PIDConstants.Right.Short.kI);
        SmartDashboard.putNumber("Right D Gain", PIDConstants.Right.Short.kD);
        SmartDashboard.putNumber("Right Feed Forward", PIDConstants.Right.kFF);

        SmartDashboard.putNumber("Left Velocity", PIDConstants.Left.Short.velocity);
        SmartDashboard.putNumber("Right Velocity", PIDConstants.Right.Short.velocity);

        SmartDashboard.putString("Shooter Mode", "Short");

        leftTarget = PIDConstants.Left.Short.velocity;
        rightTarget = PIDConstants.Right.Short.velocity;
    }

    public Command trapCommand() {
        return runOnce(this::trapMode);
    }

    public Command longCommand() {
        return runOnce(this::longMode);
    }

    public Command mediumCommand() {
        return runOnce(this::mediumMode);
    }

    public Command shortCommand() {
        return runOnce(this::shortMode);
    }

    public void setRightToVel(double velRPM) {
        rightPid.setReference(velRPM, ControlType.kVelocity);
    }

    public void setLeftToVel(double velRPM) {
        leftPid.setReference(velRPM, ControlType.kVelocity);
    }

    public Command setRightToVelCommand(double velRPM) {
        return runOnce(() -> setRightToVel(velRPM));
    }

    public Command setLeftToVelCommand(double velRPM) {
        return runOnce(() -> setLeftToVel(velRPM));
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
