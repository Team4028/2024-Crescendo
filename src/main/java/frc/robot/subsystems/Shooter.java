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

    // private DataLog m_log;
    // private DoubleLogEntry rightMotorCurrent, leftMotorCurrent,
    // rightMotorVelocity, leftMotorVelocity, leftMotorVoltage,
    // rightMotorVoltage, rightPosLog, leftPosLog;
    // public StringLogEntry stateLog;

    // private int scan;

    private int slot = 0;

    private double leftTarget, rightTarget;

    private final Timer m_zeroTimer;

    private final class PIDConstants {
        private static final int TRAP_SLOT = 3;
        private static final int SHORT_SLOT = 2;
        private static final int MEDIUM_SLOT = 1;
        private static final int LONG_SLOT = 0;

        private static class Left {
            private static double kFF = 0.00019;

            private final class Trap {
                private static double velocity = 1300;
                private static double kP = 0.0002;
                private static double kI = 0.0;
                private static double kD = 0.0;
            }

            private final class Long {
                private static double velocity = 2500;
                private static double kP = 0.001;
                private static double kI = 0.0;
                private static double kD = 0.0;
            }

            private final class Medium {
                private static double velocity = 1250;
                private static double kP = 0.001;
                private static double kI = 0.0;
                private static double kD = 0.0;
            }

            private final class Short {
                private static double velocity = 625;
                private static double kP = 0.001;
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
                private static double velocity = 3400;
                private static double kP = 0.002;
                private static double kI = 0.0;
                private static double kD = 0.0;
            }

            private final class Medium {
                private static double velocity = 1700;
                private static double kP = 0.002;
                private static double kI = 0.0;
                private static double kD = 0.0;
            }

            private final class Short {
                private static double velocity = 850;
                private static double kP = 0.002;
                private static double kI = 0.0;
                private static double kD = 0.0;
            }
        }

        private static class Pivot {
            private static double kFF = 0.0;

            private static double kP = 0.02;
            private static double kI = 0.0;
            private static double kD = 0.0;

            //@formatter:off
            private static double LONG_POSITION = 2.5; // was 10.            ===================================================
            private static double MEDIUM_POSITION = 26.75; // was 107.     ||  **DIVIDED BY 4 BC TOOK A STAGE OFF OF MPLANETARY ||
                                                           //              ||    (16:1 -> 4:1)**                                ||
            private static double SHORT_POSITION = 45.; // was 108.          ===================================================
            //@formatter:on

            private static double MIN_VAL = 0.;
            private static double MAX_VAL = 55.0;
        }
    }

    // private SysIdRoutine routine = new SysIdRoutine(
    // new SysIdRoutine.Config(null, null, null, this::logState),
    // new SysIdRoutine.Mechanism((volts) ->
    // setMotorsVoltage(volts.in(Units.Volts)), null, this));

    public Shooter() {
        // scan = 0;
        // m_log = DataLogManager.getLog();
        initLogs();

        // ==================================
        // SHOOTER WHEELS
        // ==================================
        rightMotor = new CANSparkFlex(9, MotorType.kBrushless);
        leftMotor = new CANSparkFlex(10, MotorType.kBrushless);

        rightMotor.setInverted(false);

        rightEncoder = rightMotor.getEncoder();
        leftEncoder = leftMotor.getEncoder();

        rightMotor.setSmartCurrentLimit(80);
        leftMotor.setSmartCurrentLimit(60);

        rightEncoder.setMeasurementPeriod(16);
        rightEncoder.setAverageDepth(2);

        leftEncoder.setMeasurementPeriod(16);
        leftEncoder.setAverageDepth(2);

        // GABES COOL THEORY PLS RE,MINMD
        rightMotor.setClosedLoopRampRate(0.1);
        leftMotor.setClosedLoopRampRate(0.1);

        rightPid = rightMotor.getPIDController();
        leftPid = leftMotor.getPIDController();
        rightPid.setFeedbackDevice(rightEncoder);
        leftPid.setFeedbackDevice(leftEncoder);

        // ==================================
        // SHOOTER PID
        // ==================================
        // int slot = PIDConstants.TRAP_SLOT;

        rightPid.setP(PIDConstants.Right.Long.kP, PIDConstants.LONG_SLOT);
        rightPid.setI(PIDConstants.Right.Long.kI, PIDConstants.LONG_SLOT);
        rightPid.setD(PIDConstants.Right.Long.kD, PIDConstants.LONG_SLOT);
        rightPid.setFF(PIDConstants.Right.kFF, PIDConstants.LONG_SLOT);

        // slot = PIDConstants.MEDIUM_SLOT;

        rightPid.setP(PIDConstants.Right.Medium.kP, PIDConstants.MEDIUM_SLOT);
        rightPid.setI(PIDConstants.Right.Medium.kI, PIDConstants.MEDIUM_SLOT);
        rightPid.setD(PIDConstants.Right.Medium.kD, PIDConstants.MEDIUM_SLOT);
        rightPid.setFF(PIDConstants.Right.kFF, PIDConstants.MEDIUM_SLOT);

        // slot = PIDConstants.SHORT_SLOT;

        rightPid.setP(PIDConstants.Right.Short.kP, PIDConstants.SHORT_SLOT);
        rightPid.setI(PIDConstants.Right.Short.kI, PIDConstants.SHORT_SLOT);
        rightPid.setD(PIDConstants.Right.Short.kD, PIDConstants.SHORT_SLOT);
        rightPid.setFF(PIDConstants.Right.kFF, PIDConstants.SHORT_SLOT);

        rightPid.setP(PIDConstants.Right.Trap.kP, PIDConstants.TRAP_SLOT);
        rightPid.setI(PIDConstants.Right.Trap.kI, PIDConstants.TRAP_SLOT);
        rightPid.setD(PIDConstants.Right.Trap.kD, PIDConstants.TRAP_SLOT);
        rightPid.setFF(PIDConstants.Right.kFF, PIDConstants.TRAP_SLOT);

        rightPid.setOutputRange(-1, 1);

        // LEFT
        // slot = PIDConstants.LONG_SLOT;

        leftPid.setP(PIDConstants.Left.Long.kP, PIDConstants.LONG_SLOT);
        leftPid.setI(PIDConstants.Left.Long.kI, PIDConstants.LONG_SLOT);
        leftPid.setD(PIDConstants.Left.Long.kD, PIDConstants.LONG_SLOT);
        leftPid.setFF(PIDConstants.Left.kFF, PIDConstants.LONG_SLOT);

        // slot = PIDConstants.MEDIUM_SLOT;

        leftPid.setP(PIDConstants.Left.Medium.kP, PIDConstants.MEDIUM_SLOT);
        leftPid.setI(PIDConstants.Left.Medium.kI, PIDConstants.MEDIUM_SLOT);
        leftPid.setD(PIDConstants.Left.Medium.kD, PIDConstants.MEDIUM_SLOT);
        leftPid.setFF(PIDConstants.Left.kFF, PIDConstants.MEDIUM_SLOT);

        // slot = PIDConstants.SHORT_SLOT;

        leftPid.setP(PIDConstants.Left.Short.kP, PIDConstants.SHORT_SLOT);
        leftPid.setI(PIDConstants.Left.Short.kI, PIDConstants.SHORT_SLOT);
        leftPid.setD(PIDConstants.Left.Short.kD, PIDConstants.SHORT_SLOT);
        leftPid.setFF(PIDConstants.Left.kFF, PIDConstants.SHORT_SLOT);

        // slot = PIDConstants.TRAP_SLOT;

        leftPid.setP(PIDConstants.Left.Trap.kP, PIDConstants.TRAP_SLOT);
        leftPid.setI(PIDConstants.Left.Trap.kI, PIDConstants.TRAP_SLOT);
        leftPid.setD(PIDConstants.Left.Trap.kD, PIDConstants.TRAP_SLOT);
        leftPid.setFF(PIDConstants.Left.kFF, PIDConstants.TRAP_SLOT);

        leftPid.setOutputRange(-1, 1);

        longMode();

        // ==================================
        // PIVOT
        // ==================================
        pivotMotor = new CANSparkMax(13, MotorType.kBrushless);

        pivotMotor.setInverted(true);
        pivotMotor.setIdleMode(IdleMode.kBrake);

        pivotEncoder = pivotMotor.getEncoder();
        pivotPid = pivotMotor.getPIDController();

        // ==================================
        // PIVOT PID
        // ==================================
        pivotPid.setP(PIDConstants.Pivot.kP);
        pivotPid.setI(PIDConstants.Pivot.kI);
        pivotPid.setD(PIDConstants.Pivot.kD);
        pivotPid.setFF(PIDConstants.Pivot.kFF);

        // ==================================
        // TIMER
        // ==================================
        m_zeroTimer = new Timer();
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
            m_zeroTimer.restart();
        })
                .andThen(runPivotCommand(-0.1).repeatedly()
                        .until(() -> m_zeroTimer.get() >= 0.2 && Math.abs(pivotEncoder.getVelocity()) < 0.2))
                .andThen(runPivotCommand(0.).alongWith(Commands.runOnce(() -> m_zeroTimer.stop())))
                .andThen(runOnce(() -> pivotEncoder.setPosition(0.)));
    }

    public double getPivotPosition() {
        switch (slot) {
            case PIDConstants.TRAP_SLOT:
            case PIDConstants.SHORT_SLOT:
                return PIDConstants.Pivot.SHORT_POSITION;
            case PIDConstants.MEDIUM_SLOT:
                return PIDConstants.Pivot.MEDIUM_POSITION;
            case PIDConstants.LONG_SLOT:
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
                    // double left = SmartDashboard.getNumber("Left Velocity", 0);
                    // double right = SmartDashboard.getNumber("Right Velocity", 0);

                    setRightToVel(rightTarget);
                    setLeftToVel(leftTarget);
                    runPivotToPosition(getPivotPosition());
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
        SmartDashboard.putNumber("Slot", slot);
        switch (slot) {
            case PIDConstants.TRAP_SLOT:
                trapMode();
                break;
            case PIDConstants.SHORT_SLOT:
                shortMode();
                break;
            case PIDConstants.MEDIUM_SLOT:
                mediumMode();
                break;
            case PIDConstants.LONG_SLOT:
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
        });
    }

    public void trapMode() {
        slot = PIDConstants.TRAP_SLOT;

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
        slot = PIDConstants.LONG_SLOT;

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
        slot = PIDConstants.MEDIUM_SLOT;

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
        slot = PIDConstants.SHORT_SLOT;

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

    // private void setMotorsVoltage(double volts) {
    // // m_leftMotor.setVoltage(volts);
    // m_rightMotor.setVoltage(volts);
    // }

    // private void logState(State state) {
    // switch (state) {
    // case kQuasistaticForward:
    // stateLog.append("quasistatic-forward");
    // break;
    // case kQuasistaticReverse:
    // stateLog.append("quasistatic-reverse");
    // break;
    // case kDynamicForward:
    // stateLog.append("dynamic-forward");
    // break;
    // case kDynamicReverse:
    // stateLog.append("dynamic-reverse");
    // break;
    // default:
    // break;
    // }

    // logValues();
    // }

    // public Command runQuasi(Direction dir) {
    // return routine.quasistatic(dir);
    // }

    // public Command runDynamuc(Direction dir) {
    // return routine.dynamic(dir);
    // }

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

    private void initLogs() {
        // rightMotorCurrent = new DoubleLogEntry(m_log, "/Shooter/right/Current");
        // leftMotorCurrent = new DoubleLogEntry(m_log, "/Shooter/left/Current");
        // rightMotorVelocity = new DoubleLogEntry(m_log, "/Shooter/right/Velocity");
        // leftMotorVelocity = new DoubleLogEntry(m_log, "/Shooter/left/Velocity");
        // rightMotorVoltage = new DoubleLogEntry(m_log, "/Shooter/right/Voltage");
        // leftMotorVoltage = new DoubleLogEntry(m_log, "/Shooter/left/Voltage");
        // leftPosLog = new DoubleLogEntry(m_log, "/Shooter/left/Pos");
        // rightPosLog = new DoubleLogEntry(m_log, "/Shooter/right/Pos");
        // stateLog = new StringLogEntry(m_log, "test-state");
    }

    // public void logValues() {
    // rightMotorCurrent.append(m_rightMotor.getOutputCurrent());
    // leftMotorCurrent.append(m_leftMotor.getOutputCurrent());

    // rightMotorVelocity.append(m_rightMotor.getEncoder().getVelocity());
    // leftMotorVelocity.append(m_leftMotor.getEncoder().getVelocity());

    // rightMotorVoltage.append(m_rightMotor.getAppliedOutput() *
    // m_rightMotor.getBusVoltage());
    // leftMotorVoltage.append(m_leftMotor.getAppliedOutput() *
    // m_leftMotor.getBusVoltage());

    // leftPosLog.append(m_leftEncoder.getPosition());
    // rightPosLog.append(m_rightEncoder.getPosition());
    // }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // SmartDashboard.putNumber("Left Actual", m_leftEncoder.getVelocity());
        // SmartDashboard.putNumber("Right Actual", m_rightEncoder.getVelocity());

        // // testing
        // double leftP = SmartDashboard.getNumber("Left P Gain", 0);
        // double leftI = SmartDashboard.getNumber("Left I Gain", 0);
        // double leftD = SmartDashboard.getNumber("Left D Gain", 0);
        // double leftFF = SmartDashboard.getNumber("Left Feed Forward", 0);

        // // if PID coefficients on SmartDashboard have changed, write new values to
        // // controller
        // if (leftP != m_leftPid.getP(m_slot)) {
        // m_leftPid.setP(leftP, m_slot);
        // }
        // if (leftI != m_leftPid.getI(m_slot)) {
        // m_leftPid.setI(leftI, m_slot);
        // }
        // if (leftD != m_leftPid.getD(m_slot)) {
        // m_leftPid.setD(leftD, m_slot);
        // }
        // if (leftFF != m_leftPid.getFF(m_slot)) {
        // m_leftPid.setFF(leftFF, m_slot);
        // }

        // double rightP = SmartDashboard.getNumber("Right P Gain", 0);
        // double rightI = SmartDashboard.getNumber("Right I Gain", 0);
        // double rightD = SmartDashboard.getNumber("Right D Gain", 0);
        // double rightFF = SmartDashboard.getNumber("Right Feed Forward", 0);

        // // if PID coefficients on SmartDashboard have changed, write new values to
        // // controller
        // if (rightP != m_rightPid.getP(m_slot)) {
        // m_rightPid.setP(rightP, m_slot);
        // }
        // if (rightI != m_rightPid.getI(m_slot)) {
        // m_rightPid.setI(rightI, m_slot);
        // }
        // if (rightD != m_rightPid.getD(m_slot)) {
        // m_rightPid.setD(rightD, m_slot);
        // }
        // if (rightFF != m_rightPid.getFF(m_slot)) {
        // m_rightPid.setFF(rightFF, m_slot);
        // }

        SmartDashboard.putNumber("Pivot Position", pivotEncoder.getPosition());
        SmartDashboard.putNumber("Pivot Velocity", pivotEncoder.getVelocity());
    }
}
