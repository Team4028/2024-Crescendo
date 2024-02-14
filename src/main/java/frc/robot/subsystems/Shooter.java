// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.units.Units;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog.State;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

public class Shooter extends SubsystemBase {
    private CANSparkFlex m_rightMotor, m_leftMotor;
    private CANSparkMax m_pivotMotor;
    private RelativeEncoder m_rightEncoder, m_leftEncoder, m_pivotEncoder;
    private SparkPIDController m_rightPid, m_leftPid, m_pivotPid;

    private DataLog m_log;
    private DoubleLogEntry rightMotorCurrent, leftMotorCurrent, rightMotorVelocity, leftMotorVelocity, leftMotorVoltage,
            rightMotorVoltage, rightPosLog, leftPosLog;
    public StringLogEntry stateLog;

    private int scan;

    private int m_slot = 0;

    private final class PIDConstants {
        private static final int TRAP_SLOT = 1;
        private static final int SPEAKER_SLOT = 0;

        private static class Left {
            private static double kFF = 0.00019;

            private final class Trap {
                private static double velocity = 1300;
                private static double kP = 0.0002;
                private static double kI = 0.0;
                private static double kD = 0.0;
            }

            private final class Speaker {
                private static double velocity = 2500;
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

            private final class Speaker {
                private static double velocity = 3400;
                private static double kP = 0.004;
                private static double kI = 0.0;
                private static double kD = 0.0;
            }
        }

        private static class Pivot {
            private static double kFF = 0.0;

            private static double kP = 0.1;
            private static double kI = 0.0;
            private static double kD = 0.0;
        }
    }

    private SysIdRoutine routine = new SysIdRoutine(
            new SysIdRoutine.Config(null, null, null, this::logState),
            new SysIdRoutine.Mechanism((volts) -> setMotorsVoltage(volts.in(Units.Volts)), null, this));

    public Shooter() {
        scan = 0;
        m_log = DataLogManager.getLog();
        initLogs();

        // Shooters
        m_rightMotor = new CANSparkFlex(9, MotorType.kBrushless);
        m_leftMotor = new CANSparkFlex(10, MotorType.kBrushless);

        m_rightMotor.setInverted(false);

        m_rightEncoder = m_rightMotor.getEncoder();
        m_leftEncoder = m_leftMotor.getEncoder();

        m_rightMotor.setSmartCurrentLimit(80);
        m_leftMotor.setSmartCurrentLimit(60);

        // GABES COOL THEORY PLS RE,MINMD
        m_rightMotor.setClosedLoopRampRate(0.1);
        m_leftMotor.setClosedLoopRampRate(0.1);

        m_rightPid = m_rightMotor.getPIDController();
        m_leftPid = m_leftMotor.getPIDController();
        m_rightPid.setFeedbackDevice(m_rightEncoder);
        m_leftPid.setFeedbackDevice(m_leftEncoder);

        // PID Constants
        int slot = PIDConstants.SPEAKER_SLOT;

        m_rightPid.setP(PIDConstants.Right.Speaker.kP, slot);
        m_rightPid.setI(PIDConstants.Right.Speaker.kI, slot);
        m_rightPid.setD(PIDConstants.Right.Speaker.kD, slot);
        m_rightPid.setFF(PIDConstants.Right.kFF, slot);

        slot = PIDConstants.TRAP_SLOT;

        m_rightPid.setP(PIDConstants.Right.Trap.kP, slot);
        m_rightPid.setI(PIDConstants.Right.Trap.kI, slot);
        m_rightPid.setD(PIDConstants.Right.Trap.kD, slot);
        m_rightPid.setFF(PIDConstants.Right.kFF, slot);

        m_rightPid.setOutputRange(-1, 1);

        // LEFT
        slot = PIDConstants.SPEAKER_SLOT;

        m_leftPid.setP(PIDConstants.Left.Speaker.kP, slot);
        m_leftPid.setI(PIDConstants.Left.Speaker.kI, slot);
        m_leftPid.setD(PIDConstants.Left.Speaker.kD, slot);
        m_leftPid.setFF(PIDConstants.Left.kFF, slot);

        slot = PIDConstants.TRAP_SLOT;

        m_leftPid.setP(PIDConstants.Left.Trap.kP, slot);
        m_leftPid.setI(PIDConstants.Left.Trap.kI, slot);
        m_leftPid.setD(PIDConstants.Left.Trap.kD, slot);
        m_leftPid.setFF(PIDConstants.Left.kFF, slot);

        m_leftPid.setOutputRange(-1, 1);

        speakerMode();

        // PIVOT
        m_pivotMotor = new CANSparkMax(13, MotorType.kBrushless);

        // Change this is needed
        m_pivotMotor.setInverted(false);

        m_pivotEncoder = m_pivotMotor.getEncoder();
        m_pivotPid = m_pivotMotor.getPIDController();

        m_pivotPid.setP(PIDConstants.Pivot.kP);
        m_pivotPid.setI(PIDConstants.Pivot.kI);
        m_pivotPid.setD(PIDConstants.Pivot.kD);
        m_pivotPid.setFF(PIDConstants.Pivot.kFF);
    }

    public void runPivotMotor(double vBus) {
        m_pivotMotor.set(vBus);
    }

    public Command runPivotCommand(double vBus) {
        return runOnce(() -> runPivotMotor(vBus));
    }

    public void runPivotToPosition(double position) {
        m_pivotPid.setReference(position, ControlType.kPosition);
    }

    /**
     * Run motors at the velocity input from the dashboard.
     * 
     * @return A {@link Command} that runs both motors at their desired input
     *         velocities.
     */
    public Command runVelocityCommand() {
        return runOnce(() -> {
            double left = SmartDashboard.getNumber("Left Velocity", 0);
            double right = SmartDashboard.getNumber("Right Velocity", 0);

            setRightToVel(right);
            setLeftToVel(left);
        });
    }

    public void stop() {
        m_rightMotor.stopMotor();
        m_leftMotor.stopMotor();
    }

    public Command stopCommand() {
        return runOnce(this::stop);
    }

    public void trapMode() {
        m_slot = PIDConstants.TRAP_SLOT;

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
    }

    public void speakerMode() {
        m_slot = PIDConstants.SPEAKER_SLOT;

        SmartDashboard.putNumber("Left P Gain", PIDConstants.Left.Speaker.kP);
        SmartDashboard.putNumber("Left I Gain", PIDConstants.Left.Speaker.kI);
        SmartDashboard.putNumber("Left D Gain", PIDConstants.Left.Speaker.kD);
        SmartDashboard.putNumber("Left Feed Forward", PIDConstants.Left.kFF);

        SmartDashboard.putNumber("Right P Gain", PIDConstants.Right.Speaker.kP);
        SmartDashboard.putNumber("Right I Gain", PIDConstants.Right.Speaker.kI);
        SmartDashboard.putNumber("Right D Gain", PIDConstants.Right.Speaker.kD);
        SmartDashboard.putNumber("Right Feed Forward", PIDConstants.Right.kFF);

        SmartDashboard.putNumber("Left Velocity", PIDConstants.Left.Speaker.velocity);
        SmartDashboard.putNumber("Right Velocity", PIDConstants.Right.Speaker.velocity);
    }

    public Command trapCommand() {
        return runOnce(this::trapMode);
    }

    public Command speakerCommand() {
        return runOnce(this::speakerMode);
    }

    private void setMotorsVoltage(double volts) {
        // m_leftMotor.setVoltage(volts);
        m_rightMotor.setVoltage(volts);
    }

    private void logState(State state) {
        switch (state) {
            case kQuasistaticForward:
                stateLog.append("quasistatic-forward");
                break;
            case kQuasistaticReverse:
                stateLog.append("quasistatic-reverse");
                break;
            case kDynamicForward:
                stateLog.append("dynamic-forward");
                break;
            case kDynamicReverse:
                stateLog.append("dynamic-reverse");
                break;
            default:
                break;
        }

        logValues();
    }

    public Command runQuasi(Direction dir) {
        return routine.quasistatic(dir);
    }

    public Command runDynamuc(Direction dir) {
        return routine.dynamic(dir);
    }

    public void setRightToVel(double velRPM) {
        m_rightPid.setReference(velRPM, ControlType.kVelocity);
    }

    public void setLeftToVel(double velRPM) {
        m_leftPid.setReference(velRPM, ControlType.kVelocity);
    }

    public Command setRightToVelCommand(double velRPM) {
        return runOnce(() -> setRightToVel(velRPM));
    }

    public Command setLeftToVelCommand(double velRPM) {
        return runOnce(() -> setLeftToVel(velRPM));
    }

    public void spinMotorRight(double vBus) {
        m_rightMotor.set(vBus);
    }

    public void spinMotorLeft(double vBus) {
        m_leftMotor.set(vBus);
    }

    public Command spinMotorRightCommand(double vBus) {
        return runOnce(() -> spinMotorRight(vBus));
    }

    public Command spinMotorLeftCommand(double vBus) {
        return runOnce(() -> spinMotorLeft(vBus));
    }

    private void initLogs() {
        rightMotorCurrent = new DoubleLogEntry(m_log, "/Shooter/right/Current");
        leftMotorCurrent = new DoubleLogEntry(m_log, "/Shooter/left/Current");
        rightMotorVelocity = new DoubleLogEntry(m_log, "/Shooter/right/Velocity");
        leftMotorVelocity = new DoubleLogEntry(m_log, "/Shooter/left/Velocity");
        rightMotorVoltage = new DoubleLogEntry(m_log, "/Shooter/right/Voltage");
        leftMotorVoltage = new DoubleLogEntry(m_log, "/Shooter/left/Voltage");
        leftPosLog = new DoubleLogEntry(m_log, "/Shooter/left/Pos");
        rightPosLog = new DoubleLogEntry(m_log, "/Shooter/right/Pos");
        stateLog = new StringLogEntry(m_log, "test-state");
    }

    public void logValues() {
        rightMotorCurrent.append(m_rightMotor.getOutputCurrent());
        leftMotorCurrent.append(m_leftMotor.getOutputCurrent());

        rightMotorVelocity.append(m_rightMotor.getEncoder().getVelocity());
        leftMotorVelocity.append(m_leftMotor.getEncoder().getVelocity());

        rightMotorVoltage.append(m_rightMotor.getAppliedOutput() * m_rightMotor.getBusVoltage());
        leftMotorVoltage.append(m_leftMotor.getAppliedOutput() * m_leftMotor.getBusVoltage());

        leftPosLog.append(m_leftEncoder.getPosition());
        rightPosLog.append(m_rightEncoder.getPosition());
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        if (scan != 0 && scan % 3 == 0) {
            scan = 0;
            SmartDashboard.putNumber("rightMotorCurrent", m_rightMotor.getOutputCurrent());
            SmartDashboard.putNumber("leftMotorCurrent", m_leftMotor.getOutputCurrent());
            SmartDashboard.putNumber("rightMotorVel", m_rightEncoder.getVelocity());
            SmartDashboard.putNumber("leftMotorVel", m_leftEncoder.getVelocity());
        }
        scan++;

        SmartDashboard.putNumber("Left Actual", m_leftEncoder.getVelocity());
        SmartDashboard.putNumber("Right Actual", m_rightEncoder.getVelocity());

        // testing
        double leftP = SmartDashboard.getNumber("Left P Gain", 0);
        double leftI = SmartDashboard.getNumber("Left I Gain", 0);
        double leftD = SmartDashboard.getNumber("Left D Gain", 0);
        double leftFF = SmartDashboard.getNumber("Left Feed Forward", 0);

        // if PID coefficients on SmartDashboard have changed, write new values to
        // controller
        if (leftP != m_leftPid.getP(m_slot)) {
            m_leftPid.setP(leftP, m_slot);
        }
        if (leftI != m_leftPid.getI(m_slot)) {
            m_leftPid.setI(leftI, m_slot);
        }
        if (leftD != m_leftPid.getD(m_slot)) {
            m_leftPid.setD(leftD, m_slot);
        }
        if (leftFF != m_leftPid.getFF(m_slot)) {
            m_leftPid.setFF(leftFF, m_slot);
        }

        double rightP = SmartDashboard.getNumber("Right P Gain", 0);
        double rightI = SmartDashboard.getNumber("Right I Gain", 0);
        double rightD = SmartDashboard.getNumber("Right D Gain", 0);
        double rightFF = SmartDashboard.getNumber("Right Feed Forward", 0);

        // if PID coefficients on SmartDashboard have changed, write new values to
        // controller
        if (rightP != m_rightPid.getP(m_slot)) {
            m_rightPid.setP(rightP, m_slot);
        }
        if (rightI != m_rightPid.getI(m_slot)) {
            m_rightPid.setI(rightI, m_slot);
        }
        if (rightD != m_rightPid.getD(m_slot)) {
            m_rightPid.setD(rightD, m_slot);
        }
        if (rightFF != m_rightPid.getFF(m_slot)) {
            m_rightPid.setFF(rightFF, m_slot);
        }
    }
}
