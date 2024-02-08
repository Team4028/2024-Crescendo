// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
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
    private RelativeEncoder m_rightEncoder, m_leftEncoder;
    private SparkPIDController m_rightPid, m_leftPid;

    private DataLog m_log;
    private DoubleLogEntry rightMotorCurrent, leftMotorCurrent, rightMotorVelocity, leftMotorVelocity, leftMotorVoltage,
            rightMotorVoltage, rightPosLog, leftPosLog;
    public StringLogEntry stateLog;

    private int scan;

    private static double RIGHT_SHOOTER_KP = 0.004;
    private static double RIGHT_SHOOTER_KI = 0.0;
    private static double RIGHT_SHOOTER_KD = 0.0;
    private static double RIGHT_SHOOTER_KF = 0.00022;

    private static double LEFT_SHOOTER_KP = 0.001;
    private static double LEFT_SHOOTER_KI = 0.0;
    private static double LEFT_SHOOTER_KD = 0.0;
    private static double LEFT_SHOOTER_KF = 0.00019;

    // trap:
    // left P: 0.0002
    // left vel: 1300
    // right P: 0.001
    // right vel: 1300

    private SysIdRoutine routine = new SysIdRoutine(
            new SysIdRoutine.Config(null, null, null, this::logState),
            new SysIdRoutine.Mechanism((volts) -> setMotorsVoltage(volts.in(Units.Volts)), null, this));

    public Shooter() {
        scan = 0;
        m_log = DataLogManager.getLog();
        initLogs();

        // Motor Init
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
        m_rightPid.setP(RIGHT_SHOOTER_KP);
        m_rightPid.setI(RIGHT_SHOOTER_KI);
        m_rightPid.setD(RIGHT_SHOOTER_KD);
        m_rightPid.setFF(RIGHT_SHOOTER_KF);
        m_rightPid.setOutputRange(-1, 1);

        m_leftPid.setP(LEFT_SHOOTER_KP);
        m_leftPid.setI(LEFT_SHOOTER_KI);
        m_leftPid.setD(LEFT_SHOOTER_KD);
        m_leftPid.setFF(LEFT_SHOOTER_KF);
        m_leftPid.setOutputRange(-1, 1);

        SmartDashboard.putNumber("Left P Gain", LEFT_SHOOTER_KP);
        SmartDashboard.putNumber("Left I Gain", LEFT_SHOOTER_KI);
        SmartDashboard.putNumber("Left D Gain", LEFT_SHOOTER_KD);
        SmartDashboard.putNumber("Left Feed Forward", LEFT_SHOOTER_KF);

        SmartDashboard.putNumber("Right P Gain", RIGHT_SHOOTER_KP);
        SmartDashboard.putNumber("Right I Gain", RIGHT_SHOOTER_KI);
        SmartDashboard.putNumber("Right D Gain", RIGHT_SHOOTER_KD);
        SmartDashboard.putNumber("Right Feed Forward", RIGHT_SHOOTER_KF);

        SmartDashboard.putNumber("Left Velocity", 2500);
        SmartDashboard.putNumber("Right Velocity", 3400);
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
        rightMotorCurrent = new DoubleLogEntry(m_log, "/right/Current");
        leftMotorCurrent = new DoubleLogEntry(m_log, "/left/Current");
        rightMotorVelocity = new DoubleLogEntry(m_log, "/right/Velocity");
        leftMotorVelocity = new DoubleLogEntry(m_log, "/left/Velocity");
        rightMotorVoltage = new DoubleLogEntry(m_log, "/right/Voltage");
        leftMotorVoltage = new DoubleLogEntry(m_log, "/left/Voltage");
        leftPosLog = new DoubleLogEntry(m_log, "/left/Pos");
        rightPosLog = new DoubleLogEntry(m_log, "/right/Pos");
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
        if (leftP != LEFT_SHOOTER_KP) {
            LEFT_SHOOTER_KP = leftP;
            m_leftPid.setP(leftP);
        }
        if (leftI != LEFT_SHOOTER_KI) {
            LEFT_SHOOTER_KP = leftP;
            m_leftPid.setI(leftI);
        }
        if (leftD != LEFT_SHOOTER_KD) {
            LEFT_SHOOTER_KP = leftP;
            m_leftPid.setD(leftD);
        }
        if (leftFF != LEFT_SHOOTER_KF) {
            LEFT_SHOOTER_KP = leftP;
            m_leftPid.setFF(leftFF);
        }

        double rightP = SmartDashboard.getNumber("Right P Gain", 0);
        double rightI = SmartDashboard.getNumber("Right I Gain", 0);
        double rightD = SmartDashboard.getNumber("Right D Gain", 0);
        double rightFF = SmartDashboard.getNumber("Right Feed Forward", 0);

        // if PID coefficients on SmartDashboard have changed, write new values to
        // controller
        if (rightP != RIGHT_SHOOTER_KP) {
            RIGHT_SHOOTER_KP = rightP;
            m_rightPid.setP(rightP);
        }
        if (rightI != RIGHT_SHOOTER_KI) {
            RIGHT_SHOOTER_KI = rightI;
            m_rightPid.setI(rightI);
        }
        if (rightD != RIGHT_SHOOTER_KD) {
            RIGHT_SHOOTER_KD = rightD;
            m_rightPid.setD(rightD);
        }
        if (rightFF != RIGHT_SHOOTER_KF) {
            RIGHT_SHOOTER_KF = rightFF;
            m_rightPid.setFF(rightFF);
        }
    }
}
