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
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

public class PrototypeShooter implements Subsystem {
  static PrototypeShooter instance;
  private CANSparkFlex rightMotor, leftMotor;
  private RelativeEncoder rightEncoder, leftEncoder;
  private SparkPIDController pidRight, pidLeft;

  private DataLog log;
  public DoubleLogEntry rightMotorCurrent, leftMotorCurrent, rightMotorVelocity, leftMotorVelocity, leftMotorVoltage,
      rightMotorVoltage, rightPosLog, leftPosLog;
  public StringLogEntry stateLog;

  private int scan;

  private SysIdRoutine routine = new SysIdRoutine(
      new SysIdRoutine.Config(null, null, null, this::logState),
      new SysIdRoutine.Mechanism((volts) -> setMotorsVoltage(volts.in(Units.Volts)), null, this));

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

  private PrototypeShooter() {
    scan = 0;
    log = DataLogManager.getLog();
    initLogs();
    rightMotor = new CANSparkFlex(12, MotorType.kBrushless);
    leftMotor = new CANSparkFlex(11, MotorType.kBrushless);
    rightEncoder = rightMotor.getEncoder();
    leftEncoder = leftMotor.getEncoder();

    rightMotor.setSmartCurrentLimit(80);
    leftMotor.setSmartCurrentLimit(80);

    rightMotor.setInverted(true);

    pidRight = rightMotor.getPIDController();
    pidLeft = leftMotor.getPIDController();
    pidRight.setFeedbackDevice(rightEncoder);
    pidLeft.setFeedbackDevice(leftEncoder);

    // PID Constants
    pidRight.setP(.0075);
    pidRight.setI(0);
    pidRight.setD(0.1);
    pidRight.setIZone(0);
    pidRight.setOutputRange(-1, 1);
    pidRight.setFF(0.0020301);

    pidLeft.setP(0.0075);
    pidLeft.setI(0);
    pidLeft.setD(0.1);
    pidLeft.setIZone(0);
    pidLeft.setOutputRange(-1, 1);
    pidLeft.setFF(0.0018903);
  }

  private void setRightToVel(double velRPM) {
    pidRight.setReference(velRPM, ControlType.kVelocity);
  }

  public Command runQuasi(Direction dir) {
    return routine.quasistatic(dir);
  }

  public Command runDynamuc(Direction dir) {
    return routine.dynamic(dir);
  }

  private void setLeftToVel(double velRPM) {
    pidLeft.setReference(velRPM, ControlType.kVelocity);
  }

  private void setMotorsVoltage(double volts) {
    leftMotor.setVoltage(volts);
    rightMotor.setVoltage(volts);
  }

  public Command setRightToVelCommand(double velRPM) {
    return runOnce(() -> setRightToVel(velRPM));
  }

  public Command setLeftToVelCommand(double velRPM) {
    return runOnce(() -> setLeftToVel(velRPM));
  }

  private void spinMotorRight(double vBus) {
    rightMotor.set(vBus);
  }

  private void spinMotorLeft(double vBus) {
    leftMotor.set(vBus);
  }

  public Command spinMotorRightCommand(double vBus) {
    return runOnce(() -> spinMotorRight(vBus));
  }

  public Command spinMotorLeftCommand(double vBus) {
    return runOnce(() -> spinMotorLeft(vBus));
  }

  private void initLogs() {
    rightMotorCurrent = new DoubleLogEntry(log, "/right/Current");
    leftMotorCurrent = new DoubleLogEntry(log, "/left/Current");
    rightMotorVelocity = new DoubleLogEntry(log, "/right/Velocity");
    leftMotorVelocity = new DoubleLogEntry(log, "/left/Velocity");
    rightMotorVoltage = new DoubleLogEntry(log, "/right/Voltage");
    leftMotorVoltage = new DoubleLogEntry(log, "/left/Voltage");
    leftPosLog = new DoubleLogEntry(log, "/left/Pos");
    rightPosLog = new DoubleLogEntry(log, "/right/Pos");
    stateLog = new StringLogEntry(log, "test-state");
  }

  public void logValues() {
    rightMotorCurrent.append(rightMotor.getOutputCurrent());
    leftMotorCurrent.append(leftMotor.getOutputCurrent());

    rightMotorVelocity.append(rightMotor.getEncoder().getVelocity());
    leftMotorVelocity.append(leftMotor.getEncoder().getVelocity());

    rightMotorVoltage.append(rightMotor.getAppliedOutput() * rightMotor.getBusVoltage());
    leftMotorVoltage.append(leftMotor.getAppliedOutput() * leftMotor.getBusVoltage());

    leftPosLog.append(leftEncoder.getPosition());
    rightPosLog.append(rightEncoder.getPosition());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (scan != 0 && scan % 3 == 0) {
      scan = 0;
      SmartDashboard.putNumber("rightMotorCurrent", rightMotor.getOutputCurrent());
      SmartDashboard.putNumber("leftMotorCurrent", leftMotor.getOutputCurrent());
      SmartDashboard.putNumber("rightMotorVel", rightEncoder.getVelocity());
      SmartDashboard.putNumber("leftMotorVel", leftEncoder.getVelocity());
    }
    scan++;
  }

  public static PrototypeShooter getInstance() {
    if (instance == null)
      instance = new PrototypeShooter();
    return instance;
  }
}
