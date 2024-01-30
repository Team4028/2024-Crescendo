// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PrototypeShooter extends SubsystemBase {
  static PrototypeShooter instance;
  private CANSparkFlex rightMotor, leftMotor;
  private RelativeEncoder rightEncoder, leftEncoder;
  private SparkPIDController pidRight, pidLeft;

  private DataLog log;
  public DoubleLogEntry rightMotorCurrent, leftMotorCurrent, rightMotorVelocity, leftMotorVelocity;

  private int scan;

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

    pidRight = rightMotor.getPIDController();
    pidLeft = leftMotor.getPIDController();
    pidRight.setFeedbackDevice(rightEncoder);
    pidLeft.setFeedbackDevice(leftEncoder);

    // PID Constants
    pidRight.setP(0);
    pidRight.setI(0);
    pidRight.setD(0);
    pidRight.setIZone(0);
    pidRight.setOutputRange(-0.9, 0.9);

    pidLeft.setP(0);
    pidLeft.setI(0);
    pidLeft.setD(0);
    pidLeft.setIZone(0);
    pidLeft.setOutputRange(-0.9, 0.9);
  }

  private void setRightToVel(double velRPM) {
    pidRight.setReference(velRPM, ControlType.kVelocity);
  }

  private void setLeftToVel(double velRPM) {
    pidLeft.setReference(velRPM, ControlType.kVelocity);
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
    leftMotor.set(-1. * vBus);
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
  }

  public void logValues() {
    rightMotorCurrent.append(rightMotor.getOutputCurrent());
    leftMotorCurrent.append(leftMotor.getOutputCurrent());

    rightMotorVelocity.append(rightMotor.getEncoder().getVelocity());
    leftMotorVelocity.append(leftMotor.getEncoder().getVelocity());
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
