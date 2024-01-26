// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PrototypeShooter extends SubsystemBase {
  static PrototypeShooter instance;
  CANSparkFlex shooterMotorA, shooterMotorB, feederMotor;
  SparkPIDController pidA, pidB;

  DataLog log;
  public DoubleLogEntry shooterMotorACurrent, shooterMotorBCurrent, shooterMotorAVelocity, shooterMotorBVelocity;

  public PrototypeShooter() {
    log = DataLogManager.getLog();
    configureLogs();
    shooterMotorA = new CANSparkFlex(12, MotorType.kBrushless);
    shooterMotorB = new CANSparkFlex(11, MotorType.kBrushless);

    pidA = shooterMotorA.getPIDController();
    pidB = shooterMotorB.getPIDController();

    pidA.setP(0);
    pidA.setI(0);
    pidA.setD(0);
    pidA.setIZone(0);
    pidA.setOutputRange(-0.9, 0.9);

    pidB.setP(0);
    pidB.setI(0);
    pidB.setD(0);
    pidB.setIZone(0);
    pidB.setOutputRange(-0.9, 0.9);
  }

  private void setAToVel(double velRPM) {
    pidA.setReference(velRPM, ControlType.kVelocity);
  }

  private void setBToVel(double velRPM) {
    pidB.setReference(velRPM, ControlType.kVelocity);
  }

  public Command setAToVelCommand(double velRPM) {
    return runOnce(() -> setAToVel(velRPM));
  }

  public Command setBToVelCommand(double velRPM) {
    return runOnce(() -> setBToVel(velRPM));
  }

  private void spinMotorA(double vBus) {
    shooterMotorA.set(vBus);
  }

  private void spinMotorB(double vBus) {
    shooterMotorB.set(-1. * vBus);
  }

  public Command spinMotorACommand(double vBus) {
    return runOnce(() -> spinMotorA(vBus));
  }

  public Command spinMotorBCommand(double vBus) {
    return runOnce(() -> spinMotorB(vBus));
  }

  private void configureLogs() {
    shooterMotorACurrent = new DoubleLogEntry(log, "/A/Current");
    shooterMotorBCurrent = new DoubleLogEntry(log, "/B/Current");
    shooterMotorAVelocity = new DoubleLogEntry(log, "/A/Velocity");
    shooterMotorBVelocity = new DoubleLogEntry(log, "/B/Velocity");

  }

  public void logValues() {
    shooterMotorACurrent.append(shooterMotorA.getOutputCurrent());
    shooterMotorBCurrent.append(shooterMotorB.getOutputCurrent());

    shooterMotorAVelocity.append(shooterMotorA.getEncoder().getVelocity());
    shooterMotorBVelocity.append(shooterMotorB.getEncoder().getVelocity());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
