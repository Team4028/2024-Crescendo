// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PrototypeShooter extends SubsystemBase {
  static PrototypeShooter instance;
  CANSparkFlex shooterMotorA, shooterMotorB, feederMotor;

  DataLog log;
  public DoubleLogEntry shooterMotorACurrent, shooterMotorBCurrent, shooterMotorAVelocity, shooterMotorBVelocity;

  public PrototypeShooter() {
    log = DataLogManager.getLog();
    configureLogs();
    feederMotor = new CANSparkFlex(44, MotorType.kBrushless);
    shooterMotorA = new CANSparkFlex(12, MotorType.kBrushless);
    shooterMotorB = new CANSparkFlex(11, MotorType.kBrushless);
  }

  private void spinFeederMotor(double vBus) {
    feederMotor.set(vBus);
  }

  public Command spinFeederMotorCommand(double vBus) {
    return runOnce(() -> spinFeederMotor(vBus));
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

  public static PrototypeShooter getInstance() {
    if (instance == null) {
      instance = new PrototypeShooter();
    }

    return instance;
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
