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
  CANSparkFlex shooterMotorA;
  CANSparkFlex shooterMotorB;
  DataLog log;
  public DoubleLogEntry shooterMotorACurrent, shooterMotorBCurrent, shooterMotorAVelocity, shooterMotorBVelocity;

  /** Creates a new ExampleSubsystem. */
  public PrototypeShooter() {
    shooterMotorA = new CANSparkFlex(12, MotorType.kBrushless);
    shooterMotorB = new CANSparkFlex(11, MotorType.kBrushless);
    
    log = DataLogManager.getLog();

    shooterMotorB.follow(shooterMotorA);

    shooterMotorB.setInverted(true);
    shooterMotorA.setInverted(false);
  }

  private void spinMotor(double vBus) {
    shooterMotorA.set(vBus);

  }

  public Command spinMotorACommand(double vBus) {
    return runOnce(() -> spinMotor(vBus));
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
