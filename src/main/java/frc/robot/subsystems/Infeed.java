// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.playingwithfusion.TimeOfFlight;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Infeed extends SubsystemBase {
  private static Infeed instance;
  private TimeOfFlight tofSensor;
  private TalonFX infeedMotor;

  private final double RANGE_THRESH = 100;

  DataLog log;
  public DoubleLogEntry infeedMotorCurrent, infeedMotorVelocity;

  /** Creates a new SensorMotor. */
  private Infeed() {
    log = DataLogManager.getLog();
    configureLogs();
    tofSensor = new TimeOfFlight(1);
    infeedMotor = new TalonFX(18);
    infeedMotor.setInverted(true);
    infeedMotor.getConfigurator().apply(
        new CurrentLimitsConfigs()
            .withStatorCurrentLimit(60)
            .withSupplyCurrentLimit(60)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimitEnable(true));
  }

  // public void logToDash() {
  // SmartDashboard.putNumber("tof distance", sensor.getRange());

  // public void runMotorWithSensor() {

  // if (sensorRun.getRange() > 100) {

  // } else {

  // }
  // }

  public boolean hasGamePiece() {
    return tofSensor.getRange() < RANGE_THRESH;
  }

  public BooleanSupplier hasGamePieceSupplier() {
    return this::hasGamePiece;
  }

  private void runInfeedMotor(double vBus) {
    infeedMotor.set(vBus);
  }

  public Command runInfeedMotorCommand(double vBus) {
    return runOnce(() -> runInfeedMotor(vBus));
  }

  private void configureLogs() {
    infeedMotorCurrent = new DoubleLogEntry(log, "Current");
    infeedMotorVelocity = new DoubleLogEntry(log, "Velocity");
  }

  public void logValues() {
    infeedMotorCurrent.append(infeedMotor.getSupplyCurrent().getValueAsDouble());

    infeedMotorVelocity.append(infeedMotor.getVelocity().getValueAsDouble());
  }

  // public Command runMotorWithSensorCommand() {
  // return runOnce(this::runMotorWithSensor);
  // }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Infeed TOF", tofSensor.getRange());
  }

  public static Infeed getInstance() {
    if (instance == null)
      instance = new Infeed();
    return instance;
  }
}
