// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.playingwithfusion.TimeOfFlight;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Infeed extends SubsystemBase {
  private TimeOfFlight sensorRun;
  private CANSparkFlex infeedMotor;

  /** Creates a new SensorMotor. */
  public Infeed() {
    sensorRun = new TimeOfFlight(1);
    infeedMotor = new CANSparkFlex(1, MotorType.kBrushless);
  }

  // public void logToDash() {
  // SmartDashboard.putNumber("tof distance", sensor.getRange());

  //public void runMotorWithSensor() {

   // if (sensorRun.getRange() > 100) {
      
    //  } else {

   // }
  //}

  public boolean hasGamePiece() {
    return sensorRun.getRange() < 100;
  }

  public BooleanSupplier hasGamePieceSupplier() {
    return () -> hasGamePiece();
  }

  private void runInfeedMotor(double vBus) {
    infeedMotor.set(vBus);
  }
  
  public Command runInfeedMotorCommand(double vBus) {
    return runOnce (() -> runInfeedMotor(vBus));
  }

  //public Command runMotorWithSensorCommand() {
   // return runOnce(this::runMotorWithSensor);
  //}

  @Override
  public void periodic() {
    System.out.println(sensorRun.getRange());
    // This method will be called once per scheduler run
  }
}
