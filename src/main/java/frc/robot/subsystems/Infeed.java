// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.playingwithfusion.TimeOfFlight;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Infeed extends SubsystemBase {
  private static Infeed instance;
  private TimeOfFlight tofSensor; //TODO: Add logging
  private CANSparkFlex infeedMotor; //TODO: Add logging

  private final double RANGE_THRESH = 100;

  /** Creates a new SensorMotor. */
  private Infeed() {
    tofSensor = new TimeOfFlight(1);
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

  //public Command runMotorWithSensorCommand() {
   // return runOnce(this::runMotorWithSensor);
  //}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Infeed TOF", tofSensor.getRange());
  }

  public static Infeed getInstance() {
    if (instance == null) instance = new Infeed();
    return instance;
  }
}
