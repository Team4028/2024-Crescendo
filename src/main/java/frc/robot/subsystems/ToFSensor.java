// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.playingwithfusion.TimeOfFlight;

public class ToFSensor extends SubsystemBase {

 TimeOfFlight shooterToFsensor;

  public ToFSensor() {
 shooterToFsensor = new TimeOfFlight(0);
  }

 private double shooterToFSensorDistance(){
    return shooterToFsensor.getRange();
 }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
