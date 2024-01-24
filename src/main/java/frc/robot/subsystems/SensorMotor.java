// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SensorMotor extends SubsystemBase {
  private TimeOfFlight sensorRun;
  private TalonSRX motor;

  /** Creates a new SensorMotor. */
  public SensorMotor() {
    sensorRun = new TimeOfFlight(1);
    motor = new TalonSRX(2);
  }

 // public void logToDash() {
  //  SmartDashboard.putNumber("tof distance", sensor.getRange());

  public void runMotorWithSensor() {

   

    if (sensorRun.getRange() > 100) {
      motor.set(TalonSRXControlMode.PercentOutput, .1);
    }
    else {
      motor.set(TalonSRXControlMode.PercentOutput, 0);
    }
  }

   public boolean hasGamePiece() {
        return sensorRun.getRange() < 100;
    }

    public BooleanSupplier hasGamePieceSupplier() {
        return () -> hasGamePiece();
    }
  
  public Command runMotor() {
    return run( () -> motor.set(TalonSRXControlMode.PercentOutput, .1));
  }

  public Command stopMotor() {
    return run( () -> motor.set(TalonSRXControlMode.PercentOutput, 0));
  }

  public Command runMotorWithSensorCommand() {
    return runOnce(this::runMotorWithSensor);
  }

  @Override
  public void periodic() {
     System.out.println(sensorRun.getRange());
    // This method will be called once per scheduler run
  }
}
