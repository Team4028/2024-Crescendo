// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Feeder extends SubsystemBase {
  CANSparkFlex feederMotor; //TODO: log this
  static Feeder instance;

  private Feeder() {
    feederMotor = new CANSparkFlex(11/* BAD */, MotorType.kBrushless);
  }

private final void runFeederMotor(double vBus){
  feederMotor.set(vBus);
}

public Command runFeederMotorCommand(double vBus){
  return runOnce(() -> runFeederMotor(vBus));
}

public static Feeder getInstance() {
  if (instance == null) instance = new Feeder();
  return instance;
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //TODO: Maybe print values to sd
  }
}
