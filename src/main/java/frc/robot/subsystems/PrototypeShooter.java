// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PrototypeShooter extends SubsystemBase {
  static PrototypeShooter instance;
  CANSparkFlex motorA;
  CANSparkFlex motorB;

  /** Creates a new ExampleSubsystem. */
  public PrototypeShooter() {
    motorA = new CANSparkFlex(12, MotorType.kBrushless);
    motorB = new CANSparkFlex(11, MotorType.kBrushless);
  }

  private void spinMotorA(double vBus) {
    motorA.set(vBus);
    
  }

  private void spinMotorB(double vBus) {
    motorB.set(vBus);
    
  }
  public Command spinMotorACommand(double vBus) {
    return runOnce(() -> spinMotorA(vBus));
  }

  public Command spinMotorBCommand(double vBus) {
    return runOnce(() -> spinMotorB(vBus));
  }
  public static PrototypeShooter getInstance() {
    if(instance == null) {
      instance = new PrototypeShooter();
    }

    return instance;
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.

    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a
   * digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
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
