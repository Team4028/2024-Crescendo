// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  CANSparkMax climberMotorA, climberMotorB;
  RelativeEncoder encoderA, encoderB;

  public Climber() {
    climberMotorA = new CANSparkMax(0, MotorType.kBrushless);
    climberMotorB = new CANSparkMax(1, MotorType.kBrushless);

    encoderA = climberMotorA.getEncoder();
    encoderB = climberMotorB.getEncoder();
  }
  private void spinClimberMotorA(double vBus) {
    climberMotorA.set(vBus);
  }

  private void spinClimberMotorB(double vBus) {
    climberMotorB.set(vBus);
  }

  public Command spinClimberMotorACommand(double vBus) {
    return runOnce(() -> spinClimberMotorA(vBus));
  }

  public Command spinClimberMotorBCommand(double vBus) {
    return runOnce(() -> spinClimberMotorB(vBus));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Climber Motor A Position", encoderA.getPosition());
    SmartDashboard.putNumber("Climber Motor B Position", encoderB.getPosition());
  }
}
