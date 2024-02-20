// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    /** Creates a new Climber. */
    private static Climber instance;
    CANSparkMax climberMotorA, climberMotorB;
    RelativeEncoder encoderA, encoderB;
    DataLog log;
    DoubleLogEntry aVBus, bVBus, aCurrent, bCurrent, aPos, bPos, aVel, bVel;

    public Climber() {
        climberMotorA = new CANSparkMax(0, MotorType.kBrushless);
        climberMotorB = new CANSparkMax(1, MotorType.kBrushless);

        encoderA = climberMotorA.getEncoder();
        encoderB = climberMotorB.getEncoder();
        controllerA = climberMotorA.getPIDController();
        controllerB = climberMotorB.getPIDController();

        controllerA.setP(0);
        controllerA.setI(0);
        controllerA.setD(0);

        controllerB.setP(0);
        controllerB.setI(0);
        controllerB.setD(0);

    }

    private void spinClimberMotorA(double vBus) {
        climberMotorA.set(vBus);
    }>>>>>>>

    fe6a770 (Climber Pid additions commit)

        log = DataLogManager.getLog();
        aVBus = new DoubleLogEntry(log, "A/VBus");
        bVBus = new DoubleLogEntry(log, "B/VBus");
        aCurrent = new DoubleLogEntry(log, "A/Current");
        bCurrent = new DoubleLogEntry(log, "B/Current");
        aPos = new DoubleLogEntry(log, "A/Pos");
        bPos = new DoubleLogEntry(log, "B/Pos");
        aVel = new DoubleLogEntry(log, "A/Vel");
        bVel = new DoubleLogEntry(log, "B/Vel");

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

    public void logMotorValues() {
        aVBus.append(climberMotorA.get());
        bVBus.append(climberMotorB.get());
        aCurrent.append(climberMotorA.getOutputCurrent());
        bCurrent.append(climberMotorB.getOutputCurrent());
        aPos.append(encoderA.getPosition());
        bPos.append(encoderB.getPosition());
        aVel.append(encoderA.getVelocity());
        bVel.append(encoderB.getVelocity());
    }

    public static Climber getInstance() {
        if (instance == null) {
            instance = new Climber();
        }
        return instance;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("Climber Motor A Position", encoderA.getPosition());
        SmartDashboard.putNumber("Climber Motor B Position", encoderB.getPosition());
    }=======

    public Command climberRunToPositionCommand20() {
        return run(() -> climberRunToPosition(20));
    }

    public void climberRunToPosition(double positionInInches) {
        controllerA.setReference(inchesToNativeUnits(positionInInches), ControlType.kPosition);
    }

    // May need these two for setting positions, just keep them for now
    // Native units are inches
    private static int inchesToNativeUnits(double positionInInches) {
        int nativeUnits = (int) (positionInInches * NATIVE_UNITS_PER_INCH_CONVERSION);
        return nativeUnits;
    }

    private static double nativeUnitsToInches(double nativeUnitsMeasure) {
        double positionInInches = nativeUnitsMeasure / NATIVE_UNITS_PER_INCH_CONVERSION;
        return positionInInches;
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Climber Motor A Position", encoderA.getPosition());
    SmartDashboard.putNumber("Climber Motor B Position", encoderB.getPosition());
  }>>>>>>>

    fe6a770 (Climber Pid additions commit)
}
