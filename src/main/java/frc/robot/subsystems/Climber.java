// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    /** Creates a new Climber. */
    private final TalonFX motor;
    private final DutyCycleEncoder encoder;

    private final DataLog log;
    private final DoubleLogEntry vbusLog, currentLog, positionLog, velocityLog;

    private final Timer zeroTimer;

    private static final double ZERO_TIMER_THRESHOLD = 0.2; // 10 scans
    private static final double ZERO_CURRENT_THRESHOLD = 4.5;

    private boolean oneShot = false;

    private static final double ZERO_VBUS = -0.1;
    private static final double ZERO_ABSOLUTE_ENCODER_POSITION = .447;
    private static final double ABSOLUTE_ENCODER_ROT_TO_MOTOR_ROT = 390.6;

    private static final int CAN_ID = 15;

    /* Configs */
    private final Slot0Configs pidConfigs = new Slot0Configs()
            .withKP(0.66)
            .withKI(0.0)
            .withKD(0.0); // needs tuning

    private final MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(40.)
            .withMotionMagicAcceleration(80.)
            .withMotionMagicJerk(800.);

    private final CurrentLimitsConfigs currentConfigs = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(100.)
            .withSupplyCurrentLimit(80.);

    /* Requests */
    private final PositionVoltage positionRequest = new PositionVoltage(0.)
            .withEnableFOC(true)
            .withOverrideBrakeDurNeutral(true);

    private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0.)
            .withEnableFOC(true)
            .withOverrideBrakeDurNeutral(true);

    // TODO: these are incorrect rn
    public enum ClimberPositions {
        CLIMB(-4.),
        HOME(0.),
        DOWN_ONE(156.),
        DOWN_TWO(125.),
        READY(175.);

        public double Position;

        private ClimberPositions(double position) {
            this.Position = position;
        }
    }

    public Climber() {
        motor = new TalonFX(CAN_ID);
        encoder = new DutyCycleEncoder(9);

        motor.setNeutralMode(NeutralModeValue.Brake);
        motor.setInverted(true);

        /* ======= */
        /* CONFIGS */
        /* ======= */
        motor.getConfigurator().apply(motionMagicConfigs);
        motor.getConfigurator().apply(pidConfigs);
        motor.getConfigurator().apply(currentConfigs);

        zeroTimer = new Timer();

        log = DataLogManager.getLog();
        vbusLog = new DoubleLogEntry(log, "/Climber/Vbus");
        currentLog = new DoubleLogEntry(log, "/Climber/Current");
        positionLog = new DoubleLogEntry(log, "/Climber/Position");
        velocityLog = new DoubleLogEntry(log, "/Climber/Velocity");
    }

    public void runMotor(double vBus) {
        motor.set(vBus);
    }

    public Command runMotorCommand(double vBus) {
        return runOnce(() -> runMotor(vBus));
    }

    public Command setEncoderZeroCmd() {
        return runOnce(() -> motor.setPosition(0.0));
    }

    public Command zeroCommand() {
        return runOnce(() -> {
            zeroTimer.restart();
        })
                .andThen(runMotorCommand(ZERO_VBUS).repeatedly()
                        .until(() -> zeroTimer.get() >= ZERO_TIMER_THRESHOLD
                                && Math.abs(motor.getStatorCurrent().getValueAsDouble()) > ZERO_CURRENT_THRESHOLD))
                .andThen(runMotorCommand(0.),
                        Commands.runOnce(() -> zeroTimer.stop()),
                        Commands.runOnce(() -> motor.setPosition(0.0)));
    }


    
    public void runToPosition(double position) {
        motor.setControl(positionRequest.withPosition(position));
    }

    public Command runToPositionCommand(double position) {
        return runOnce(() -> runToPosition(position));
    }

    public void climb() {
        motor.setControl(motionMagicRequest.withPosition(ClimberPositions.CLIMB.Position));
    }

    public Command climbCommand() {
        return runOnce(this::climb);
    }

    public Command runToPositionCommand(ClimberPositions position) {
        return runToPositionCommand(position.Position);
    }

    public void logValues() {
        vbusLog.append(motor.get());
        currentLog.append(motor.getStatorCurrent().getValueAsDouble());
        positionLog.append(motor.getPosition().getValueAsDouble());
        velocityLog.append(motor.getVelocity().getValueAsDouble());
    }

    @Override
    public void periodic() {
        if (!oneShot) {
            motor.setPosition((encoder.getAbsolutePosition() - ZERO_ABSOLUTE_ENCODER_POSITION)
                    * ABSOLUTE_ENCODER_ROT_TO_MOTOR_ROT);
            oneShot = true;
        }

        // // This method will be called once per scheduler run
        SmartDashboard.putNumber("Climber Position", motor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Climber Current", motor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Climber Velocity", motor.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Abs Endocer Pos", encoder.getAbsolutePosition());
    }
}
