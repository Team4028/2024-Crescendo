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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    /** Creates a new Climber. */
    private final TalonFX motor;
    private final DataLog log;
    private final DoubleLogEntry vbusLog, currentLog, positionLog, velocityLog;

    private final Timer zeroTimer;

    private static final double ZERO_TIMER_THRESHOLD = 0.1; // 5 scans
    private static final double ZERO_VELOCITY_THRESHOLD = 5;

    private static final int CAN_ID = 150;
    private static final int SUPPLY_CURRENT_LIMIT = 80;
    private static final int STATOR_CURRENT_LIMIT = 100;

    // ====================== //
    /* MOTION MAGIC CONSTANTS */
    // ====================== //
    private static final double CRUISE_VELOCITY = 20.;
    private static final double ACCELERATION = 40.;
    private static final double JERK = 400.;

    private final Slot0Configs pid = new Slot0Configs()
            .withKP(2.)
            .withKI(0.0)
            .withKD(0.0); // needs tuning

    private final PositionVoltage positionRequest = new PositionVoltage(
            0,
            0.,
            true,
            0,
            0,
            true,
            false,
            false);

    private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(
            0,
            true,
            0,
            0,
            true,
            false,
            false);

    public enum ClimberPositions {
        HOME(1.),
        DOWN_ONE(50.),
        DOWN_TWO(40.),
        READY(65.);

        public double Position;

        private ClimberPositions(double position) {
            this.Position = position;
        }
    }

    public Climber() {
        motor = new TalonFX(CAN_ID);
        
        motor.setNeutralMode(NeutralModeValue.Brake);
        motor.setInverted(false);

        // current limits
        motor.getConfigurator().apply(
                new CurrentLimitsConfigs().withSupplyCurrentLimit(SUPPLY_CURRENT_LIMIT)
                        .withStatorCurrentLimit(STATOR_CURRENT_LIMIT));

        /* Motion Magic */
        MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
        motionMagicConfigs.MotionMagicCruiseVelocity = CRUISE_VELOCITY;
        motionMagicConfigs.MotionMagicAcceleration = ACCELERATION;
        motionMagicConfigs.MotionMagicJerk = JERK;

        motor.getConfigurator().apply(motionMagicConfigs);

        // pid config
        motor.getConfigurator().apply(pid);

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

    public Command zeroCommand() {
        return runOnce(() -> {
            zeroTimer.restart();
        })
                .andThen(runMotorCommand(-0.1).repeatedly()
                        .until(() -> zeroTimer.get() >= ZERO_TIMER_THRESHOLD
                                && Math.abs(motor.getVelocity().getValueAsDouble()) < ZERO_VELOCITY_THRESHOLD))
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
        motor.setControl(motionMagicRequest.withPosition(ClimberPositions.HOME.Position - 2.0)); //subtract 5 from this probably
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
        // // This method will be called once per scheduler run
        // SmartDashboard.putNumber("Climber Position", motor.getPosition().getValueAsDouble());
        // SmartDashboard.putNumber("Climber Current", motor.getStatorCurrent().getValueAsDouble());
    }
}
