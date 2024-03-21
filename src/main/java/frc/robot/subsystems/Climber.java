// // // Copyright (c) FIRST and other WPILib contributors.
// // // Open Source Software; you can modify and/or share it under the terms of
// // // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import java.util.function.BooleanSupplier;

// import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
// import com.ctre.phoenix6.configs.MotionMagicConfigs;
// import com.ctre.phoenix6.configs.Slot0Configs;
// import com.ctre.phoenix6.controls.MotionMagicVoltage;
// import com.ctre.phoenix6.controls.PositionVoltage;
// import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.signals.NeutralModeValue;

// import edu.wpi.first.util.datalog.DataLog;
// import edu.wpi.first.util.datalog.DoubleLogEntry;
// import edu.wpi.first.wpilibj.DataLogManager;
// import edu.wpi.first.wpilibj.DutyCycleEncoder;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.utils.DashboardStore;

// public class Climber extends SubsystemBase {
//     /** Creates a new Climber. */
//     private final TalonFX motor;
//     private final DutyCycleEncoder encoder;

//     private final DataLog log;
//     private final DoubleLogEntry vbusLog, currentLog, positionLog, velocityLog;

//     private double targetPosition = 0.;

//     private static final double ZERO_ABSOLUTE_ENCODER_POSITION = 0.9375; //.933;
//     private static final double ABSOLUTE_ENCODER_ROT_TO_MOTOR_ROT = 287.5;

//     private static final int CAN_ID = 15;

//     /* Configs */
//     private final Slot0Configs pidConfigs = new Slot0Configs()
//             .withKP(0.66)
//             .withKI(0.0)
//             .withKD(0.0); // needs tuning

//     private final MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs()
//             .withMotionMagicCruiseVelocity(50.)
//             .withMotionMagicAcceleration(100.)
//             .withMotionMagicJerk(1000.);

//     private final CurrentLimitsConfigs currentConfigs = new CurrentLimitsConfigs()
//             .withStatorCurrentLimit(100.)
//             .withSupplyCurrentLimit(80.);

//     /* Requests */
//     private final PositionVoltage positionRequest = new PositionVoltage(0.)
//             .withEnableFOC(true)
//             .withOverrideBrakeDurNeutral(true);

//     private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0.)
//             .withEnableFOC(true)
//             .withOverrideBrakeDurNeutral(true);

//     public enum ClimberPositions {
//         CLIMB(-10.),
//         HOME(0.),
//         TENSION(100.),
//         DOWN_TWO(115.), // unused
//         READY(133.5);

//         public double Position;

//         private ClimberPositions(double position) {
//             this.Position = position;
//         }
//     }

//     public Climber() {
//         /* Setup */
//         motor = new TalonFX(CAN_ID);
//         encoder = new DutyCycleEncoder(9);

//         motor.setNeutralMode(NeutralModeValue.Brake);
//         motor.setInverted(true);

//         /* ======= */
//         /* CONFIGS */
//         /* ======= */
//         motor.getConfigurator().apply(motionMagicConfigs);
//         motor.getConfigurator().apply(pidConfigs);
//         motor.getConfigurator().apply(currentConfigs);

//         /* CAN Bus */
//         motor.getVelocity().setUpdateFrequency(20.);
//         motor.getPosition().setUpdateFrequency(20.);
//         motor.getStatorCurrent().setUpdateFrequency(20.);
//         motor.getDutyCycle().setUpdateFrequency(20.);
//         motor.optimizeBusUtilization();

//         /* Logs */
//         log = DataLogManager.getLog();
//         vbusLog = new DoubleLogEntry(log, "/Climber/Vbus");
//         currentLog = new DoubleLogEntry(log, "/Climber/Current");
//         positionLog = new DoubleLogEntry(log, "/Climber/Position");
//         velocityLog = new DoubleLogEntry(log, "/Climber/Velocity");

//         /* Dashboard */
//         DashboardStore.add("Climber Position", () -> motor.getPosition().getValueAsDouble());
//         DashboardStore.add("Climber Current", () -> motor.getStatorCurrent().getValueAsDouble());
//         DashboardStore.add("Climber Velocity", () -> motor.getVelocity().getValueAsDouble());
//         DashboardStore.add("Absolute Encoder Position", () -> encoder.getAbsolutePosition());
//     }

//     public void runMotor(double vBus) {
//         motor.set(vBus);
//     }

//     public Command runMotorCommand(double vBus) {
//         return runOnce(() -> runMotor(vBus));
//     }

//     public Command setEncoderZeroCmd() {
//         return runOnce(() -> motor.setPosition(0.0));
//     }

//     public void runToPosition(double position) {
//         targetPosition = position;
//         motor.setControl(positionRequest.withPosition(position));
//     }

//     public Command runToPositionCommand(double position) {
//         return runOnce(() -> runToPosition(position));
//     }

//     public void climb() {
//         targetPosition = ClimberPositions.CLIMB.Position;
//         motor.setControl(motionMagicRequest.withPosition(ClimberPositions.CLIMB.Position));
//     }

//     public Command climbCommand() {
//         return runOnce(this::climb);
//     }

//     public void runToPosition(ClimberPositions position) {
//         runToPosition(position.Position);
//     }

//     public Command runToPositionCommand(ClimberPositions position) {
//         return runToPositionCommand(position.Position);
//     }

//     public boolean inPosition() {
//         return Math.abs(targetPosition - motor.getPosition().getValueAsDouble()) < 1.5;
//     }

//     public BooleanSupplier inPositionSupplier() {
//         return () -> inPosition();
//     }

//     public void logValues() {
//         vbusLog.append(motor.get());
//         currentLog.append(motor.getStatorCurrent().getValueAsDouble());
//         positionLog.append(motor.getPosition().getValueAsDouble());
//         velocityLog.append(motor.getVelocity().getValueAsDouble());
//     }

//     public boolean encoderReady() {
//         return encoder.isConnected() && Math.abs(encoder.getAbsolutePosition()) > 0.01;
//     }

//     public void reZero() {
//         double newPosition = (encoder.getAbsolutePosition() - ZERO_ABSOLUTE_ENCODER_POSITION)
//                 * ABSOLUTE_ENCODER_ROT_TO_MOTOR_ROT;
//         motor.setPosition(newPosition);

//         if (newPosition < -10. || Math.abs(encoder.getAbsolutePosition()) < 0.01) {
//             motor.setPosition(0.);
//         }
//     }

//     @Override
//     public void periodic() {
//         // This method will be called once per scheduler run
//     }
// }
