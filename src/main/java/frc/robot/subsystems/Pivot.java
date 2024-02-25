package frc.robot.subsystems;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class Pivot extends SubsystemBase {
    private final TalonFX motor;

    private final DataLog log;
    private final DoubleLogEntry pivotCurrent, pivotVelocity, pivotVoltage;

    private final Timer zeroTimer;
    private final double ZERO_TIMER_THRESHOLD = 0.14; // 7 scans
    private final double ZERO_VELOCITY_THRESHOLD = 0.2;

    private final PositionVoltage positionRequest = new PositionVoltage(
        0.,
        0.,
        true,
        0.,
        0,
        false,
        false,
        false
    );

    

    private static final int PIVOT_CAN_ID = 13;

    private static class PIDConstants {
        private static final double kP = 0.12;
        
        private static double MIN_VAL = 1.;
        private static double MAX_VAL = 17.5;
    }

    public Pivot() {
        // ==================================
        // PIVOT
        // ==================================
        motor = new TalonFX(PIVOT_CAN_ID);

        motor.setInverted(true);
        motor.setIdleMode(IdleMode.kBrake);

        pivotEncoder = motor.getEncoder();
        pivotPid = motor.getPIDController();

        // ==================================
        // PIVOT PID
        // ==================================
        pivotPid.setP(PIDConstants.kP);

        motor.burnFlash();

        // ==================================
        // TIMER
        // ==================================
        zeroTimer = new Timer();

        // Logging *Wo-HO!!
        log = DataLogManager.getLog();
        pivotCurrent = new DoubleLogEntry(log, "/Pivot/Current");
        pivotVoltage = new DoubleLogEntry(log, "/Pivot/Voltage");
        pivotVelocity = new DoubleLogEntry(log, "/Pivot/Velocity");
    }
    // ==================================
    // PIVOT COMMANDS
    // ==================================

    public void runMotor(double vBus) {
        motor.set(vBus);
    }

    public Command runMotorCommand(double vBus) {
        return runOnce(() -> runMotor(vBus));
    }

    public void runToPosition(double position) {
        pivotPid.setReference(MathUtil.clamp(position, PIDConstants.MIN_VAL, PIDConstants.MAX_VAL),
                ControlType.kPosition);
    }

    public Command runToPositionCommand(double position) {
        return runOnce(() -> runToPosition(position));
    }

    public Command zeroCommand() {
        return runOnce(() -> {
            zeroTimer.restart();
        })
                .andThen(runMotorCommand(-0.1).repeatedly()
                        .until(() -> zeroTimer.get() >= ZERO_TIMER_THRESHOLD
                                && Math.abs(pivotEncoder.getVelocity()) < ZERO_VELOCITY_THRESHOLD))
                .andThen(runMotorCommand(0.).alongWith(Commands.runOnce(() -> zeroTimer.stop())))
                .andThen(runOnce(() -> pivotEncoder.setPosition(0.)))
                .andThen(new WaitCommand(0.5).andThen(runToPositionCommand(PIDConstants.MIN_VAL)));

    }

    public void logValues() {
        pivotCurrent.append(motor.getOutputCurrent());
        pivotVoltage.append(motor.getBusVoltage() * motor.getAppliedOutput());
        pivotVelocity.append(pivotEncoder.getVelocity());
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Pivot pos", pivotEncoder.getPosition());
    }
}
