package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

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
    private final CANSparkMax motor;
    private final RelativeEncoder encoder;
    private final SparkPIDController pid;

    private final DataLog log;
    private final DoubleLogEntry currentLog, velocityLog, voltageLog, positionLog;

    private final Timer zeroTimer;
    private final double ZERO_TIMER_THRESHOLD = 0.14; // 7 scans
    private final double ZERO_VELOCITY_THRESHOLD = 0.8;

    private static final int CAN_ID = 13;

    public final static double MAX_POSITION = 64.0;
    public final static double MIN_POSITION = 1.;

    public final static double CLIMB_POSITION = MAX_POSITION - 5.;
    public final static double HOLD_POSITION = MIN_POSITION;
    public final static double TRAP_POSITION = 42.;

    /* PID */
    private class PIDConstants {
        // TODO: three different zones
        private final static double kP = 0.05;
        private final static double kD = 0.5;

        private final static double MIN_OUTPUT = -0.5;
        private final static double MAX_OUTPUT = 0.7;
    }

    /* Current Limit */
    private final static int CURRENT_LIMIT = 80;

    /* Ramp */
    private final static double RAMP_RATE = 1.0;

    private double targetPosition;

    public Pivot() {
        /* ======== */
        /* MOTAHHHH */
        /* ======== */
        motor = new CANSparkMax(CAN_ID, MotorType.kBrushless);
        motor.setInverted(true);

        encoder = motor.getEncoder();
        pid = motor.getPIDController();
        pid.setFeedbackDevice(encoder);

        /* ======= */
        /* PID!!!! */
        /* ======= */
        
        pid.setP(PIDConstants.kP);
        pid.setD(PIDConstants.kD);
        pid.setOutputRange(PIDConstants.MIN_OUTPUT, PIDConstants.MAX_OUTPUT);

        // ============= //
        /* Other Configs */
        // ============= //

        motor.setSmartCurrentLimit(CURRENT_LIMIT);
        motor.setClosedLoopRampRate(RAMP_RATE);
        motor.setSoftLimit(SoftLimitDirection.kForward, (float) MAX_POSITION);

        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20);
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 101);
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 102);
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 103);
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 104);
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus7, 106);

        targetPosition = 1.;

        /* ===== */
        /* TIMER */
        /* ===== */
        zeroTimer = new Timer();

        // Logging *Wo-HO!!
        log = DataLogManager.getLog();
        currentLog = new DoubleLogEntry(log, "/Pivot/Current");
        voltageLog = new DoubleLogEntry(log, "/Pivot/Voltage");
        velocityLog = new DoubleLogEntry(log, "/Pivot/Velocity");
        positionLog = new DoubleLogEntry(log, "/Pivot/Position");
    }
    // ==================================
    // PIVOT COMMANDS
    // ==================================

    public double getPosition() {
        return encoder.getPosition();
    }

    public void runMotor(double vBus) {
        motor.set(vBus);
    }

    public Command runMotorCommand(double vBus) {
        return runOnce(() -> runMotor(vBus));
    }

    public void runToPosition(double position) {
        targetPosition = position;
        pid.setReference(position, ControlType.kPosition);
    }

    public Command runToPositionCommand(double position) {
        return runOnce(() -> runToPosition(position));
    }

    public Command runToHomeCommand() {
        return runToPositionCommand(HOLD_POSITION);
    }

    public Command runToTrapCommand() {
        return runToPositionCommand(TRAP_POSITION);
    }

    public Command runToClimbCommand() {
        return runToPositionCommand(CLIMB_POSITION);
    }

    public void holdPosition() {
        runToPosition(encoder.getPosition());
    }

    public Command holdPositionCommand() {
        return runOnce(this::holdPosition).andThen(Commands.idle());
    }

    public Command zeroCommand() {
        return runOnce(() -> {
            zeroTimer.restart();
        })
                .andThen(runMotorCommand(-0.1).repeatedly()
                        .until(() -> zeroTimer.get() >= ZERO_TIMER_THRESHOLD
                                && Math.abs(encoder.getVelocity()) < ZERO_VELOCITY_THRESHOLD))
                .andThen(runMotorCommand(0.).alongWith(Commands.runOnce(() -> zeroTimer.stop())))
                .andThen(runOnce(() -> encoder.setPosition(0.)))
                .andThen(new WaitCommand(0.25).andThen(runToPositionCommand(HOLD_POSITION)));

    }

    // public boolean inPosition() {
    // return Math.abs(motor.getPosition() - targetPosition) < 1.0;
    // }

    public BooleanSupplier inPositionSupplier() {
        return () -> (Math.abs(getPosition() - targetPosition) < 2.0);
    }

    public void logValues() {
        currentLog.append(motor.getOutputCurrent());
        voltageLog.append(motor.getAppliedOutput() * motor.getBusVoltage());
        velocityLog.append(encoder.getVelocity());
        positionLog.append(getPosition());
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Pivot Position", getPosition());
        SmartDashboard.putNumber("Pivot Current", motor.getOutputCurrent());
        SmartDashboard.putNumber("Pivot Target", targetPosition);
    }
}
