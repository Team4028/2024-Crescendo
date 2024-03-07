package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants;

public class Pivot extends SubsystemBase {

    public static class ConversionConstants {
        public static final double SHOOTER_PIVOT_TO_LINEAR_ACTUATOR_PIVOT = 5.5;
        public static final double SHOOTER_PIVOT_TO_TOP_SHOOTER_PIVOT = 19.5;
        public static final double LINEAR_ACTUATOR_INITIAL_LENGTH = 17;
        public static final double LINEAR_ACTUATOR_REDUCTION = 4.0;
        public static final double LINEAR_ACTUATOR_ROTATIONS_TO_INCHES_SCALAR = 0.472;
    }

    private final CANSparkMax motor;
    private final RelativeEncoder encoder;
    private final SparkPIDController pid;
    private final ArmFeedforward armFF;
    private final SysIdRoutine sysIdRoutine;

    private final DataLog log;
    private final DoubleLogEntry currentLog, velocityLog, voltageLog, positionLog;
    private final StringLogEntry sysIDTestMode;

    private final Timer zeroTimer;
    private final double ZERO_TIMER_THRESHOLD = 0.14; // 7 scans
    private final double ZERO_VELOCITY_THRESHOLD = 0.8;

    private static final int CAN_ID = 13;

    public final static double MAX_POSITION = 64.0;
    public final static double MIN_POSITION = 1.;

    public final static double CLIMB_POSITION = MAX_POSITION - 5.;
    public final static double HOLD_POSITION = MIN_POSITION;
    public final static double TRAP_POSITION = 42.;

    private final static double ANGULAR_CANSTANT = 0.9123; // rad

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

        armFF = new ArmFeedforward(0, 0, 0);

        // Logging *Wo-HO!!
        log = DataLogManager.getLog();
        currentLog = new DoubleLogEntry(log, "/Pivot/Current");
        voltageLog = new DoubleLogEntry(log, "/Pivot/Voltage");
        velocityLog = new DoubleLogEntry(log, "/Pivot/Velocity");
        positionLog = new DoubleLogEntry(log, "/Pivot/Position");
        sysIDTestMode = new StringLogEntry(log, "/Pivot/SysID test mode");

        sysIdRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(null, null, null, (state) -> sysIDTestMode.append(state.toString())),
                new SysIdRoutine.Mechanism(
                        (Measure<Voltage> volts) -> motor.setVoltage(volts.in(Units.Volts)), null,
                        this));

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
    }

    public Command runQuasi(Direction dir) {
        return sysIdRoutine.quasistatic(dir);
    }

    public Command runDyn(Direction dir) {
        return sysIdRoutine.dynamic(dir);
    }

    private double convertEncoderToRadians(double encoder) {
        double sideC = (encoder / ConversionConstants.LINEAR_ACTUATOR_REDUCTION
                * ConversionConstants.LINEAR_ACTUATOR_ROTATIONS_TO_INCHES_SCALAR)
                + ConversionConstants.LINEAR_ACTUATOR_INITIAL_LENGTH;

        double angle = Math.acos((ConversionConstants.SHOOTER_PIVOT_TO_LINEAR_ACTUATOR_PIVOT
                * ConversionConstants.SHOOTER_PIVOT_TO_LINEAR_ACTUATOR_PIVOT
                + ConversionConstants.SHOOTER_PIVOT_TO_TOP_SHOOTER_PIVOT
                        * ConversionConstants.SHOOTER_PIVOT_TO_TOP_SHOOTER_PIVOT
                - sideC * sideC)
                / (2 * ConversionConstants.SHOOTER_PIVOT_TO_LINEAR_ACTUATOR_PIVOT
                        * ConversionConstants.SHOOTER_PIVOT_TO_TOP_SHOOTER_PIVOT));

        return angle - ANGULAR_CANSTANT;
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
        velocityLog.append(convertEncoderToRadians(encoder.getVelocity()));
        positionLog.append(convertEncoderToRadians(getPosition()));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Pivot Position", convertEncoderToRadians(getPosition()));
        SmartDashboard.putNumber("Pivot Current", motor.getOutputCurrent());
        SmartDashboard.putNumber("Pivot Target", targetPosition);

        // pid.setReference(targetPosition, ControlType.kPosition, 0, armFF.calculate(
        //         convertEncoderToRadians(encoder.getPosition()), convertEncoderToRadians(encoder.getVelocity())));
    }
}
