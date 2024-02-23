package frc.robot.subsystems;

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
    private final CANSparkMax pivotMotor;

    private final RelativeEncoder pivotEncoder;
    private final SparkPIDController pivotPid;

    private final DataLog log;
    private final DoubleLogEntry pivotCurrent, pivotVelocity, pivotVoltage;

    private final Timer zeroTimer;
    private final double ZERO_TIMER_THRESHOLD = 0.14; // 7 scans
    private final double ZERO_VELOCITY_THRESHOLD = 0.2;

    private static final int PIVOT_CAN_ID = 13;

    private static class PIDConstants {
        private static final double kP = 0.03;
        
        private static double MIN_VAL = 4.;
        private static double MAX_VAL = 70.0;
    }

    public Pivot() {
        // ==================================
        // PIVOT
        // ==================================
        pivotMotor = new CANSparkMax(PIVOT_CAN_ID, MotorType.kBrushless);

        pivotMotor.setInverted(true);
        pivotMotor.setIdleMode(IdleMode.kBrake);

        pivotEncoder = pivotMotor.getEncoder();
        pivotPid = pivotMotor.getPIDController();

        // ==================================
        // PIVOT PID
        // ==================================
        pivotPid.setP(PIDConstants.kP);

        pivotMotor.burnFlash();

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
        pivotMotor.set(vBus);
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
        pivotCurrent.append(pivotMotor.getOutputCurrent());
        pivotVoltage.append(pivotMotor.getBusVoltage() * pivotMotor.getAppliedOutput());
        pivotVelocity.append(pivotEncoder.getVelocity());
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Pivot pos", pivotEncoder.getPosition());
    }
}
