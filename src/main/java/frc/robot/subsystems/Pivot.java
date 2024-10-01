package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.ArbFFUnits;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.utils.DashboardStore;
import frc.robot.utils.LogStore;

public class Pivot extends SubsystemBase {

	public static class ConversionConstants {
		public static final double SHOOTER_PIVOT_TO_LINEAR_ACTUATOR_PIVOT = 5.5;
		public static final double SHOOTER_PIVOT_TO_LINEAR_ACTUATOR_PIVOT_DY = 4.35261;
		public static final double SHOOTER_PIVOT_TO_TOP_SHOOTER_PIVOT = 18.0;
		public static final double LINEAR_ACTUATOR_INITIAL_LENGTH = 17;
		public static final double LINEAR_ACTUATOR_REDUCTION = 4.0;
		public static final double LINEAR_ACTUATOR_ROTATIONS_TO_INCHES_SCALAR = 0.472;
	}

	private final CANSparkMax motor;
	private final RelativeEncoder encoder;
	private final SparkPIDController pid;
	private final ArmFeedforward armFF;

	private final Timer zeroTimer;
	private final double ZERO_TIMER_THRESHOLD = 0.2; // 10 scans
	private final double ZERO_VELOCITY_THRESHOLD = 200;

	private static final int CAN_ID = 13;

	private boolean atHome = true;

	public final static double MAX_POSITION = 60.0;
	public final static double MIN_POSITION = 1.0;

	public final static double HOLD_POSITION = MIN_POSITION;

	public final static double HARD_STOP = 58.3;// 59.4;
	public final static double STAGE_PIVOT = 54.;

	private static final double IN_POSITION_THRESHOLD = 0.9;

	private final static double INCIDENT_OFFSET = (Math.PI / 2)
			- Math.acos(ConversionConstants.SHOOTER_PIVOT_TO_LINEAR_ACTUATOR_PIVOT_DY
					/ ConversionConstants.SHOOTER_PIVOT_TO_LINEAR_ACTUATOR_PIVOT);

	/* PID */
	private class PIDConstants {
		private final static double kP = 0.08;
		private final static double kD = 0.5;

		private final static double MIN_OUTPUT = -0.80;
		private final static double MAX_OUTPUT = 0.85;
	}

	private final double FEEDFORWARD_THRESHOLD = 0.1;

	/* Current Limit */
	private final static int CURRENT_LIMIT = 80;

	/* Ramp */
	private final static double RAMP_RATE = 0.25;

	private double targetPosition = 1.0;

	private boolean isVbus = true;

	public Pivot() {
		/* ======== */
		/* MOTAHHHH */
		/* ======== */
		motor = new CANSparkMax(CAN_ID, MotorType.kBrushless);
		motor.setInverted(true);

		encoder = motor.getEncoder();
		pid = motor.getPIDController();
		pid.setFeedbackDevice(encoder);

		// armFF = new ArmFeedforward(0.41062, 0.081135, 2.8182E-05);
		armFF = new ArmFeedforward(0.20531, 0.081135, 2.8182E-05);

		encoder.setMeasurementPeriod(16);
		encoder.setAverageDepth(2);

		/* Logs */
		LogStore.add("/Pivot/Current", motor::getOutputCurrent);
		LogStore.add("/Pivot/Voltage", motor::getAppliedOutput);
		LogStore.add("/Pivot/Velocity", encoder::getVelocity);
		LogStore.add("/Pivot/Position", encoder::getPosition);

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

		/* CAN Bus */
		motor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20);
		motor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
		motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);
		motor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 101);
		motor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 102);
		motor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 103);
		motor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 104);
		motor.setPeriodicFramePeriod(PeriodicFrame.kStatus7, 106);

		/* Burn Flash */
		motor.burnFlash();

		/* ===== */
		/* TIMER */
		/* ===== */
		zeroTimer = new Timer();

		/* Dashboard */
		DashboardStore.add("Pivot Position", () -> convertEncoderToRadians(getPosition()));
		DashboardStore.add("Pivot Native Position", this::getPosition);
		DashboardStore.add("Pivot Current", motor::getOutputCurrent);
		DashboardStore.add("Pivot Target", () -> targetPosition);
		DashboardStore.add("Pivot Velocity", encoder::getVelocity);
	}

	private double convertEncoderToRadians(double encoder) {
		double sideC = (encoder / ConversionConstants.LINEAR_ACTUATOR_REDUCTION
				* ConversionConstants.LINEAR_ACTUATOR_ROTATIONS_TO_INCHES_SCALAR)
				+ ConversionConstants.LINEAR_ACTUATOR_INITIAL_LENGTH;

		double angle = Math.acos((Math.pow(ConversionConstants.SHOOTER_PIVOT_TO_LINEAR_ACTUATOR_PIVOT, 2)
				+ Math.pow(ConversionConstants.SHOOTER_PIVOT_TO_TOP_SHOOTER_PIVOT, 2) - sideC * sideC //
		) / (2 * ConversionConstants.SHOOTER_PIVOT_TO_LINEAR_ACTUATOR_PIVOT
				* ConversionConstants.SHOOTER_PIVOT_TO_TOP_SHOOTER_PIVOT));

		return angle - INCIDENT_OFFSET;
	}

	// ==================================
	// PIVOT COMMANDS
	// ==================================

	public void zeroEncoder() {
		encoder.setPosition(0.);
	}

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
		if (position == HOLD_POSITION)
			atHome = true;
		else if (position == HARD_STOP || position == STAGE_PIVOT)
			atHome = false;
		pid.setReference(position, ControlType.kPosition);
	}

	public Command runToPositionCommand(DoubleSupplier position) {
		return runOnce(() -> runToPosition(position.getAsDouble()));
	}

	public Command runToPositionCommand(double position) {
		return runOnce(() -> runToPosition(position));
	}

	public Command runToHomeCommand() {
		return runToPositionCommand(HOLD_POSITION);
	}

	public Command runToTrapCommand() {
		return runToPositionCommand(STAGE_PIVOT).andThen(runToPositionCommand(HARD_STOP));
	}

	public Command runToClimbCommand() {
		return runToTrapCommand();
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
			isVbus = true;
		}).andThen(runMotorCommand(-0.05).repeatedly()
				.until(() -> zeroTimer.get() >= ZERO_TIMER_THRESHOLD
						&& Math.abs(encoder.getVelocity()) < ZERO_VELOCITY_THRESHOLD))
				.andThen(runMotorCommand(0.).alongWith(Commands.runOnce(() -> zeroTimer.stop())))
				.andThen(runOnce(() -> encoder.setPosition(0.)))
				.andThen(new WaitCommand(0.25).andThen(runToPositionCommand(HOLD_POSITION)))
				.andThen(runOnce(() -> isVbus = false));

	}

	public boolean inPosition() {
		return Math.abs(getPosition() - targetPosition) < IN_POSITION_THRESHOLD;
	}

	public boolean getIsAtHome() {
		return atHome;
	}

	public BooleanSupplier inPositionSupplier() {
		return () -> (inPosition());
	}

	@Override
	public void periodic() {
		double error = targetPosition - getPosition();

		if (!isVbus && Math.abs(error) > FEEDFORWARD_THRESHOLD)
			pid.setReference(targetPosition, ControlType.kPosition, 0,
					armFF.calculate(convertEncoderToRadians(encoder.getPosition()),
							(error / STAGE_PIVOT) * PIDConstants.MAX_OUTPUT * 6000.),
					ArbFFUnits.kVoltage);
	}
}
