// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.util.function.BooleanSupplier;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

public class NoteSensing {
	private final TimeOfFlight conveyorSensor;
	private final TimeOfFlight shooterSensor;

	private boolean hasInfedCache = false;

	private static final double SHOOTER_THRESHOLD = 100.;
	private static final double CONVEYOR_THRESHOLD = 80.;

	private static final int CONVEYOR_CAN_ID = 21;
	private static final int SHOOTER_CAN_ID = 11;

	/** Creates a new NoteSensing. */
	public NoteSensing() {
		/* Sensor Setup */
		conveyorSensor = new TimeOfFlight(CONVEYOR_CAN_ID);
		conveyorSensor.setRangingMode(RangingMode.Short, 20.0);
		conveyorSensor.setRangeOfInterest(4, 4, 11, 11);

		shooterSensor = new TimeOfFlight(SHOOTER_CAN_ID);
		shooterSensor.setRangingMode(RangingMode.Short, 20.0);
		shooterSensor.setRangeOfInterest(4, 4, 11, 11);

		/* Dashboard */
		DashboardStore.add("ToF Sensor", () -> conveyorSensor.getRange());
		DashboardStore.add("Shooter Sensor", () -> shooterSensor.getRange());
	}

	public boolean hasInfed() {
		return shooterSensor.getRange() < SHOOTER_THRESHOLD && conveyorSensor.getRange() > CONVEYOR_THRESHOLD;
	}

	public BooleanSupplier hasInfedSupplier() {
		return this::hasInfed;
	}

	public boolean conveyorSeesNote() {
		return conveyorSensor.getRange() < CONVEYOR_THRESHOLD;
	}

	public BooleanSupplier conveyorSeesNoteSupplier() {
		return this::conveyorSeesNote;
	}

	public BooleanSupplier getHasInfedCache() {
		return () -> hasInfedCache;
	}

	public void setHasInfedCache(BooleanSupplier hasInfed) {
		hasInfedCache = hasInfed.getAsBoolean();
	}

	public void cacheInfeedState() {
		hasInfedCache = hasInfed();
	}
}
