// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.util.HashMap;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;

/** Add your docs here. */
public final class LogStore {
	private static final DataLog log = DataLogManager.getLog();

	private static HashMap<DoubleLogEntry, DoubleSupplier> doubleEntries = new HashMap<>();
	private static HashMap<StringLogEntry, Supplier<String>> stringEntries = new HashMap<>();
	private static HashMap<BooleanLogEntry, BooleanSupplier> boolEntries = new HashMap<>();

	public static void add(String name, DoubleSupplier value) {
		doubleEntries.put(new DoubleLogEntry(log, name), value);
	}

	public static void add(String name, Supplier<String> value) {
		stringEntries.put(new StringLogEntry(log, name), value);
	}

	public static void add(String name, BooleanSupplier value) {
		boolEntries.put(new BooleanLogEntry(log, name), value);
	}

	public static void update() {
		doubleEntries.forEach((entry, value) -> {
			entry.append(value.getAsDouble());
		});

		stringEntries.forEach((entry, value) -> {
			entry.append(value.get());
		});

		boolEntries.forEach((entry, value) -> {
			entry.append(value.getAsBoolean());
		});
	}
}
