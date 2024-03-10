// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.util.HashMap;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class DashboardStore {
    private static Map<String, BooleanSupplier> boolMap = new HashMap<String, BooleanSupplier>();
    private static Map<String, DoubleSupplier> doubleMap = new HashMap<String, DoubleSupplier>();
    private static Map<String, IntSupplier> intMap = new HashMap<String, IntSupplier>();
    private static Map<String, Supplier<String>> stringMap = new HashMap<String, Supplier<String>>();

    public static void add(String key, BooleanSupplier value) {
        boolMap.put(key, value);
    }

    public static void add(String key, DoubleSupplier value) {
        doubleMap.put(key, value);
    }
    
    public static void add(String key, IntSupplier value) {
        intMap.put(key, value);
    }
    
    public static void add(String key, Supplier<String> value) {
        stringMap.put(key, value);
    }

    public static void update() {
        boolMap.forEach((key, value) -> {
            SmartDashboard.putBoolean(key, value.getAsBoolean());
        });

        doubleMap.forEach((key, value) -> {
            SmartDashboard.putNumber(key, value.getAsDouble());
        });

        intMap.forEach((key, value) -> {
            SmartDashboard.putNumber(key, value.getAsInt());
        });

        stringMap.forEach((key, value) -> {
            SmartDashboard.putString(key, value.get());
        });
    }
    
}
