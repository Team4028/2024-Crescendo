// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TestSubsystem<T> extends SubsystemBase {
    enum ControllerType {
        SPARK_MAX,
        SPARK_FLEX,
        TALONFX,
        TALONSRX
    };

// ============================================================================
// ============================ ONLY EDIT THESE VALUES ========================
// ============================================================================
    private final ControllerType MOTOR_1_TYPE = ControllerType.SPARK_FLEX;
    private final ControllerType MOTOR_2_TYPE = ControllerType.SPARK_FLEX;

    private final int MOTOR_1_ID = 0;
    private final int MOTOR_2_ID = 1;

    private final double MOTOR_1_VBUS = 0.2;
    private final double MOTOR_2_VBUS = 0.2;
// ============================================================================
// ============================================================================
// ============================================================================

    private CANSparkFlex flex1;
    private CANSparkFlex flex2;
    private CANSparkMax max1;
    private CANSparkMax max2;
    private TalonFX talonfx1;
    private TalonFX talonfx2;
    private TalonSRX talonsrx1;
    private TalonSRX talonsrx2;

    /** Creates a new TestSubsystem. */
    public TestSubsystem() {
        switch (MOTOR_1_TYPE) {
            case SPARK_FLEX:
                flex1 = new CANSparkFlex(MOTOR_1_ID, MotorType.kBrushless);
                break;
            case SPARK_MAX:
                max1 = new CANSparkMax(MOTOR_1_ID, MotorType.kBrushless);
                break;
            case TALONFX:
                talonfx1 = new TalonFX(MOTOR_1_ID);
                break;
            case TALONSRX:
                talonsrx1 = new TalonSRX(MOTOR_1_ID);
                break;
        }
        switch (MOTOR_2_TYPE) {
            case SPARK_FLEX:
                flex2 = new CANSparkFlex(MOTOR_2_ID, MotorType.kBrushless);
                break;
            case SPARK_MAX:
                max2 = new CANSparkMax(MOTOR_2_ID, MotorType.kBrushless);
                break;
            case TALONFX:
                talonfx2 = new TalonFX(MOTOR_2_ID);
                break;
            case TALONSRX:
                talonsrx2 = new TalonSRX(MOTOR_2_ID);
                break;
        }
    }

    /**
     * Example command factory method.
     *
     * @return a command
     */
    public Command runMotor1Command() {
        // Inline construction of command goes here.
        // Subsystem::Run implicitly requires `this` subsystem.
        return run(
                () -> {
                    switch (MOTOR_1_TYPE) {
                        case SPARK_FLEX:
                            flex1.set(MOTOR_1_VBUS);
                            break;
                        case SPARK_MAX:
                            max1.set(MOTOR_1_VBUS);
                            break;
                        case TALONFX:
                            talonfx1.set(MOTOR_1_VBUS);
                            break;
                        case TALONSRX:
                            talonsrx1.set(ControlMode.PercentOutput, MOTOR_1_VBUS);
                            break;
                    }
                });
    }

    public Command reverseMotor1Command() {
        // Inline construction of command goes here.
        // Subsystem::Run implicitly requires `this` subsystem.
        return run(
                () -> {
                    switch (MOTOR_1_TYPE) {
                        case SPARK_FLEX:
                            flex1.set(-MOTOR_1_VBUS);
                            break;
                        case SPARK_MAX:
                            max1.set(-MOTOR_1_VBUS);
                            break;
                        case TALONFX:
                            talonfx1.set(-MOTOR_1_VBUS);
                            break;
                        case TALONSRX:
                            talonsrx1.set(ControlMode.PercentOutput, -MOTOR_1_VBUS);
                            break;
                    }
                });
    }

    public Command stopMotor1Command() {
        // Inline construction of command goes here.
        // Subsystem::Run implicitly requires `this` subsystem.
        return run(
                () -> {
                    switch (MOTOR_1_TYPE) {
                        case SPARK_FLEX:
                            flex1.set(0);
                            break;
                        case SPARK_MAX:
                            max1.set(0);
                            break;
                        case TALONFX:
                            talonfx1.set(0);
                            break;
                        case TALONSRX:
                            talonsrx1.set(ControlMode.PercentOutput, 0);
                            break;
                    }
                });
    }

    public Command runMotor2Command() {
        // Inline construction of command goes here.
        // Subsystem::Run implicitly requires `this` subsystem.
        return run(
                () -> {
                    switch (MOTOR_2_TYPE) {
                        case SPARK_FLEX:
                            flex2.set(MOTOR_2_VBUS);
                            break;
                        case SPARK_MAX:
                            max2.set(MOTOR_2_VBUS);
                            break;
                        case TALONFX:
                            talonfx2.set(MOTOR_2_VBUS);
                            break;
                        case TALONSRX:
                            talonsrx2.set(ControlMode.PercentOutput, MOTOR_2_VBUS);
                            break;
                    }
                });
    }

    public Command reverseMotor2Command() {
        // Inline construction of command goes here.
        // Subsystem::Run implicitly requires `this` subsystem.
        return run(
                () -> {
                    switch (MOTOR_2_TYPE) {
                        case SPARK_FLEX:
                            flex2.set(-MOTOR_2_VBUS);
                            break;
                        case SPARK_MAX:
                            max2.set(-MOTOR_2_VBUS);
                            break;
                        case TALONFX:
                            talonfx2.set(-MOTOR_2_VBUS);
                            break;
                        case TALONSRX:
                            talonsrx2.set(ControlMode.PercentOutput, -MOTOR_2_VBUS);
                            break;
                    }
                });
    }

    public Command stopMotor2Command() {
        // Inline construction of command goes here.
        // Subsystem::Run implicitly requires `this` subsystem.
        return run(
                () -> {
                    switch (MOTOR_2_TYPE) {
                        case SPARK_FLEX:
                            flex2.set(0);
                            break;
                        case SPARK_MAX:
                            max2.set(0);
                            break;
                        case TALONFX:
                            talonfx2.set(0);
                            break;
                        case TALONSRX:
                            talonsrx2.set(ControlMode.PercentOutput, 0);
                            break;
                    }
                });
    }

    /**
     * An example method querying a boolean state of the subsystem (for example, a
     * digital sensor).
     *
     * @return value of some boolean subsystem state, such as a digital sensor.
     */
    public boolean exampleCondition() {
        // Query some boolean state, such as a digital sensor.
        return false;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
