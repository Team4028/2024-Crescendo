// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.BeakCommands;
import frc.robot.utils.Limelight;

public class Candle extends SubsystemBase {
    private final CANdle candle;
    private Color[] colors = new Color[8];
    private int[][] llViewports = new int[][] { { 0, 7 }, { 1, 6 }, { 2, 5 }, { 3, 4 } };

    public enum Color {
        GREEN(0, 254, 0),
        PURPLE(118, 0, 254),
        PINK(254, 0, 118),
        YELLOW(118, 118, 0),
        ORANGE(254, 55, 0),
        LBLUE(55, 55, 254),
        BLUE(0, 0, 254),
        WHITE(254, 254, 254),
        RED(254, 0, 0),
        OFF(0, 0, 0);

        public int r;
        public int g;
        public int b;

        Color(int r, int g, int b) {
            this.r = r;
            this.g = g;
            this.b = b;
        }
    }

    public Candle() {
        candle = new CANdle(3, "rio");
        candle.configBrightnessScalar(0.25);
        candle.configLEDType(LEDStripType.GRB);
    }

    public void setLED(int index, Color color) {
        colors[index] = (color.r + color.g + color.b == 0 ? color : colors[index]);
        candle.setLEDs(color.r, color.g, color.b, 0, index, 1);
    }

    public void setLEDs(int startIdx, int endIdx, Color color) {
        for (int i = startIdx; i < endIdx; i++)
            setLED(i, color);
    }

    public void setLEDs(int[] ledIndicies, Color color) {
        for (var i : ledIndicies)
            setLED(i, color);
    }

    public void setLEDsToColors() {
        for (int i = 0; i < colors.length; i++) {
            setLED(i, colors[i]);
        }
    }

    public Command encodeLimelights(Limelight shooterLimelight, Limelight leftLimelight, Limelight rightLimelight) {
        return runOnce(() -> {
            setLEDs(llViewports[0], shooterLimelight.getTV() ? Color.GREEN : Color.RED);

            for (int i = 0; i < 2; i++) {
                setLEDs(llViewports[i],
                        Color.values()[(i == 0 ? leftLimelight : rightLimelight).getBotposeEstimateMT2().tagCount]);
            }
        });
    }

    public Command strobeCommand(double cyclePeriod, int numCycles) {
        return BeakCommands.repeatCommand(runOnce(() -> setLEDs(0, 7, Color.OFF))
                .andThen(Commands.waitSeconds(cyclePeriod)).andThen(runOnce(this::setLEDsToColors))
                .andThen(Commands.waitSeconds(cyclePeriod)), numCycles);
    }
}