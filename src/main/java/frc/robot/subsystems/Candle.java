// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.BeakCommands;
import frc.robot.utils.Limelight;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class Candle extends SubsystemBase {
    private final CANdle candle;

    private Color color;

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
        public int color;

        Color(int r, int g, int b) {
            this.r = r;
            this.g = g;
            this.b = b;
            this.color = r | g << 8 | b << 16;
        }
    }

    /* Creates a new light. */
    public Candle() {
        candle = new CANdle(3, "rio");
        candle.configBrightnessScalar(.25);
        candle.configLEDType(LEDStripType.GRB);
        setColor(Color.WHITE);
    }

    public Command encodeLimelights(Limelight ll3, Limelight... llGs) {
        return runOnce(() -> {
            int[][] pairs = new int[][]{{0, 7}, {1, 6}, {3, 4}};
            setLedsColor(pairs[0][0], 1, 0xFF << (ll3.getTV() * 8));
            setLedsColor(pairs[0][1], 1, 0xFF << (ll3.getTV() * 8));
            for (int i = 0; i < llGs.length; i++) {
                Color color = switch (llGs[i].getBotposeEstimateMT2().tagCount) {
                    case 0 -> Color.RED;
                    case 1 -> Color.YELLOW;
                    case 2 -> Color.PURPLE;
                    case 3 -> Color.LBLUE;
                    case 4 -> Color.BLUE;
                    default -> Color.PINK;
                };
                setLedsColor(pairs[i + 1][0], 1, color.color);
                setLedsColor(pairs[i + 1][1], 1, color.color);
                setLedsColor(2, 1, Color.OFF.color);
                setLedsColor(5, 1, Color.OFF.color);
            }
        });
    }

    public void setLedsColor(int ledIndex, int count, int color) {
        setLedsColor(ledIndex, count, color & 0xFF, (color >> 8) & 0xFF, (color >> 16) & 0xFF);
    }

    public void setLedsColor(int ledIndex, int count, int r, int g, int b) {
        candle.setLEDs(r, g, b, 0, ledIndex, count);
        
    }

    // Basic colors //
    public void setColor(Color color) {
        this.color = color;
        candle.animate(null);
        candle.setLEDs(this.color.r, this.color.g, this.color.b);
    }

    public void setNoColor() {
        setColor(Color.OFF);
    }

    public Command setColorGreenCommand() {
        return runOnce(() -> setColor(Color.GREEN));
    }

    public Command setColorRedCommand() {
        return runOnce(() -> setColor(Color.RED));
    }

    public Command setNoColorCommand() {
        return runOnce(() -> setColor(Color.OFF));
    }

    // Creates the complex animations //

    // public FireAnimation burnyBurn() {
    //     return new FireAnimation(1, 0.5, NUM_LEDS, 0.5, 0.1);
    // }

    // public RainbowAnimation rainbow() {
    //     return new RainbowAnimation(100.00, 0.9, NUM_LEDS, true, 0);
    // }

    // public SingleFadeAnimation fade(Color color) {
    //     return new SingleFadeAnimation(color.r, color.g, color.b, 100, 0.8, NUM_LEDS);
    // }

    // public ColorFlowAnimation shootFlow(Color color) {
    //     return new ColorFlowAnimation(color.r, color.g, color.b, 100, 0.8, NUM_LEDS, Direction.Forward);
    // }

    // public ColorFlowAnimation infeedFlow(Color color) {
    //     return new ColorFlowAnimation(color.r, color.g, color.b, 100, 0.8, NUM_LEDS, Direction.Backward);
    // }

    // public StrobeAnimation strobe(Color color) {
    //     return new StrobeAnimation(color.r, color.g, color.b, 100, 0.8, NUM_LEDS, 0);
    // }

    // // Commands for complex animations //
    // public Command runBurnyBurnCommand() {
    //     return runOnce(() -> candle.clearAnimation(NUM_LEDS)).andThen(runOnce(() -> candle.animate(burnyBurn())));
    // }

    // public Command runRainbowAnimationCommnad() {
    //     return runOnce(() -> candle.animate(rainbow()));
    // }

    // public Command runFade(Color color) {
    //     return runOnce(() -> candle.animate(fade(color)));
    // }

    // public Command runShootFlow(Color color) {
    //     return runOnce(() -> candle.animate(shootFlow(color)));
    // }

    // public Command runInfeedFlow(Color color) {
    //     return runOnce(() -> candle.animate(infeedFlow(color)));
    // }

    // public Command runStrobe(Color color) {
    //     return runOnce(() -> candle.animate(strobe(color)));
    // }

    // Blink command -> Use for anything //
    public Command blink(Color color, int cycles) {
        return BeakCommands.repeatCommand(
                setNoColorCommand().andThen(
                        Commands.waitSeconds(0.25),
                        runOnce(() -> setColor(color)),
                        Commands.waitSeconds(0.25)),
                cycles);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

}