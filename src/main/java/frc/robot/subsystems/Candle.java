// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Candle extends SubsystemBase {
    private final CANdle candle;
    private final int NUM_LEDS = 68;
    private final int STRIP_LEDS = 60;
    private Color color;

    public enum Color {

        GREEN(0, 254, 0),
        PURPLE(118, 0, 254),
        ORANGE(254, 55, 0),
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

    /* Creates a new light. */
    public Candle() {
        candle = new CANdle(21, "rio");
        candle.configBrightnessScalar(.5);
        candle.configLEDType(LEDStripType.GRB);
        setColor(Color.WHITE);
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

    public FireAnimation burnyBurn() {
        return new FireAnimation(1, 0.5, NUM_LEDS, 0.5, 0.1);
    }

    public RainbowAnimation rainbow() {
        return new RainbowAnimation(100.00, 0.9, NUM_LEDS, true, 0);
    }

    public SingleFadeAnimation fade(Color color) {
        return new SingleFadeAnimation(color.r, color.g, color.b, 100, 0.8, NUM_LEDS);
    }

    public ColorFlowAnimation shootFlow(Color color) {
        return new ColorFlowAnimation(color.r, color.g, color.b, 100, 0.8, NUM_LEDS, Direction.Forward);
    }

    public ColorFlowAnimation infeedFlow(Color color) {
        return new ColorFlowAnimation(color.r, color.g, color.b, 100, 0.8, NUM_LEDS, Direction.Backward);
    }

    public StrobeAnimation strobe(Color color) {
        return new StrobeAnimation(color.r, color.g, color.b, 100, 0.8, NUM_LEDS, 0);
    }

    // Commands for complex animations //
    public Command runBurnyBurnCommand() {
        return runOnce(() -> candle.animate(burnyBurn()));
    }

    public Command runRainbowAnimationCommnad() {
        return runOnce(() -> candle.animate(rainbow()));
    }

    public Command runFade(Color color) {
        return runOnce(() -> candle.animate(fade(color)));
    }

    public Command runShootFlow(Color color) {
        return runOnce(() -> candle.animate(shootFlow(color)));
    }

    public Command runInfeedFlow(Color color) {
        return runOnce(() -> candle.animate(infeedFlow(color)));
    }

    public Command runStrobe(Color color) {
        return runOnce(() -> candle.animate(strobe(color)));
    }

    // Blink command -> Use for anything //
    public Command blink(Color color, double cycles) {
        SequentialCommandGroup commandGroup = new SequentialCommandGroup();

        for (int i = 0; i < cycles; i++) {
            commandGroup.addCommands(
                    new InstantCommand(() -> setNoColor()),
                    new WaitCommand(0.1),
                    new InstantCommand(() -> setColor(color)),
                    new WaitCommand(0.1));

        }
        return commandGroup;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

}