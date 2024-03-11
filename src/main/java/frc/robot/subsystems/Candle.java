// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Candle extends SubsystemBase {
    private final CANdle candle;
    private final int NUM_LEDS = 69;
    private Color color;

    enum Color {

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

    /** Creates a new light. */
    private Candle() {
        candle = new CANdle(21, "rio");
        candle.configBrightnessScalar(.5);
        candle.configLEDType(LEDStripType.GRB);
        setColor(Color.WHITE);
    }

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

    public FireAnimation burnyBurn() {
        return new FireAnimation(1, .5, NUM_LEDS, .5, .5);
    }

    public RainbowAnimation rainbow() {
        return new RainbowAnimation();
    }

    public Command runBurnyBurnCommand() {
        return runOnce(() -> candle.animate(burnyBurn()));
    }

    public Command runRainbowAnimationCommnad() {
        return runOnce(() -> candle.animate(rainbow()));
    }

    //Blink command -> Use for anything
public SequentialCommandGroup blink(Color color) {
    return new SequentialCommandGroup(
        new InstantCommand(() -> setNoColor()),
        new WaitCommand(0.02),
        new InstantCommand(() -> setColor(color)),
        new WaitCommand(0.02),
        new InstantCommand(() -> setNoColor()),
        new WaitCommand(0.02),
        new InstantCommand(() -> setColor(color)),
        new InstantCommand(() -> setNoColor()));
}





    //TODO: Add following options: Blinking, quick-flash, fade/breathe mode, more colors???


    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}