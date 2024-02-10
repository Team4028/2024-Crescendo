// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;
import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;



public class Light extends SubsystemBase {
  private static Light instance;
  private Color m_color;
  private CANdle candle;
  private final int NUM_LEDS = 119;
  private final int STRIP_LENGTH = 51;

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
  private Light() {
    candle = new CANdle(21, "rio");
    candle.configBrightnessScalar(1.);
    candle.configLEDType(LEDStripType.GRB);
    setColor(Color.WHITE);
  }

  public void setColor(Color color) {
    m_color = color;
  }

  public void setNoColor(Color color){
  setColor(Color.OFF); 
  }

  public static Light getInstance() {
    return instance == null ? instance = new Light() : instance;
  }
  public FireAnimation BurnyBurn() {
    return new FireAnimation(1, .5, NUM_LEDS, .5, .5);
  }
  
  public RainbowAnimation Rainbow() {
    return new RainbowAnimation();
  }

  public Command runBurnyBurnCommand() {
    return runOnce (() -> BurnyBurn());
  }

  public Command runRainbowAnimationCommnad() {
    return runOnce(() -> Rainbow());
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}