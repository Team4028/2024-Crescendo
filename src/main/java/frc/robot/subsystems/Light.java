// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Light extends SubsystemBase {
  private static Light instance;
  private CANdle m_candle;

  enum Color {

    BLUE(0,0,245),
    GREEN(0,245,0),
    RED(245,0,0),
    WALTER_WHITE(254, 254, 254),
    HOT_PINK(254, 0, 170);

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
    m_candle = new CANdle(21, "rio");
    m_candle.configBrightnessScalar(1.);
    m_candle.configLEDType(LEDStripType.GRB);
    setLEDsToColor(Color.WALTER_WHITE);
  }

  public void setLEDsToColor(Color color) {
    m_candle.setLEDs(color.r, color.g, color.b);
  }

  public static Light getInstance() {
    return instance == null ? instance = new Light() : instance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}