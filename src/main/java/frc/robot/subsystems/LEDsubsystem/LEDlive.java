// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.LEDsubsystem;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;

public class LEDlive extends SubsystemBase {
  // Subsystems
  private Drive drive;

  public int ticks = 0;
  private final AddressableLED m_led;
  private final AddressableLEDBuffer m_ledBuffer;

  // Create an LED pattern that will display a rainbow across
  // all hues at maximum saturation and half brightness
  private final LEDPattern m_rainbow = LEDPattern.rainbow(255, 128);

  // Our LED strip has a density of 120 LEDs per meter
  private static final Distance kLedSpacing = Meters.of(1 / 120.0);

  // Create a new pattern that scrolls the rainbow pattern across the LED strip, moving at a speed
  // of 1 meter per second.
  private final LEDPattern m_scrollingRainbow =
      m_rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(1), kLedSpacing);

  // Basic Colors
  private final LEDPattern m_red = LEDPattern.solid(Color.kRed);
  private final LEDPattern m_yellow = LEDPattern.solid(Color.kYellow);
  private final LEDPattern m_green = LEDPattern.solid(Color.kGreen);
  private final LEDPattern m_blue = LEDPattern.solid(Color.kBlue);
  private final LEDPattern m_purple = LEDPattern.solid(Color.kPurple);
  private final LEDPattern m_orange = LEDPattern.solid(Color.kOrange);
  private final LEDPattern m_black = LEDPattern.solid(Color.kBlack);
  private final LEDPattern m_white = LEDPattern.solid(Color.kWhite);

  public void main(LEDPattern[] args) {
    List<LEDPattern> basicColors = new ArrayList<>();

    basicColors.add(m_red);
    basicColors.add(m_orange);
    basicColors.add(m_yellow);
    basicColors.add(m_green);
    basicColors.add(m_blue);
    basicColors.add(m_purple);
    basicColors.add(m_black);
    basicColors.add(m_white);
  }

  // Staggered colors
  private final LEDPattern staggerBasic(Color inputColor) {
    LEDPattern finalColor = LEDPattern.solid(Color.kCrimson); // Error color: Crimson
    // This should alternate between the input color and black color every 10% of the total LED
    // length
    // If crimson is seen, most likely LEDPattern.steps is not returning a proper value somehow
    finalColor =
        LEDPattern.steps(
            Map.of(
                0,
                inputColor,
                0.1,
                Color.kBlack,
                0.2,
                inputColor,
                0.3,
                Color.kBlack,
                0.4,
                inputColor,
                0.5,
                Color.kBlack,
                0.6,
                inputColor,
                0.7,
                Color.kBlack,
                0.8,
                inputColor,
                0.9,
                Color.kBlack));

    // System.out.println(finalColor);
    return finalColor;
  }

  private final LEDPattern m_redSplit =
      LEDPattern.steps(Map.of(0, Color.kRed, 0.5, Color.kAntiqueWhite));

  private LEDPattern blink(LEDPattern inputColor, Integer interval, Integer ticks) {
    LEDPattern finalColor = LEDPattern.solid(Color.kCrimson); // Error color: Crimson
    // If crimson is seen, something is probably wrong with the conditional below
    if ((ticks % interval) < interval / 2) {
      finalColor = LEDPattern.kOff;
    } else {
      finalColor = inputColor;
    }

    // System.out.println(finalColor);
    return finalColor;
  }

  /** Called once at the beginning of the robot program. */
  public LEDlive(Drive drive) {
    // PWM port 9
    // Must be a PWM header, not MXP or DIO
    m_led = new AddressableLED(9);

    // Reuse buffer
    // Default to a length of 60, start empty output
    // Length is expensive to set, so only set it once, then just update data
    m_ledBuffer = new AddressableLEDBuffer(60);
    m_led.setLength(m_ledBuffer.getLength());

    // Set the data
    m_led.setData(m_ledBuffer);
    m_led.start();

    this.drive = drive;
  }

  @Override
  public void periodic() {
    ticks += 1;
    ticks %= 50000; // will limit ticks to 50000 and then reset it to 0
    // System.out.println(ticks);

    // Value for testing conditions, set to 0 to ignore
    int testVal = 0;

    // Update the buffer
    LEDPattern xColor = m_white;
    if (testVal > 0) {
      if (testVal == 1) {
        xColor = blink(staggerBasic(Color.kRed), 80, ticks);
      }
      if (testVal == 2) {
        xColor = staggerBasic(Color.kBlue);
      }
      if (testVal == 3) {
        xColor = blink(m_green, 80, ticks);
      }
      if (testVal == 4) {
        xColor = m_scrollingRainbow;
      }
      if (testVal == 5) {
        xColor = LEDPattern.kOff;
      }
    } else {

      // Apply pattern based on current drive style (true = absolute)
      if (drive.getDriveStyle() == true) {
        xColor = m_green;
      } else {
        xColor = staggerBasic(Color.kGreen);
      }
    }

    // System.out.println(xColor); // Use this print function if LEDs are not currently testable
    xColor.applyTo(m_ledBuffer);

    // Set the LEDs
    m_led.setData(m_ledBuffer);
  }
}

/// Error codes (non-exhaustive)
/// Crimson - Colors are not returning, LEDs unusable
///   + Blink, Stagger functions are somehow broken
/// Solid White - No conditionals are being met (default color)
/// ...
