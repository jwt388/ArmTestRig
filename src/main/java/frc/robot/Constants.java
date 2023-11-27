// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  
  public static final class ArmConstants {
    public static final int kMotorPort = 4;

    public static final double kP = 3; // was 5

    // These are fake gains; in actuality these must be determined individually for each robot
    public static final double kSVolts = 0.5;
    public static final double kGVolts = 1.25;
    public static final double kVVoltSecondPerRad = 0.8;
    public static final double kAVoltSecondSquaredPerRad = 0.05;

    public static final double kMaxVelocityRadPerSecond = Units.degreesToRadians(90);
    public static final double kMaxAccelerationRadPerSecSquared = Units.degreesToRadians(360);

    public static final int[] kEncoderPorts = new int[] {4, 5};
    public static final int kEncoderPPR = 256;
    public static final double gearRatio = 1/100;
    public static final double kArmRadiansPerEncoderRotation = 2.0 * Math.PI * gearRatio;
    public static final double kRPMtoRadPerSec = kArmRadiansPerEncoderRotation / 60;

    // Arm positions.  Horizontal = 0 radians
    public static final double kMinAngleRads = Units.degreesToRadians(-45);
    public static final double kMaxAngleRads = Units.degreesToRadians(120);
    public static final double kArmOffsetRads = kMaxAngleRads;
    public static final double kArmLowPositionRad = Units.degreesToRadians(-30);
    public static final double kArmHighPositionRad = Units.degreesToRadians(45);
    public static final double kPosIncrement = Units.degreesToRadians(2);
  }

  public static final class AutoConstants {
    public static final double kAutoTimeoutSeconds = 12;
    public static final double kAutoShootTimeSeconds = 7;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
  }

  // RevRobotics Blinkin
  public static final int BLIKIN_SPARK_PORT = 0;
  public static final double BLINKIN_RED = 0.61;
  public static final double BLINKIN_DARK_GREEN = 0.75;

  // ----------------------------------------------------------------------
  // Constants for simulation from ArmSimulation example
  public static final int kEncoderAChannel = 0; 
  public static final int kEncoderBChannel = 1; 
  public static final int kMotorPort = 0;

  // distance per pulse = (angle per revolution) / (pulses per revolution)
  //  = (2 * PI rads) / (4096 pulses)
  public static final double kMotorEncoderDistPerPulse = 2.0 * Math.PI / 4096;

  public static final double kArmReduction = 200;
  public static final double kArmMass = 8.0; // Kilograms
  public static final double kArmLengthInches = 30;
  public static final double kArmExtendedLengthInches = 45;
  public static final double kArmLength = Units.inchesToMeters(kArmLengthInches);
  public static final double kArmExtendedLength = Units.inchesToMeters(kArmExtendedLengthInches);
  public static final double kStartAngleRads = ArmConstants.kMinAngleRads;
  public static final double kArmEncoderDistPerPulse = kMotorEncoderDistPerPulse / kArmReduction;


  // ----------------------------------------------------------------------
  // Constants for debugging
public static final boolean allowReenable = false;

}
