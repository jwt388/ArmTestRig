// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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

    public static final double kP = 1;

    // These are fake gains; in actuality these must be determined individually for each robot
    public static final double kSVolts = 1;
    public static final double kGVolts = 1;
    public static final double kVVoltSecondPerRad = 0.5;
    public static final double kAVoltSecondSquaredPerRad = 0.1;

    public static final double kMaxVelocityRadPerSecond = 3;
    public static final double kMaxAccelerationRadPerSecSquared = 10;

    public static final int[] kEncoderPorts = new int[] {4, 5};
    public static final int kEncoderPPR = 256;
    public static final double gearRatio = 1/100; // FIX FIX FIX
    public static final double kArmRadiansPerEncoderRotation = 2.0 * Math.PI * gearRatio;
    public static final double kRPMtoRadPerSec = kArmRadiansPerEncoderRotation / 60;

    // The offset of the arm from the horizontal in its neutral position,
    // measured from the horizontal
    public static final double kArmOffsetRads = 0.5;
    public static final double kArmHighPosition = 2.0;
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
}
