// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;

/** A robot arm subsystem that moves with a motion profile. */
public class ArmSubsystem extends ProfiledPIDSubsystem {
  private final CANSparkMax m_motor =
  new CANSparkMax(ArmConstants.kMotorPort, MotorType.kBrushless);
  private final RelativeEncoder m_encoder = m_motor.getEncoder();
  
/*   private final PWMSparkMax m_motor = new PWMSparkMax(ArmConstants.kMotorPort);
  private final Encoder m_encoder =
      new Encoder(ArmConstants.kEncoderPorts[0], ArmConstants.kEncoderPorts[1]); */
  private final ArmFeedforward m_feedforward =
      new ArmFeedforward(
          ArmConstants.kSVolts, ArmConstants.kGVolts,
          ArmConstants.kVVoltSecondPerRad, ArmConstants.kAVoltSecondSquaredPerRad);
  private final Spark blinkinSpark = new Spark(Constants.BLIKIN_SPARK_PORT);
  private double blinkinVoltage = Constants.BLINKIN_DARK_GREEN;

  /** Create a new ArmSubsystem. */
  public ArmSubsystem() {
    super(
        new ProfiledPIDController(
            ArmConstants.kP,
            0,
            0,
            new TrapezoidProfile.Constraints(
                ArmConstants.kMaxVelocityRadPerSecond,
                ArmConstants.kMaxAccelerationRadPerSecSquared)),
        0);

    m_encoder.setPositionConversionFactor(ArmConstants.kArmRadiansPerEncoderRotation);
    m_encoder.setVelocityConversionFactor(ArmConstants.kRPMtoRadPerSec);
    resetPosition();

    m_motor.setIdleMode(IdleMode.kBrake);
    m_motor.setVoltage(0.0);

    blinkinSpark.set(blinkinVoltage);

    // Start arm at rest in neutral position
    setGoal(ArmConstants.kArmOffsetRads);
  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("Arm Position", getMeasurement());
    SmartDashboard.putNumber("Arm Velocity", getVelocity());
    
  }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    // Calculate the feedforward from the sepoint
    double feedforward = m_feedforward.calculate(setpoint.position, setpoint.velocity);
    // Add the feedforward to the PID output to get the motor output
    m_motor.setVoltage(output + feedforward);
  }

  @Override
  // Arm position for PID measurement
  public double getMeasurement() {
    return m_encoder.getPosition() + ArmConstants.kArmOffsetRads;

  }

  // Motor speed (Rad/sec)
  public double getVelocity() {
    return m_encoder.getVelocity();

  }

  // Reset the encoders to zero. Should only be used when arm is in neutral position.
  public void resetPosition() {
    // Arm position for PID measurement
     m_encoder.setPosition(0) ;
  
    }
}
