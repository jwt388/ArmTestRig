// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import org.ejml.simple.SimpleMatrix;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
// added for simulation
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax; 
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;


/** A robot arm subsystem that moves with a motion profile. */
public class ArmSubsystem extends ProfiledPIDSubsystem {
  private final CANSparkMax m_motor =
  new CANSparkMax(ArmConstants.kMotorPort, MotorType.kBrushless);
  private final RelativeEncoder m_encoder = m_motor.getEncoder();
  
  private final PWMSparkMax m_motor2 = new PWMSparkMax(Constants.kMotorPort); // for simulation

  private final Encoder m_encoder2 =
      new Encoder(Constants.kEncoderAChannel, Constants.kEncoderBChannel); // for simulation

  private final ArmFeedforward m_feedforward =
      new ArmFeedforward(
          ArmConstants.kSVolts, ArmConstants.kGVolts,
          ArmConstants.kVVoltSecondPerRad, ArmConstants.kAVoltSecondSquaredPerRad);

  //private final Spark blinkinSpark = new Spark(Constants.BLIKIN_SPARK_PORT);
  //private double blinkinVoltage = Constants.BLINKIN_DARK_GREEN;
  
  private double m_voltageCommand = 0;
  private double m_goalposition;

  // The arm gearbox represents a gearbox containing two Vex 775pro motors.
  private final DCMotor m_armGearbox = DCMotor.getVex775Pro(2);

  // Simulation classes help us simulate what's going on, including gravity.
  // This arm sim represents an arm that can travel from -75 degrees (rotated down front)
  // to 255 degrees (rotated down in the back).
  private final SingleJointedArmSim m_armSim =
      new SingleJointedArmSim(
          m_armGearbox,
          Constants.kArmReduction,
          SingleJointedArmSim.estimateMOI(Constants.kArmLength, Constants.kArmMass),
          Constants.kArmLength,
          ArmConstants.kMinAngleRads,
          ArmConstants.kMaxAngleRads,
          true,
          VecBuilder.fill(Constants.kArmEncoderDistPerPulse) // Add noise with a std-dev of 1 tick
          );

  private Matrix<N2,N1> startState = new Matrix<>(new SimpleMatrix(2, 1));
  private final EncoderSim m_encoderSim = new EncoderSim(m_encoder2);

  // Create a Mechanism2d display of an Arm with a fixed ArmTower and moving Arm.
  private final Mechanism2d m_mech2d = new Mechanism2d(90, 60);
  private final MechanismRoot2d m_armPivot = m_mech2d.getRoot("ArmPivot", 40, 30);
  private final MechanismLigament2d m_armTower =
      m_armPivot.append(new MechanismLigament2d("ArmTower", 30, -90));
  private final MechanismLigament2d m_arm =
      m_armPivot.append(
          new MechanismLigament2d(
              "Arm",
              Constants.kArmLengthInches,
              Units.radiansToDegrees(m_armSim.getAngleRads()),
              6,
              new Color8Bit(Color.kYellow)));

  /** Create a new ArmSubsystem. */
  public ArmSubsystem() {
    super(new ProfiledPIDController(
      ArmConstants.kP,
      0,
      0,
      new TrapezoidProfile.Constraints(
          ArmConstants.kMaxVelocityRadPerSecond,
          ArmConstants.kMaxAccelerationRadPerSecSquared)),
      0);
        
    m_encoder.setPositionConversionFactor(ArmConstants.kArmRadiansPerEncoderRotation);
    m_encoder.setVelocityConversionFactor(ArmConstants.kRPMtoRadPerSec);
    m_encoder2.setDistancePerPulse(Constants.kArmEncoderDistPerPulse);

    resetPosition();

    m_motor.setIdleMode(IdleMode.kBrake);
    m_motor.setVoltage(0.0);

    m_motor2.setVoltage(0.0); // for simulation

    //blinkinSpark.set(blinkinVoltage);

    // Assume the arm is starting in the back rest position
    setGoal(ArmConstants.kArmOffsetRads);

    if (RobotBase.isSimulation()) {
      simulationInit();
    }

    setupShuffleboard();

    // Put Mechanism 2d to SmartDashboard
    SmartDashboard.putData("Arm Sim", m_mech2d);
    m_armTower.setColor(new Color8Bit(Color.kBlue));

  }

  @Override
  public void periodic() {
    if (m_enabled) {
      useOutput(m_controller.calculate(getMeasurement()), m_controller.getSetpoint());
    }
    updateShuffleboard();

  }

  public void simulationInit() {
    startState.set(0, 0, ArmConstants.kArmOffsetRads);
    startState.set(1, 0, 0);
    m_armSim.setState(startState);
    DataLogManager.log("Arm Initialized to : " + Units.radiansToDegrees(ArmConstants.kArmOffsetRads) + " Deg");
  }

  /** Update the simulation model. */
  public void simulationPeriodic() {
    // In this method, we update our simulation of what our arm is doing
    // First, we set our "inputs" (voltages)
    m_armSim.setInput(m_motor2.get() * RobotController.getBatteryVoltage());

    // Next, we update it. The standard loop time is 20ms.
    m_armSim.update(0.020);

    // Finally, we set our simulated encoder's readings and simulated battery voltage
    m_encoderSim.setDistance((m_armSim.getAngleRads() - ArmConstants.kArmOffsetRads)*Constants.kArmReduction);
    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(m_armSim.getCurrentDrawAmps()));

    // Update the Mechanism Arm angle based on the simulated arm angle
    m_arm.setAngle(Units.radiansToDegrees(m_armSim.getAngleRads()));

  }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    double feedforward = 0;
    if (m_enabled) {
      // Calculate the feedforward from the sepoint
      feedforward = m_feedforward.calculate(setpoint.position, setpoint.velocity);
      // Add the feedforward to the PID output to get the motor output
      m_voltageCommand = output + feedforward;
    }
    else {
      m_voltageCommand = 0;
    } 
    m_motor.setVoltage(m_voltageCommand);
    m_motor2.setVoltage(m_voltageCommand); // for simulation

    SmartDashboard.putNumber("feedforward", feedforward);
    SmartDashboard.putNumber("output", output);
    SmartDashboard.putNumber("SetPt Pos", Units.radiansToDegrees(setpoint.position));
    SmartDashboard.putNumber("SetPt Vel", Units.radiansToDegrees(setpoint.velocity));

  }

  @Override
  // Arm position for PID measurement
  public double getMeasurement() {
    if (RobotBase.isReal()) {
      return m_encoder.getPosition() + ArmConstants.kArmOffsetRads; // Add ofset for starting zero point
    }
    else {
      return m_encoder2.getDistance()/Constants.kArmReduction + ArmConstants.kArmOffsetRads; // Add ofset for starting zero point
    }

  }

  // Motor speed (Rad/sec)
  public double getVelocity() {
    if (RobotBase.isReal()) {
      return m_encoder.getVelocity();
    }
    else {
      return m_encoder2.getRate();
    }

  }

  // Reset the encoders to zero. Should only be used when arm is in neutral position.
  public void resetPosition() {
    // Arm position for PID measurement
     if (RobotBase.isReal()) {
      m_encoder.setPosition(0) ;
    }
    else {
      m_encoder2.reset();
    }
  }

   // Calculate increased  goal limited to allowed range
   public double increasedGoal() {
    double newGoal = m_controller.getGoal().position + Constants.ArmConstants.kPosIncrement;
    return MathUtil.clamp(newGoal, Constants.ArmConstants.kMinAngleRads, Constants.ArmConstants.kMaxAngleRads);
  }

  // Calculate decreased  goal limited to allowed range
  public double decreasedGoal() {
    double newGoal =  m_controller.getGoal().position - Constants.ArmConstants.kPosIncrement;
    return MathUtil.clamp(newGoal, Constants.ArmConstants.kMinAngleRads, Constants.ArmConstants.kMaxAngleRads);

  } 
  @Override
  /** Enables the PID control. Resets the controller. */
  public void enable() {

    // Don't enable if already enabled since this may cause control transients
    if (!m_enabled | Constants.allowReenable) {
      m_enabled = true;
      m_controller.reset(getMeasurement());
      DataLogManager.log("Arm Enabled");
    }
    else {
      DataLogManager.log("Arm Already Enabled");
    }
  }

  @Override
  /** Disables the PID control. Sets output to zero. */
  public void disable() {

    // Set goal to current position to minimize movement on re-enable and reset output
    m_enabled = false;
    setGoal(getMeasurement()); 
    useOutput(0, new State());
    DataLogManager.log("Arm Disabled");
    ;
  }

  /** Sets the goal state for the subsystem. Goal velocity assumed to be zero. */
  @Override
    public void setGoal(double goal) {
    setGoal(new TrapezoidProfile.State(goal, 0));
    m_goalposition = goal;

  }

  /** Extend the arm - only affects simulation display for now - no affect on MOI*/
  public void extendArm() {
    m_arm.setLength(Constants.kArmExtendedLengthInches);
  }

  /** Retract the arm - only affects simulation display for now - no affect on MOI*/
  public void retractArm() {
    m_arm.setLength(Constants.kArmLengthInches);
  }

  private void setupShuffleboard() {

    SmartDashboard.putData(m_controller);

  }

  public void updateShuffleboard() {

    SmartDashboard.putBoolean("Arm Enabled", m_enabled);
    SmartDashboard.putNumber("Arm Goal", Units.radiansToDegrees(m_goalposition));
    SmartDashboard.putNumber("Measured Angle", Units.radiansToDegrees(getMeasurement()));
    SmartDashboard.putNumber("Arm Velocity", Units.radiansToDegrees(getVelocity()));
    SmartDashboard.putNumber("Voltage", m_voltageCommand); 
    SmartDashboard.putNumber("Battery Voltage",RobotController.getBatteryVoltage());

    if (RobotBase.isReal()) {
      SmartDashboard.putNumber("Current", m_motor.getOutputCurrent()); 
    }
    else {
      SmartDashboard.putNumber("Mechanical Angle", Units.radiansToDegrees(m_armSim.getAngleRads()));
      SmartDashboard.putNumber("Sim Current", m_armSim.getCurrentDrawAmps());
    }

  }

  // @Override
  // public void close() {
  //   m_motor.close();
  //   m_encoder.close();
  //   m_mech2d.close();
  //   m_armPivot.close();
  //   m_controller.close();
  //   m_arm.close();
  // }
}
