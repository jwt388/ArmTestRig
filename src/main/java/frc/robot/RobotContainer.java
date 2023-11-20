// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final ArmSubsystem m_robotArm = new ArmSubsystem();

  // The driver's controller
  CommandXboxController m_driverController =
      new CommandXboxController(OIConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    
    SmartDashboard.putData(m_robotArm);

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * JoystickButton}.
   */
  private void configureButtonBindings() {
    // Move the arm to high position when the 'B' button is pressed.
    m_driverController
        .b()
        .onTrue(
            Commands.runOnce(
                () -> {
                  m_robotArm.setGoal(Constants.ArmConstants.kArmHighPosition);
                  m_robotArm.enable();
                },
                m_robotArm));

    // Move the arm to neutral (low) position when the 'A' button is pressed.
    m_driverController
        .a()
        .onTrue(
            Commands.runOnce(
                () -> {
                  m_robotArm.setGoal(Constants.ArmConstants.kArmOffsetRads);
                  m_robotArm.enable();
                },
                m_robotArm));

    m_driverController
          .povDown()
          .onTrue(
              Commands.runOnce(
                () -> {
                  m_robotArm.setGoal(0);
                },
                m_robotArm));

    // Reset the encoders to zero when the 'X' button is pressed. 
    //   Should only be used when are is in neutral position.
    m_driverController.x().onTrue(Commands.runOnce(m_robotArm::resetPosition));

    // Disable the arm controller when Y is pressed.
    m_driverController.y().onTrue(Commands.runOnce(m_robotArm::disable));

  }

  /**
   * Disables all ProfiledPIDSubsystem and PIDSubsystem instances. This should be called on robot
   * disable to prevent integral windup.
   */
  public void disablePIDSubsystems() {
    m_robotArm.disable();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return Commands.none();
  }
}
