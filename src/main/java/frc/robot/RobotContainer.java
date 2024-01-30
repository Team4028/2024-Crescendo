// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final PrototypeShooter prototypeShooter = PrototypeShooter.getInstance();
  private final Feeder feeder = Feeder.getInstance();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    //vBus for trap is 0.15
    // vbus for  speaker is 0.75
    m_driverController.a().onTrue(prototypeShooter.spinMotorRightCommand(0.75))
        .onFalse(prototypeShooter.spinMotorRightCommand(0));

    // Set vBus to as needed for speed difference
    //vBus for trap is 0.25
    // vBus for speaker is 0.9
    m_driverController.a().onTrue(prototypeShooter.spinMotorLeftCommand(0.9))
        .onFalse(prototypeShooter.spinMotorLeftCommand(0));
  
        //consistent feed vbus is ???
    m_driverController.b().onTrue(feeder.runFeederMotorCommand(.5))
    .onFalse(feeder.runFeederMotorCommand(0));
    //m_driverController.x().onTrue(prototypeShooter.setAToVelCommand(600)).onTrue(prototypeShooter.setBToVelCommand(600));
    //m_driverController.y().onTrue(prototypeShooter.setAToVelCommand(0)).onTrue(prototypeShooter.setBToVelCommand(0));
      }

  public void logVals() {
    prototypeShooter.logValues();
  }
}
