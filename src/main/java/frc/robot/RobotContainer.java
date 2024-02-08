// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
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
    private final Shooter shooter = new Shooter();
    private final Feeder feeder = Feeder.getInstance();

    // Replace with CommandPS4Controller or CommandJoystick if needed
    private final CommandXboxController driverController = new CommandXboxController(
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
        // vBus for trap is 0.15
        // vbus for speaker is 0.75
        // driverController.a().onTrue(shooter.setRightToVelCommand(7000))
        // .onFalse(shooter.setRightToVelCommand(0));

        // Set vBus to as needed for speed difference
        // vBus for trap is 0.25
        // vBus for speaker is 0.9
        // driverController.a().onTrue(shooter.setLeftToVelCommand(7000))
        // .onFalse(shooter.setLeftToVelCommand(0));

        // consistent feed vbus is ???
        driverController.b().onTrue(feeder.runFeederMotorCommand(.5))
                .onFalse(feeder.runFeederMotorCommand(0));

        driverController.a().onTrue(shooter.runVelocityCommand());

        driverController.x().and(driverController.pov(0)).whileTrue(shooter.runDynamuc(Direction.kForward));
        driverController.x().and(driverController.pov(180)).whileTrue(shooter.runDynamuc(Direction.kReverse));

        driverController.y().and(driverController.pov(0)).whileTrue(shooter.runQuasi(Direction.kForward));
        driverController.y().and(driverController.pov(180)).whileTrue(shooter.runQuasi(Direction.kReverse));

    }

    public void logVals() {
        // shooter.logValues();
    }
}
