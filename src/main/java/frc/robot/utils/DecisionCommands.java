// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/** Add your docs here. */
public class DecisionCommands {
    // private final CommandSwerveDrivetrain m_drivetrain;
    private final NoteSensing m_sensing;

    // public DecisionCommands(CommandSwerveDrivetrain drivetrain, NoteSensing sensing) {
    //     m_drivetrain = drivetrain;
    //     m_sensing = sensing;
    // }

    public DecisionCommands(NoteSensing m_sensing) {
        this.m_sensing = m_sensing;
    }

    public Command noteDecision(Command hasNote, Command noNote) {
        return Commands.waitUntil(m_sensing::hasInfed).withTimeout(0.5)
        .andThen(Commands.either(
            hasNote, noNote, m_sensing::hasInfed));
    }
}
