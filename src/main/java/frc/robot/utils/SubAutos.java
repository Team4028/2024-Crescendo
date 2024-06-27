// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.wpilibj2.command.Command;

/** Add your docs here. */
public class SubAutos {
    private final DecisionCommands m_decision;

    public SubAutos(NoteSensing sensing) {
        m_decision = new DecisionCommands(sensing);
    }

    public Command note5or4() {
        Command shoot = new SubAutoGenerator().addPath("Source Move 5 Shoot")
                .addNamedCommand("Magic Shoot")
                .addPath("Source Shot - 4");
        Command bail = new SubAutoGenerator().addPath("Source 5-4");

        return m_decision.noteDecision(shoot, bail);
    }

    public Command note4or3() {
        Command shoot = new SubAutoGenerator().addPath("Source 4 - Shot")
                .addNamedCommand("Magic Shoot")
                .addPath("Source Move Shot - 3");
        Command bail = new SubAutoGenerator().addPath("Source 4 - 3");

        return m_decision.noteDecision(shoot, bail);
    }
}
