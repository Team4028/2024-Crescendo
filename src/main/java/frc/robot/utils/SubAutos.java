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

    private Command standardMagic(String successPath, String returnPath, String bailPath) {
        Command shoot = new SubAutoGenerator().addPath(successPath)
                .addNamedCommand("Magic Shoot")
                .addPath(returnPath);
        Command bail = new SubAutoGenerator().addPath(bailPath);

        return m_decision.noteDecision(shoot, bail);

    }

    public Command note5or4() {
        return standardMagic("Source Move 5 Shoot", "Source Shot - 4", "Source 5-4");
    }

    public Command note4or5() {
        return standardMagic("Source 4 - Shot", "Source Shot - 5", "Source 4-5");
    }

    public Command note5or3() {
        return standardMagic("Source Move 5 Shoot", "Source Move Shot - 3", "Trans 5-3");
    }

    public Command note4or3() {
        return standardMagic("Source 4 - Shot", "Source Move Shot - 3", "Source 4 - 3");
    }

    public Command note1or2() {
        return standardMagic("Amp 1 - Magic", "Amp Magic - 2", "Amp 1 - 2");
    }

    public Command note2or3() {
        return standardMagic("Amp 2 - Magic", "Amp Magic - 3", "Amp 2 - 3");
    }
}
