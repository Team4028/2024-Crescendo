// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/** Add your docs here. */
public class SubAutos {
    private final DecisionCommands decision;

    public SubAutos(NoteSensing sensing) {
        decision = new DecisionCommands(sensing);
    }

    private Command standardMagic(String successPath, String returnPath, String bailPath) {
        SubAutoGenerator shoot = new SubAutoGenerator().addPath(successPath)
                .addNamedCommand("Magic Shoot");
        if (returnPath != null)
                shoot.addPath(returnPath);
        Command bail = Commands.none();
        if (bailPath != null)
            bail = new SubAutoGenerator().addPath(bailPath);

        return decision.noteDecision(shoot, bail);

    }

    public Command note5or4() {
        return standardMagic("Source Move 5 Shoot", "Source Shot - 4", "Source 5-4");
    }

    public Command note4or5() {
        return standardMagic("Source 4 - Shot", "Source Shot - 5", "Trans 4-5");
    }

    public Command note5or3() {
        return standardMagic("Source Move 5 Shoot", "Source Move Shot - 3", "Trans 5-3");
    }

    public Command note4or3() {
        return standardMagic("Source 4 - Shot", "Source Move Shot - 3", "Source 4 - 3");
    }

    public Command note3orStop() {
        return standardMagic("Source Move 3 - Shot", "Source Move 3 - Shot", null);
    }

    public Command note1or2() {
        return standardMagic("Amp 1 - Magic", "Amp Magic - 2", "Amp 1 - 2");
    }

    public Command note2or1() {
        return standardMagic("Amp 2 - Magic", "Amp Magic - 1", "Trans 2-1");
    }

    public Command note1or3() {
        return standardMagic("Amp 1 - Magic", "Amp Magic - 3", "Trans 1-3");
    }

    public Command note2or3() {
        return standardMagic("Amp 2 - Magic", "Amp Magic - 3", "Amp 2 - 3");
    }
}
