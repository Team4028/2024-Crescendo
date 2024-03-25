// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** Some Command helpers. */
public final class BeakCommands {
    /*
     * Repeat the specified command n number of times.
     */
    public static Command repeatCommand(Command command, int n) {
        SequentialCommandGroup cmd = new SequentialCommandGroup();

        for (int i = 0; i < n; ++i) {
            cmd.addCommands(command);
        }

        return cmd;
    }
}
