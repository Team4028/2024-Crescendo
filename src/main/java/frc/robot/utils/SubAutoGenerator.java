// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class SubAutoGenerator extends SequentialCommandGroup {
	/** Creates a new SubAutoGenerator. */
	public SubAutoGenerator() {
	}

	public SubAutoGenerator addPath(String pathName) {
		return addCommand(AutoBuilder.followPath(PathPlannerPath.fromPathFile(pathName)));
	}

	public SubAutoGenerator addCommand(Command command) {
		addCommands(command);
		return this;
	}

	public SubAutoGenerator addNamedCommand(String name) {
		return addCommand(NamedCommands.getCommand(name));
	}
}
