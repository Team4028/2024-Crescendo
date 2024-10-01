// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WrapperCommand;

/** Add your docs here. */
public class BeakWrappedCommand extends WrapperCommand {
	public BeakWrappedCommand(Command command) {
		super(command);
	}
}
