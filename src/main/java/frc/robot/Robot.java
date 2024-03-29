// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private RobotContainer m_robotContainer;

    @Override
    public void robotInit() {
        DataLogManager.start();
        m_robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
        DataLogManager.stop();
    }

    @Override
    public void disabledPeriodic() {
        // var pose = m_robotContainer.getBestPose();
        // if (pose.isPresent())
        // m_robotContainer.drivetrain.seedFieldRelative(pose.get().estimatedPose.toPose2d());
    }

    @Override
    public void disabledExit() {
    }

    @Override
    public void autonomousInit() {
        DataLogManager.start();

        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }

        // m_robotContainer.zero();
    }

    @Override
    public void autonomousPeriodic() {
        m_robotContainer.logValues();
    }

    @Override
    public void autonomousExit() {
    }

    @Override
    public void teleopInit() {
        DataLogManager.start();

        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }

        // m_robotContainer.zero();
    }

    @Override
    public void teleopPeriodic() {
        m_robotContainer.logValues();
    }

    @Override
    public void teleopExit() {
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void testExit() {
    }

    @Override
    public void simulationPeriodic() {
    }
}
