// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.wpilibj.DataLogManager;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.utils.DashboardStore;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.LogStore;
import frc.robot.utils.ShooterTable;
import frc.robot.utils.SignalStore;
import frc.robot.utils.ShooterTable.VisionTableEntry.CameraLerpStrat;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private RobotContainer m_robotContainer;

    private boolean hasZeroedPivot = false;

    @Override
    public void robotInit() {
        DataLogManager.start();
        m_robotContainer = new RobotContainer();
        SignalLogger.setPath("/media/sda1/ctre");

        // load static libs into memory
        ShooterTable.calcShooterTableEntryCamera(LimelightHelpers.getLatestResults("").targetingResults.pipelineID,
                CameraLerpStrat.LimelightTY);
        FollowPathCommand.warmupCommand().schedule();

        // Do this every 100 ms
        addPeriodic(() -> {
            DashboardStore.update();
            if (!RobotState.isAutonomous()) {
                m_robotContainer.updateDrivePoseMT2();
            }
        }, 0.1);
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        // m_robotContainer.updateDrivePoseMT2();
        // m_robotContainer.printDistanceValues();
        m_robotContainer.encodeLimelights().ignoringDisable(true).schedule();
        SignalStore.update();
    }

    @Override
    public void disabledInit() {
        DataLogManager.stop();
        SignalLogger.stop();
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void disabledExit() {
    }

    @Override
    public void autonomousInit() {
        hasZeroedPivot = true;
        DataLogManager.start();

        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }

        // m_robotContainer.zero();
        // m_robotContainer.configVisionFieldOrigins();
    }

    @Override
    public void autonomousPeriodic() {
        LogStore.update();
    }

    @Override
    public void autonomousExit() {
    }

    @Override
    public void teleopInit() {
        DataLogManager.start();
        SignalLogger.start();

        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }

        m_robotContainer.stopShooter();
        if (!hasZeroedPivot)
            m_robotContainer.zero();
        // m_robotContainer.configVisionFieldOrigins();
    }

    @Override
    public void teleopPeriodic() {
        LogStore.update();
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
