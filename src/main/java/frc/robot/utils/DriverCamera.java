// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.cscore.HttpCamera.HttpCameraKind;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class DriverCamera {
    private final HttpCamera m_shooterCamera, m_infeedCamera;
    private final VideoSink m_dashboardCamera;

    private static final String SHOOTER_CAMERA_URL = "http://limelight-shooter.local:5800";
    private static final String INFEED_CAMERA_URL = "http://limelight.local:5800";

    /** Creates a new DriverCamera. */
    public DriverCamera() {
        m_shooterCamera = new HttpCamera("Shooter Camera", SHOOTER_CAMERA_URL, HttpCameraKind.kMJPGStreamer);
        m_infeedCamera = new HttpCamera("Infeed Camera", INFEED_CAMERA_URL, HttpCameraKind.kMJPGStreamer);

        m_dashboardCamera = CameraServer.addSwitchedCamera("Dashboard Camera");
    }

    public void setShooterCamera() {
        m_dashboardCamera.setSource(m_shooterCamera);
    }

    public Command setShooterCameraCommand() {
        return Commands.runOnce(this::setShooterCamera);
    }

    public void setInfeedCamera() {
        m_dashboardCamera.setSource(m_infeedCamera);
    }

    public Command setInfeedCameraCommand() {
        return Commands.runOnce(this::setInfeedCamera);
    }
}
