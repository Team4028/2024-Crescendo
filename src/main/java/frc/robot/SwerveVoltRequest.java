package frc.robot;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

public class SwerveVoltRequest implements SwerveRequest {
    private final MotionMagicVoltage m_motionMagicCtrl = new MotionMagicVoltage(0.);
    private final VoltageOut m_voltOutCtrl = new VoltageOut(0.).withEnableFOC(true);

    private double m_targetVolt = 0.;

    @Override
    public StatusCode apply(SwerveControlRequestParameters parameters, SwerveModule... modulesToApply) {
        for (var mod : modulesToApply) {
            mod.getSteerMotor().setControl(m_motionMagicCtrl);

            mod.getDriveMotor().setControl(m_voltOutCtrl.withOutput(m_targetVolt));
        }

        return StatusCode.OK;
    }

    public SwerveVoltRequest withVoltage(double targetVoltage) {
        this.m_targetVolt = targetVoltage;
        return this;
    }
}