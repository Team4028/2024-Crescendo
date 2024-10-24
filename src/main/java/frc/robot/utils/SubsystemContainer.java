package frc.robot.utils;

import edu.wpi.first.math.filter.SlewRateLimiter;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Candle;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Fan;
import frc.robot.subsystems.FanPivot;
import frc.robot.subsystems.Infeed;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Whippy;

/** POJO for Subsystems */
public class SubsystemContainer {
    public final CommandSwerveDrivetrain drivetrain;
    public final Infeed infeed;
    public final Shooter shooter;
    public final Conveyor conveyor;
    public final Climber climber;
    public final Pivot pivot;
    public final Fan fan;
    public final FanPivot fanPivot;
    public final Whippy whippy;
    public final Candle candle;
    public final NoteSensing noteSensing;
    public final DriverCamera driverCamera;
    public final Limelight shooterLimelight, chassisLimelight, infeedLimelight3G;
    public final ShootingStrategy shooterLimelightStrategy, odometryStrategy, chassisLimelight2dStrategy;

    public SubsystemContainer(CommandSwerveDrivetrain drivetrain, Infeed infeed, Shooter shooter, Conveyor conveyor,
            Climber climber, Pivot pivot, Fan fan, FanPivot fanPivot, Whippy whippy, Candle candle,
            NoteSensing noteSensing, DriverCamera driverCamera, Limelight shooterLimelight, Limelight chassisLimelight,
            Limelight infeedLimelight3G, ShootingStrategy shooterLimelightStrategy, ShootingStrategy odometryStrategy,
            ShootingStrategy chassisLimelight2dStrategy) {
        this.drivetrain = drivetrain;
        this.infeed = infeed;
        this.shooter = shooter;
        this.conveyor = conveyor;
        this.climber = climber;
        this.pivot = pivot;
        this.fan = fan;
        this.fanPivot = fanPivot;
        this.whippy = whippy;
        this.candle = candle;
        this.noteSensing = noteSensing;
        this.driverCamera = driverCamera;
        this.shooterLimelight = shooterLimelight;
        this.chassisLimelight = chassisLimelight;
        this.infeedLimelight3G = infeedLimelight3G;
        this.shooterLimelightStrategy = shooterLimelightStrategy;
        this.odometryStrategy = odometryStrategy;
        this.chassisLimelight2dStrategy = chassisLimelight2dStrategy;
    }
}
