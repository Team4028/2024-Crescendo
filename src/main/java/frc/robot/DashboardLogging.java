package frc.robot;

import static edu.wpi.first.units.Units.Feet;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.util.Units;
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
import frc.robot.utils.BeakUtils;
import frc.robot.utils.DashboardStore;
import frc.robot.utils.DriverCamera;
import frc.robot.utils.Limelight;
import frc.robot.utils.NoteSensing;
import frc.robot.utils.ShootingStrategy;
import frc.robot.utils.SubAutos;

public class DashboardLogging {
    private final CommandSwerveDrivetrain drivetrain;
    private final Infeed infeed;
    private final Shooter shooter;
    private final Conveyor conveyor;
    private final Climber climber;
    private final Pivot pivot;
    private final Fan fan;
    private final FanPivot fanPivot;
    private final Whippy whippy;
    private final Candle candle;
    private final NoteSensing noteSensing;
    private final DriverCamera driverCamera;
    private final Limelight shooterLimelight, chassisLimelight, infeedLimelight3G;
    private final ShootingStrategy shooterLimelightStrategy, odometryStrategy, chassisLimelight2dStrategy;
    private final SlewRateLimiter xLimiter, yLimiter, thetaLimiter;
    private final HashMap<ShootingStrategy, String> strategyMap;
    private final LinkedHashMap<Double, String> indexMap = new LinkedHashMap<>();
    private final SubAutos autos;
    private final CommandFactory commandFactory;
    private final RobotContainer robotContainer;

    public DashboardLogging(CommandSwerveDrivetrain drivetrain, Infeed infeed, Shooter shooter, Conveyor conveyor,
            Climber climber, Pivot pivot, Fan fan, FanPivot fanPivot, Whippy whippy, Candle candle,
            NoteSensing noteSensing, DriverCamera driverCamera, Limelight shooterLimelight, Limelight chassisLimelight,
            Limelight infeedLimelight3G, ShootingStrategy shooterLimelightStrategy, ShootingStrategy odometryStrategy,
            ShootingStrategy chassisLimelight2dStrategy, SlewRateLimiter xLimiter, SlewRateLimiter yLimiter,
            SlewRateLimiter thetaLimiter, SubAutos autos, RobotContainer robotContainer) {
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
        this.xLimiter = xLimiter;
        this.yLimiter = yLimiter;
        this.thetaLimiter = thetaLimiter;
        this.autos = autos;
        this.robotContainer = robotContainer;
        this.commandFactory = robotContainer.getCommandFactory();

        strategyMap = new HashMap<>(Map.of(shooterLimelightStrategy, "MVR", odometryStrategy, "MegaTag2",
                chassisLimelight2dStrategy, "3G 2D"));

        /* Init Index Map */
        indexMap.put(4.2, "Speaker");
        indexMap.put(5.2, "SideWoofer");
        indexMap.put(10.0, "Protected");
        indexMap.put(13.0, "Chain");
        indexMap.put(16.0, "Truss");
        indexMap.put(19.0, "Left Wing");
        indexMap.put(22.0, "Right Wing");
    }

    public void initDashboardLogging() {
        DashboardStore.add("Climber Enabled", () -> commandFactory.getEnableClimber());
        DashboardStore.add("Trap Enabled", () -> commandFactory.getEnableTrap());
        DashboardStore.add("Sequence", () -> commandFactory.getCurrentSequence().name());
        DashboardStore.add("Last Shot", () -> commandFactory.getLastShot());
        DashboardStore.add("Strategy", () -> strategyMap.get(commandFactory.getSelectedStrategy()));

        /* Dashboard */
        DashboardStore.add("Shooter Table Index", () -> commandFactory.getCurrentIndex());
        DashboardStore.add("Shooter Table Name",
                () -> indexMap.containsKey(commandFactory.getCurrentIndex())
                        ? indexMap.get(commandFactory.getCurrentIndex())
                        : "Manual");

        DashboardStore.add("Chassis 2D Distance", () -> {
            var res = chassisLimelight.getTagDistance(7);
            if (res.isPresent())
                return Units.metersToFeet(res.get());
            return Double.NaN;
        });

        // this is weird
        DashboardStore.add("Snapped",
                () -> drivetrain.getCurrentRequest().getClass().equals(SwerveRequest.FieldCentricFacingAngle.class));
        DashboardStore.add("Robot Relative",
                () -> drivetrain.getCurrentRequest().getClass().equals(SwerveRequest.RobotCentric.class));

        DashboardStore.add("Manual Indexing", () -> commandFactory.getUseManual());

        DashboardStore.add("Limelight Distance", () -> shooterLimelightStrategy.getTargetEntry().Distance.in(Feet));
        DashboardStore.add("LimelightG Distance", () -> chassisLimelight2dStrategy.getTargetEntry().Distance.in(Feet));

        DashboardStore.add("Chassis MT2 Distance", () -> BeakUtils
                .goalTranslation(chassisLimelight.getBotposeEstimateMT2().pose.getTranslation()).getNorm());

        DashboardStore.add("Infeed MT2 Distance", () -> BeakUtils
                .goalTranslation(infeedLimelight3G.getBotposeEstimateMT2().pose.getTranslation()).getNorm());

        DashboardStore.add("Odometry Distance",
                () -> Units.metersToFeet(BeakUtils.goalTranslation(drivetrain.getTranslation()).getNorm()));

        DashboardStore.add("Auton Color", () -> {
            if (robotContainer.getAutonChooserCommand().getName().contains("RED"))
                return "RED";
            else if (AutoBuilder.getAllAutoNames().contains(robotContainer.getAutonChooserCommand().getName() + " RED"))
                return "BLUE";
            else
                return "NULL";
        });

        DashboardStore.add("Infed Cache", noteSensing.getHasInfedCache());
    }

    /** Put Current ST Index Data to Dashboard */
    public void pushIndexData() {
        commandFactory.setPresetIndex(MathUtil.clamp(commandFactory.getPresetIndex(), 0, indexMap.size() - 1));
        commandFactory.setManualIndex(
                MathUtil.clamp(commandFactory.getManualIndex(), Constants.MIN_INDEX, Constants.MAX_INDEX));

        if (commandFactory.getUseManual()) {
            commandFactory.setCurrentIndex(commandFactory.getManualIndex());
        } else {
            List<Double> keys = new ArrayList<>(indexMap.keySet());
            commandFactory.setCurrentIndex(keys.get(commandFactory.getPresetIndex()));
            commandFactory.setManualIndex(commandFactory.getCurrentIndex());
        }
    }
}
