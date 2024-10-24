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
import frc.robot.utils.BeakUtils;
import frc.robot.utils.DashboardStore;
import frc.robot.utils.ShootingStrategy;
import frc.robot.utils.SubAutos;
import frc.robot.utils.SubsystemContainer;

public class DashboardLogging {
    private final SubsystemContainer subsystems;
    private final SlewRateLimiter xLimiter, yLimiter, thetaLimiter;
    private final HashMap<ShootingStrategy, String> strategyMap;
    private final LinkedHashMap<Double, String> indexMap = new LinkedHashMap<>();
    private final SubAutos autos;
    private final CommandFactory commandFactory;
    private final RobotContainer robotContainer;

    public DashboardLogging(SubsystemContainer subsystems, SlewRateLimiter xLimiter, SlewRateLimiter yLimiter,
            SlewRateLimiter thetaLimiter, SubAutos autos, RobotContainer robotContainer) {
        this.subsystems = subsystems;
        this.xLimiter = xLimiter;
        this.yLimiter = yLimiter;
        this.thetaLimiter = thetaLimiter;
        this.autos = autos;
        this.robotContainer = robotContainer;
        this.commandFactory = robotContainer.getCommandFactory();

        strategyMap = new HashMap<>(
                Map.of(subsystems.shooterLimelightStrategy, "MVR", subsystems.odometryStrategy, "MegaTag2",
                        subsystems.chassisLimelight2dStrategy, "3G 2D"));

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
            var res = subsystems.chassisLimelight.getTagDistance(7);
            if (res.isPresent())
                return Units.metersToFeet(res.get());
            return Double.NaN;
        });

        // this is weird
        DashboardStore.add("Snapped",
                () -> subsystems.drivetrain.getCurrentRequest().getClass()
                        .equals(SwerveRequest.FieldCentricFacingAngle.class));
        DashboardStore.add("Robot Relative",
                () -> subsystems.drivetrain.getCurrentRequest().getClass().equals(SwerveRequest.RobotCentric.class));

        DashboardStore.add("Manual Indexing", () -> commandFactory.getUseManual());

        DashboardStore.add("Limelight Distance",
                () -> subsystems.shooterLimelightStrategy.getTargetEntry().Distance.in(Feet));
        DashboardStore.add("LimelightG Distance",
                () -> subsystems.chassisLimelight2dStrategy.getTargetEntry().Distance.in(Feet));

        DashboardStore.add("Chassis MT2 Distance", () -> BeakUtils
                .goalTranslation(subsystems.chassisLimelight.getBotposeEstimateMT2().pose.getTranslation()).getNorm());

        DashboardStore.add("Infeed MT2 Distance", () -> BeakUtils
                .goalTranslation(subsystems.infeedLimelight3G.getBotposeEstimateMT2().pose.getTranslation()).getNorm());

        DashboardStore.add("Odometry Distance",
                () -> Units.metersToFeet(BeakUtils.goalTranslation(subsystems.drivetrain.getTranslation()).getNorm()));

        DashboardStore.add("Auton Color", () -> {
            if (robotContainer.getAutonChooserCommand().getName().contains("RED"))
                return "RED";
            else if (AutoBuilder.getAllAutoNames().contains(robotContainer.getAutonChooserCommand().getName() + " RED"))
                return "BLUE";
            else
                return "NULL";
        });

        DashboardStore.add("Infed Cache", subsystems.noteSensing.getHasInfedCache());
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
