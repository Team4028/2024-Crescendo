package frc.robot.utils;

import static edu.wpi.first.units.Units.Feet;

import java.util.ArrayList;
import java.util.function.Function;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utils.ShooterTable.VisionTableEntry.CameraLerpStrat;

public class ShooterTable {
    private static final boolean AT_HOME_SHUTTLE_TABLE = false;
    
    public static final class ShooterTableEntry {
        public Measure<Distance> Distance;
        public double Angle;
        public double Beans;
        public Measure<Distance> HeckyOffset;


        public ShooterTableEntry(Measure<Distance> distance, double angle, double beans,
                Measure<Distance> heckyOffset) {
            Angle = angle;
            Beans = beans;
            Distance = distance;
            HeckyOffset = heckyOffset;
        }

        public ShooterTableEntry average(ShooterTableEntry other) {
            return new ShooterTableEntry(Distance.plus(other.Distance).divide(2.), (Angle + other.Angle) / 2,
                    (Beans + other.Beans) / 2, Feet.zero());
        }

        public ShooterTableEntry applyHeckyOffset() {
            double rotationThreshold = 30.0;
            double allianceZeroAngle = BeakUtils.allianceIsBlue() ? 0 : 180;
            if (heckinessSupplier != null && Math.abs(heckinessSupplier.get().getDegrees() - allianceZeroAngle) > rotationThreshold) {
                Distance = Distance.plus(HeckyOffset);
            }
            
            return this;
        }
    }

    public static final class VisionTableEntry {
        public Measure<Distance> Distance;
        public double PhotonDistance;
        public double PhotonStationaryDistance;
        public double LLTY;
        public double LL3GTY;

        public static enum CameraLerpStrat {
            LimelightTY,
            Limelight3GTY,
            PhotonVisionDistance,
            PhotonVisionStationaryDistance;
        }

        public VisionTableEntry(Measure<Distance> distance, double photonDistance, double photonStationaryDistance,
                double llTY, double ll3GTY) {
            Distance = distance;
            PhotonDistance = photonDistance;
            PhotonStationaryDistance = photonStationaryDistance;
            LLTY = llTY;
            LL3GTY = ll3GTY;
        }
    }

    private static ArrayList<ShooterTableEntry> shooterTable = new ArrayList<>();
    private static ArrayList<ShooterTableEntry> shuttleTable = new ArrayList<>();
    private static ArrayList<VisionTableEntry> visionTable = new ArrayList<>();

    private static Supplier<Rotation2d> heckinessSupplier;

    private static void fillInTable() { // TODO: fill in photonStationaryDistance values
        // put entries here
        // Distances must go from top to bottom: shortest to longest
        // IF YOU INCREASE DISTANCE, SHOOTER ANGLE GOES UP
        shooterTable.add(new ShooterTableEntry(Feet.of(4.4), 30.9 + 0.25, 0.6, Feet.of(0.0))); // 55 degrees
        shooterTable.add(new ShooterTableEntry(Feet.of(5), 28.0 + 0.25, 0.7, Feet.of(0.0))); // 50 degrees
        shooterTable.add(new ShooterTableEntry(Feet.of(6), 23.8 + 0.25, 0.8, Feet.of(0.0))); // 45 degrees
        shooterTable.add(new ShooterTableEntry(Feet.of(8), 17.0 + 0.25, 0.9, Feet.of(0.0))); // 36 degrees
        shooterTable.add(new ShooterTableEntry(Feet.of(10), 13.1 + 0.25, 1.0, Feet.of(0.0))); // 31 degrees
        shooterTable.add(new ShooterTableEntry(Feet.of(11.5), 10.2 + 0.25, 1.0, Feet.of(0.0)));
        shooterTable.add(new ShooterTableEntry(Feet.of(13), 7.7 + 0.25, 1.0, Feet.of(0.0))); // 25 degrees
        shooterTable.add(new ShooterTableEntry(Feet.of(14.5), 6.5 + 0.25, 1.0, Feet.of(0.0)));
        shooterTable.add(new ShooterTableEntry(Feet.of(16), 5.1 + 0.25, 1.0, Feet.of(0.0))); // 23 degrees
        shooterTable.add(new ShooterTableEntry(Feet.of(17.5), 4.9 + 0.25, 1.0, Feet.of(0.0)));
        shooterTable.add(new ShooterTableEntry(Feet.of(19), 4.4 + 0.25, 1.0, Feet.of(0.0))); // 21.5 degrees       
        shooterTable.add(new ShooterTableEntry(Feet.of(20.5), 3.58 + 0.25, 1.0, Feet.of(0.0)));     
        shooterTable.add(new ShooterTableEntry(Feet.of(22), 3.1 + 0.25, 1.0, Feet.of(0.0))); // 20.5 degrees
        shooterTable.add(new ShooterTableEntry(Feet.of(24), 2.6 + 0.25, 1.0, Feet.of(0.0)));
        shooterTable.add(new ShooterTableEntry(Feet.of(27), 0.25 + 0.25, 0.98, Feet.of(0.0))); // 20 degrees

        // shuttle table entries
        if (AT_HOME_SHUTTLE_TABLE) {
            shuttleTable.add(new ShooterTableEntry(Feet.of(27), 18, 0.55, Feet.zero()));
            shuttleTable.add(new ShooterTableEntry(Feet.of(31), 14, 0.55, Feet.zero()));
            shuttleTable.add(new ShooterTableEntry(Feet.of(36), 12, 0.65, Feet.zero()));
        } else {
            shuttleTable.add(new ShooterTableEntry(Feet.of(27), 28.5, 0.63, Feet.zero()));
            shuttleTable.add(new ShooterTableEntry(Feet.of(31), 28.5, 0.675, Feet.zero()));
            shuttleTable.add(new ShooterTableEntry(Feet.of(36), 24, 0.70, Feet.zero()));
        }

        
        // vision table entries
        visionTable.add(new VisionTableEntry(Feet.of(4.4), 4.34, 0, 20.87, 12.57));
        visionTable.add(new VisionTableEntry(Feet.of(5), 5.09, 1.565, 16.2, 7.81));
        visionTable.add(new VisionTableEntry(Feet.of(6), 6.094, 1.884, 8.83, 0.57));
        visionTable.add(new VisionTableEntry(Feet.of(8), 7.799, 2.392, -0.59, -8.61));
        visionTable.add(new VisionTableEntry(Feet.of(10), 9.17, 2.821, -5.88, -14.02));
        visionTable.add(new VisionTableEntry(Feet.of(11.5), 9.17, 2.821, -9.45, -17.53));
        visionTable.add(new VisionTableEntry(Feet.of(13), 11.34, 3.443, -11.76, -19.95));
        visionTable.add(new VisionTableEntry(Feet.of(14.5), 11.34, 3.443, -13.45, -21.98));
        visionTable.add(new VisionTableEntry(Feet.of(16), 13.115, 3.954, -15.03, -23.49));
        visionTable.add(new VisionTableEntry(Feet.of(17.5), 13.115, 3.954, -15.4, -23.93));
        visionTable.add(new VisionTableEntry(Feet.of(19), 13.845, 4.168, -16.26, -24.97));
        visionTable.add(new VisionTableEntry(Feet.of(20.5), 13.845, 4.168, -17.18, -25.89));
        visionTable.add(new VisionTableEntry(Feet.of(22), 15.095, 4.504, -17.93, -26.7));
        visionTable.add(new VisionTableEntry(Feet.of(24), 15.095, 4.504, -18.77, -27.55));
        visionTable.add(new VisionTableEntry(Feet.of(27), 17.24, 5.111, -20.32, -29.14));
    }

    static {
        fillInTable();
    }

    // public static ShooterTableEntry calcShooterTableEntryByBeans(double
    // beans) {
    // Measure<Distance> size = shooterTable.get(shooterTable.size() -
    // 1).Distance.minus(shooterTable.get(0).Distance);
    // return calcShooterTableEntry(size.times(beans));
    // }

    public static ShooterTableEntry calcShooterTableEntryCamera(double cameraValue, CameraLerpStrat strategy) {
        return calcShooterTableEntry(calcVisionTableEntryCamera(cameraValue, strategy).Distance.plus(Feet.of(0.2))).applyHeckyOffset();
    }

    public static VisionTableEntry calcVisionTableEntryCamera(double cameraValue, CameraLerpStrat strategy) {
        VisionTableEntry closestLower = visionTable.get(0);
        VisionTableEntry closestHigher = visionTable.get(visionTable.size() - 1);

        Function<VisionTableEntry, Double> cameraValueGetter;

        // SmartDashboard.putNumber("epic ty", cameraValue);

        switch (strategy) {
            case LimelightTY:
                // negate everything because TY is inversely proportional to distance
                cameraValueGetter = (vte) -> -vte.LLTY;
                cameraValue = -cameraValue;
                break;
            case Limelight3GTY:
                // negate everything because TY is inversely proportional to distance
                cameraValueGetter = (vte) -> -vte.LL3GTY;
                cameraValue = -cameraValue;
                break;
            case PhotonVisionStationaryDistance:
                cameraValueGetter = (vte) -> vte.PhotonStationaryDistance;
                break;
            default:
                cameraValueGetter = (vte) -> vte.PhotonDistance;
                break;
        }

        if (cameraValue <= cameraValueGetter.apply(closestLower))
            return closestLower;

        if (cameraValue >= cameraValueGetter.apply(closestHigher))
            return closestHigher;

        for (var entry : visionTable) {
            // entry is less than distancce
            // delta b/t lower & dist > entry & dist (entry closer to dist than lower)
            if (cameraValueGetter.apply(entry) < cameraValue
                    && (Math.abs(cameraValue - cameraValueGetter.apply(closestLower)) > Math
                            .abs(cameraValue - cameraValueGetter.apply(entry)))) {
                closestLower = entry;
                // entry is greater than distancce
                // delta b/t higher & dist > entry & dist (entry closer to dist than higher)
            } else if (cameraValueGetter.apply(entry) > cameraValue
                    && (Math.abs(cameraValue - cameraValueGetter.apply(closestHigher)) > Math
                            .abs(cameraValue - cameraValueGetter.apply(entry)))) {
                closestHigher = entry;
            } else if (cameraValueGetter.apply(entry) == cameraValue) {
                return entry;
            }
        }

        double scaleFactor = (cameraValue - cameraValueGetter.apply(closestLower))
                / (cameraValueGetter.apply(closestHigher) -
                        cameraValueGetter.apply(closestLower));

        scaleFactor = Math.abs(scaleFactor);

        Measure<Distance> interpolatedDistance = closestHigher.Distance.minus(closestLower.Distance).times(scaleFactor)
                .plus(closestLower.Distance);

        return new VisionTableEntry(interpolatedDistance, cameraValue, cameraValue, cameraValue, cameraValue);
    }

    public static ShooterTableEntry calcShuttleTableEntry(Measure<Distance> distance) {
        ShooterTableEntry closestLower = shuttleTable.get(0);
        ShooterTableEntry closestHigher = shuttleTable.get(shuttleTable.size() - 1);

        if (distance.lte(closestLower.Distance))
            return closestLower;
        if (distance.gte(closestHigher.Distance))
            return closestHigher;

        // loop thru all of the entrys of the shootertable
        for (ShooterTableEntry entry : shooterTable) {
            if (entry.Distance.lt(distance)
                    && (Math.abs(distance.minus(closestLower.Distance).baseUnitMagnitude()) > Math
                            .abs(distance.minus(entry.Distance).baseUnitMagnitude()))) {
                closestLower = entry;
            } else if (entry.Distance.gt(distance)
                    && (Math.abs(closestHigher.Distance.minus(distance).baseUnitMagnitude()) > Math
                            .abs(entry.Distance.minus(distance).baseUnitMagnitude()))) {
                closestHigher = entry;
            } else if (entry.Distance.isEquivalent(distance)) {
                return entry;
            }
        }

        double scaleFactor = (distance.minus(closestLower.Distance).baseUnitMagnitude())
                / (closestHigher.Distance.minus(closestLower.Distance).baseUnitMagnitude());

        double calculatedBeans = scaleFactor * (closestHigher.Beans -
                closestLower.Beans)
                + closestLower.Beans;

        double calculatedAngle = scaleFactor * (closestHigher.Angle -
                closestLower.Angle) + closestLower.Angle;

        var calculatedHeckyOffset = closestHigher.HeckyOffset.minus(closestLower.HeckyOffset).times(scaleFactor)
                .plus(closestLower.HeckyOffset);

        return new ShooterTableEntry(distance, calculatedAngle, calculatedBeans, calculatedHeckyOffset);
    }

    public static ShooterTableEntry calcShooterTableEntry(Measure<Distance> distance) {
        SmartDashboard.putNumber("Index", distance.in(Feet));
        ShooterTableEntry closestLower = shooterTable.get(0);
        ShooterTableEntry closestHigher = shooterTable.get(shooterTable.size() - 1);

        if (distance.lte(closestLower.Distance))
            return closestLower;
        if (distance.gte(closestHigher.Distance))
            return closestHigher;

        // loop thru all of the entrys of the shootertable
        for (ShooterTableEntry entry : shooterTable) {
            if (entry.Distance.lt(distance)
                    && (Math.abs(distance.minus(closestLower.Distance).baseUnitMagnitude()) > Math
                            .abs(distance.minus(entry.Distance).baseUnitMagnitude()))) {
                closestLower = entry;
            } else if (entry.Distance.gt(distance)
                    && (Math.abs(closestHigher.Distance.minus(distance).baseUnitMagnitude()) > Math
                            .abs(entry.Distance.minus(distance).baseUnitMagnitude()))) {
                closestHigher = entry;
            } else if (entry.Distance.isEquivalent(distance)) {
                return entry;
            }
        }

        double scaleFactor = (distance.minus(closestLower.Distance).baseUnitMagnitude())
                / (closestHigher.Distance.minus(closestLower.Distance).baseUnitMagnitude());

        double calculatedBeans = scaleFactor * (closestHigher.Beans -
                closestLower.Beans)
                + closestLower.Beans;

        double calculatedAngle = scaleFactor * (closestHigher.Angle -
                closestLower.Angle) + closestLower.Angle;

        var calculatedHeckyOffset = closestHigher.HeckyOffset.minus(closestLower.HeckyOffset).times(scaleFactor)
                .plus(closestLower.HeckyOffset);

        return new ShooterTableEntry(distance, calculatedAngle, calculatedBeans, calculatedHeckyOffset);
    }

    public static void setHeckinessLevel(Supplier<Rotation2d> heckSource) {
        heckinessSupplier = heckSource;
    }

}
