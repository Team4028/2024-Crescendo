package frc.robot.utils;

import static edu.wpi.first.units.Units.Feet;

import java.util.ArrayList;
import java.util.function.Function;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.utils.ShooterTable.VisionTableEntry.CameraLerpStrat;

public class ShooterTable {
    public static final class ShooterTableEntry {
        public Measure<Distance> Distance;
        public double Angle;
        public double Beans;

        /**
         * Construct a shooter table entry.
         * 
         * @param distance       Distance from the target
         * @param photonDistance The arb dist reported by garbage PV
         * @param llTY           The arb ty reported by garbage LL
         * @param angle          Angle of the shooter pivot, in rotations.
         * @param leftSpeed      Speed of the left shooter motor, in RPM.
         * @param rightSpeed     Speed of the right shooter motor, in RPM.
         */
        public ShooterTableEntry(Measure<Distance> distance, double angle, double beans) {
            Angle = angle;
            Beans = beans;
            Distance = distance;
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
    private static ArrayList<VisionTableEntry> visionTable = new ArrayList<>();
    private static Interpolator<Measure<Distance>> distanceInterpolation = (startValue, endValue, t) -> endValue
            .minus(startValue).times(MathUtil.clamp(t, 0, 1)).plus(startValue);

    private static Interpolator<Double> cameraAngleInterpolation = Interpolator.forDouble();

    private static void fillInTable() { // TODO: fill in photonStationaryDistance values
        // put entries here
        // Distances must go from top to bottom: shortest to longest
        // IF YOU INCREASE DISTANCE, SHOOTER ANGLE GOES UP
        shooterTable.add(new ShooterTableEntry(Feet.of(4.4), 30.9 + 0.25, 0.6)); // 55 degrees
        shooterTable.add(new ShooterTableEntry(Feet.of(5), 28.0 + 0.25, 0.7)); // 50 degrees
        shooterTable.add(new ShooterTableEntry(Feet.of(6), 23.8 + 0.25, 0.8)); // 45 degrees
        shooterTable.add(new ShooterTableEntry(Feet.of(8), 17.0 + 0.25, 0.9)); // 36 degrees
        shooterTable.add(new ShooterTableEntry(Feet.of(10), 13.1 + 0.25, 1.0)); // 31 degrees
        shooterTable.add(new ShooterTableEntry(Feet.of(11.5), 10.2 + 0.25, 1.0));
        shooterTable.add(new ShooterTableEntry(Feet.of(13), 7.7 + 0.25, 1.0)); // 25 degrees
        shooterTable.add(new ShooterTableEntry(Feet.of(14.5), 6.5 + 0.25, 1.0));
        shooterTable.add(new ShooterTableEntry(Feet.of(16), 5.1 + 0.25, 1.0)); // 23 degrees
        shooterTable.add(new ShooterTableEntry(Feet.of(17.5), 4.9 + 0.25, 1.0));
        shooterTable.add(new ShooterTableEntry(Feet.of(19), 4.4 + 0.25, 1.0)); // 21.5 degrees       
        shooterTable.add(new ShooterTableEntry(Feet.of(20.5), 3.58 + 0.25, 1.0));     
        shooterTable.add(new ShooterTableEntry(Feet.of(22), 3.1 + 0.25, 1.0)); // 20.5 degrees
        shooterTable.add(new ShooterTableEntry(Feet.of(24), 2.6 + 0.25, 1.0));
        shooterTable.add(new ShooterTableEntry(Feet.of(27), 0.25 + 0.25, 0.98)); // 20 degrees

        // vision table entries
        visionTable.add(new VisionTableEntry(Feet.of(4.4), 4.34, 0, 21.3, 15.42 ));
        visionTable.add(new VisionTableEntry(Feet.of(5), 5.09, 1.565, 16.62, 10.77));
        visionTable.add(new VisionTableEntry(Feet.of(6), 6.094, 1.884, 9.15, 3.52));
        visionTable.add(new VisionTableEntry(Feet.of(8), 7.799, 2.392, 0.0, -5.92));
        visionTable.add(new VisionTableEntry(Feet.of(10), 9.17, 2.821, -5.29, -11.46));
        visionTable.add(new VisionTableEntry(Feet.of(11.5), 9.17, 2.821, -8.93, -15.05));
        visionTable.add(new VisionTableEntry(Feet.of(13), 11.34, 3.443, -11.15, -17.42));
        visionTable.add(new VisionTableEntry(Feet.of(14.5), 11.34, 3.443, -12.91, -19.32));
        visionTable.add(new VisionTableEntry(Feet.of(16), 13.115, 3.954, -14.46, -20.99));
        visionTable.add(new VisionTableEntry(Feet.of(17.5), 13.115, 3.954, -14.84, -21.38));
        visionTable.add(new VisionTableEntry(Feet.of(19), 13.845, 4.168, -15.73, -22.38));
        visionTable.add(new VisionTableEntry(Feet.of(20.5), 13.845, 4.168, -16.68, -23.4));
        visionTable.add(new VisionTableEntry(Feet.of(22), 15.095, 4.504, -17.40, -24.1));
        visionTable.add(new VisionTableEntry(Feet.of(24), 15.095, 4.504, -18.22, -24.99));
        visionTable.add(new VisionTableEntry(Feet.of(27), 17.24, 5.111, -19.68, -26.57));
    }

    static {
        fillInTable();
    }

    // public static ShooterTableEntry calcShooterTableEntryByPercent(double
    // percent) {
    // Measure<Distance> size = shooterTable.get(shooterTable.size() -
    // 1).Distance.minus(shooterTable.get(0).Distance);
    // return calcShooterTableEntry(size.times(percent));
    // }

    public static ShooterTableEntry calcShooterTableEntryCamera(double cameraValue, CameraLerpStrat strategy) {
        return calcShooterTableEntry(calcVisionTableEntryCamera(cameraValue, strategy).Distance);
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

        double calculatedPercent = scaleFactor * (closestHigher.Beans -
                closestLower.Beans)
                + closestLower.Beans;

        double calculatedAngle = scaleFactor * (closestHigher.Angle -
                closestLower.Angle) + closestLower.Angle;

        return new ShooterTableEntry(distance, calculatedAngle, calculatedPercent);
    }

}
