package frc.robot.utils;

import static edu.wpi.first.units.Units.Feet;

import java.util.ArrayList;
import java.util.function.DoubleSupplier;
import java.util.function.Function;
import java.util.function.UnaryOperator;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.UnaryFunction;
import frc.robot.utils.ShooterTable.ShooterTableEntry.CameraLerpStrat;

public class ShooterTable {
    // TODO: We may want P values here, or just tune kF
    public static final class ShooterTableEntry {
        public Measure<Distance> Distance;
        public double PhotonDistance;
        public double LLTY;
        public double LLTA;
        public double Angle;
        public double Percent;

        public static enum CameraLerpStrat {
            LimelightTY,
            LimelightArea,
            PhotonVisionDistance;
        }

        /**
         * Construct a shooter table entry.
         * 
         * @param distance       Distance from the target, in feet (?)
         * @param photonDistance The arb dist reported by garbage PV
         * @param llTY           The arb ty reported by garbage LL
         * @param angle          Angle of the shooter pivot, in rotations.
         * @param leftSpeed      Speed of the left shooter motor, in RPM.
         * @param rightSpeed     Speed of the right shooter motor, in RPM.
         */
        public ShooterTableEntry(Measure<Distance> distance, double photonDistance, double llTY, double llTA,
                double angle,
                double percent) {
            this.Angle = angle;
            this.Percent = percent;
            this.Distance = distance;
            this.PhotonDistance = photonDistance;
            this.LLTY = llTY;
            this.LLTA = llTA;
        }
    }

    private static ArrayList<ShooterTableEntry> table = new ArrayList<>();

    private static void fillInTable() {
        // put entries here
        // Distances must go from top to bottom: shortest to longest
        table.add(new ShooterTableEntry(Feet.of(4.2), 0.0, 16.29, 0.772, 30.9, 0.6)); // 55 degrees
        table.add(new ShooterTableEntry(Feet.of(5.0), 0.0, 10.27, 0.65, 27.0, 0.7)); // 50 degrees
        table.add(new ShooterTableEntry(Feet.of(6.0), 0.0, 3.35, 0.541, 22.8, 0.8)); // 45 degrees
        table.add(new ShooterTableEntry(Feet.of(8.0), 0.0, -5.8, 0.386, 16., 1.0)); // 36 degrees
        table.add(new ShooterTableEntry(Feet.of(10.), 0.0, -11.26, 0.289, 12.1, 1.0)); // 31 degrees
        table.add(new ShooterTableEntry(Feet.of(13.), 0.0, -17.5, 0.191, 6.7, 1.0)); // 25 degrees
        table.add(new ShooterTableEntry(Feet.of(16.), 0.0, -20.96, 0.142, 4.3, 1.0)); // 23 degrees
        table.add(new ShooterTableEntry(Feet.of(19.), 0.0, -22.33, 0.106, 3.25, 1.0)); // 21.5 degrees
        table.add(new ShooterTableEntry(Feet.of(22.), 0.0, -30, 0.1, 2.24, 1.0)); // 20.5 degrees
        table.add(new ShooterTableEntry(Feet.of(27.), 0.0, -35, 0.08, 0.25, 0.95)); // 20 degrees
    }

    static {
        fillInTable();
    }

    public static ShooterTableEntry calcShooterTableEntryByPercent(double percent) {
        Measure<Distance> size = table.get(table.size() - 1).Distance.minus(table.get(0).Distance);
        return calcShooterTableEntry(size.times(percent));
    }

    public static ShooterTableEntry calcShooterTableEntryCamera(double cameraDistance, CameraLerpStrat strategy) {
        ShooterTableEntry closestLower = table.get(0);
        ShooterTableEntry closestHigher = table.get(table.size() - 1);

        Function<ShooterTableEntry, Double> cameraDistanceGetter;

        switch (strategy) {
            case LimelightTY:
                // negate everything because TY is inversely proportional to distance
                cameraDistanceGetter = (ste) -> -ste.LLTY;
                cameraDistance = -cameraDistance;
                break;
            case LimelightArea:
                // negate everything because TA is inversely proportional to distance
                cameraDistanceGetter = (ste) -> -ste.LLTA;
                cameraDistance = -cameraDistance;
            default:
                cameraDistanceGetter = (ste) -> ste.PhotonDistance;
                break;
        }

        if (cameraDistance <= cameraDistanceGetter.apply(closestLower))
            return closestLower;
        if (cameraDistance >= cameraDistanceGetter.apply(closestHigher))
            return closestHigher;

        for (var entry : table) {
            if (cameraDistanceGetter.apply(entry) < cameraDistance
                    && (Math.abs(cameraDistance - cameraDistanceGetter.apply(closestLower)) > Math
                            .abs(cameraDistance - cameraDistanceGetter.apply(entry)))) {
                closestLower = entry;
            } else if (cameraDistanceGetter.apply(entry) > cameraDistance
                    && (Math.abs(cameraDistance - cameraDistanceGetter.apply(closestHigher)) > Math
                            .abs(cameraDistance - cameraDistanceGetter.apply(entry)))) {
                closestHigher = entry;
            } else if (cameraDistanceGetter.apply(entry) == cameraDistance) {
                return entry;
            }
        }

        double scaleFactor = (cameraDistance - cameraDistanceGetter.apply(closestLower))
                / (cameraDistanceGetter.apply(closestHigher) - cameraDistanceGetter.apply(closestLower));

        scaleFactor = Math.abs(scaleFactor);

        Measure<Distance> interpolatedDistance = closestHigher.Distance.minus(closestLower.Distance).times(scaleFactor)
                .plus(closestLower.Distance);

        double interpolatedAngle = scaleFactor * (closestHigher.Angle - closestLower.Angle) + closestLower.Angle;

        double interpolatedShooterPercent = scaleFactor * (closestHigher.Percent - closestLower.Percent)
                + closestLower.Percent;

        return new ShooterTableEntry(interpolatedDistance, cameraDistance, cameraDistance, cameraDistance,
                interpolatedAngle, interpolatedShooterPercent);
    }

    public static ShooterTableEntry calcShooterTableEntry(Measure<Distance> distance) {
        ShooterTableEntry closestLower = table.get(0);
        ShooterTableEntry closestHigher = table.get(table.size() - 1);

        if (distance.lte(closestLower.Distance))
            return closestLower;
        if (distance.gte(closestHigher.Distance))
            return closestHigher;

        // loop thru all of the entrys of the shootertable
        for (ShooterTableEntry entry : table) {
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

        double calculatedCameraDist = scaleFactor * (closestHigher.PhotonDistance - closestLower.PhotonDistance)
                + closestLower.PhotonDistance;

        double calculatedPercent = scaleFactor * (closestHigher.Percent - closestLower.Percent)
                + closestLower.Percent;

        double calculatedAngle = scaleFactor * (closestHigher.Angle - closestLower.Angle) + closestLower.Angle;

        return new ShooterTableEntry(distance, calculatedCameraDist, calculatedCameraDist, calculatedCameraDist,
                calculatedAngle, calculatedPercent);
    }

}
