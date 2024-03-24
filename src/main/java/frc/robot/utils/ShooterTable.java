package frc.robot.utils;

import static edu.wpi.first.units.Units.Feet;

import java.util.ArrayList;
import java.util.function.Function;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utils.ShooterTable.ShooterTableEntry.CameraLerpStrat;

public class ShooterTable {
    public static final class ShooterTableEntry {
        public Measure<Distance> Distance;
        public double PhotonDistance;
        public double LLTY;
        public double LLTA;
        public double LLTAMulti;
        public double Angle;
        public double Percent;

        public static enum CameraLerpStrat {
            LimelightTY,
            LimelightArea,
            LimelightMultiTagArea,
            PhotonVisionDistance;
        }

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
        public ShooterTableEntry(Measure<Distance> distance, double photonDistance, double llTY, double llTA,
                double llTAMulti,
                double angle,
                double percent) {
            Angle = angle;
            Percent = percent;
            Distance = distance;
            PhotonDistance = photonDistance;
            LLTY = llTY;
            LLTA = llTA;
            LLTAMulti = llTAMulti;
        }
    }

    private static ArrayList<ShooterTableEntry> table = new ArrayList<>();

    private static void fillInTable() {
        // put entries here
        // Distances must go from top to bottom: shortest to longest
        table.add(new ShooterTableEntry(Feet.of(4.2), 4.26, 20.11, 0.97, 4.5, 30.9, 0.6)); // 55 degrees
        table.add(new ShooterTableEntry(Feet.of(5.0), 5.06, 13.63, 0.785, 3.757, 27.0, 0.7)); // 50 degrees
        table.add(new ShooterTableEntry(Feet.of(6.0), 6.05, 6.25, 0.598, 2.796, 22.8, 0.8)); // 45 degrees
        table.add(new ShooterTableEntry(Feet.of(8.0), 7.75, -3.13, 0.368, 1.761, 16., 1.0)); // 36 degrees
        table.add(new ShooterTableEntry(Feet.of(10.), 9.14, -8.3, 0.257, 1.195, 12.1, 1.0)); // 31 degrees
        table.add(new ShooterTableEntry(Feet.of(13.), 11.25, -14.06, 0.166, 0.78, 6.7, 1.0)); // 25 degrees
        table.add(new ShooterTableEntry(Feet.of(16.), 13.02, -17.5, 0.114, 0.528, 4.3, 1.0)); // 23 degrees
        table.add(new ShooterTableEntry(Feet.of(19.), 13.71, -18.59, 0.082, 0.38, 3.25, 1.0)); // 21.5 degrees
        table.add(new ShooterTableEntry(Feet.of(22.), 14.92, -20.26, 0.064, 0.29, 2.24, 1.0)); // 20.5 degrees
        table.add(new ShooterTableEntry(Feet.of(27.), 17.16, -22, 0.045, 0.25, 0.25, 1.0)); // 20 degrees
    }

    static {
        fillInTable();
    }

    public static ShooterTableEntry calcShooterTableEntryByPercent(double percent) {
        Measure<Distance> size = table.get(table.size() - 1).Distance.minus(table.get(0).Distance);
        return calcShooterTableEntry(size.times(percent));
    }

    public static ShooterTableEntry calcShooterTableEntryCamera(double cameraValue, CameraLerpStrat strategy) {
        System.out.println("bruv, " + cameraValue);
        ShooterTableEntry closestLower = table.get(0);
        ShooterTableEntry closestHigher = table.get(table.size() - 1);

        Function<ShooterTableEntry, Double> cameraValueGetter;

        SmartDashboard.putNumber("epic ty", cameraValue);

        switch (strategy) {
            case LimelightTY:
                // negate everything because TY is inversely proportional to distance
                cameraValueGetter = (ste) -> -ste.LLTY;
                cameraValue = -cameraValue;
                break;
            case LimelightArea:
                // negate everything because TA is inversely proportional to distance
                cameraValueGetter = (ste) -> -ste.LLTA;
                cameraValue = -cameraValue;
            case LimelightMultiTagArea:
                cameraValueGetter = (ste) -> -ste.LLTAMulti;
                cameraValue = -cameraValue;
            default:
                cameraValueGetter = (ste) -> ste.PhotonDistance;
                break;
        }

        if (cameraValue <= cameraValueGetter.apply(closestLower))
            return closestLower;
        if (cameraValue >= cameraValueGetter.apply(closestHigher))
            return closestHigher;

        for (var entry : table) {
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

        // 20.5 - 20.44 = 0.06
        // 26.2 - 20.5 = 5.7

        double scaleFactor = (cameraValue - cameraValueGetter.apply(closestLower))
                / (cameraValueGetter.apply(closestHigher) - cameraValueGetter.apply(closestLower));

        SmartDashboard.putNumber("Scale Factor", scaleFactor);

        scaleFactor = Math.abs(scaleFactor);

        Measure<Distance> interpolatedDistance = closestHigher.Distance.minus(closestLower.Distance).times(scaleFactor)
                .plus(closestLower.Distance);

        SmartDashboard.putNumber("Interpolated Distance", interpolatedDistance.in(Feet));
        SmartDashboard.putNumber("Higher", closestHigher.Distance.in(Feet));
        SmartDashboard.putNumber("Lower", closestLower.Distance.in(Feet));

        double interpolatedAngle = scaleFactor * (closestHigher.Angle - closestLower.Angle) + closestLower.Angle;

        double interpolatedShooterPercent = scaleFactor * (closestHigher.Percent - closestLower.Percent)
                + closestLower.Percent;

        return new ShooterTableEntry(interpolatedDistance, cameraValue, cameraValue, cameraValue, cameraValue,
                interpolatedAngle, interpolatedShooterPercent);
    }

    public static ShooterTableEntry calcShooterTableEntry(Measure<Distance> distance) {
        SmartDashboard.putNumber("Index", distance.in(Feet));
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
                calculatedCameraDist,
                calculatedAngle, calculatedPercent);
    }

}
