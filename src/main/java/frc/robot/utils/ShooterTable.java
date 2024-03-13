package frc.robot.utils;

import static edu.wpi.first.units.Units.Feet;

import java.util.ArrayList;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;

public class ShooterTable {
    // TODO: We may want P values here, or just tune kF
    public static final class ShooterTableEntry {
        public Measure<Distance> Distance;
        public double CameraDistance;
        public double Angle;
        public double Percent;

        /**
         * Construct a shooter table entry.
         * 
         * @param distance   Distance from the target, in feet (?)
         * @param angle      Angle of the shooter pivot, in rotations.
         * @param leftSpeed  Speed of the left shooter motor, in RPM.
         * @param rightSpeed Speed of the right shooter motor, in RPM.
         */
        public ShooterTableEntry(Measure<Distance> distance, double cameraDistance, double angle, double percent) {
            this.Angle = angle;
            this.Percent = percent;
            this.Distance = distance;
            this.CameraDistance = cameraDistance;
        }
    }

    private static ArrayList<ShooterTableEntry> table = new ArrayList<>();

    private static void fillInTable() {
        // put entries here
        // Distances must go from top to bottom: shortest to longest
        table.add(new ShooterTableEntry(Feet.of(4.2), 0.0, 30.9, 0.6)); // 55 degrees
        table.add(new ShooterTableEntry(Feet.of(5.0), 0.0, 27.0, 0.7)); // 50 degrees
        table.add(new ShooterTableEntry(Feet.of(6.0), 0.0, 22.8, 0.8)); // 45 degrees
        table.add(new ShooterTableEntry(Feet.of(8.0), 0.0, 16., 1.0)); // 36 degrees
        table.add(new ShooterTableEntry(Feet.of(10.), 0.0, 12.1, 1.0)); // 31 degrees
        table.add(new ShooterTableEntry(Feet.of(13.), 0.0, 6.7, 1.0)); // 25 degrees
        table.add(new ShooterTableEntry(Feet.of(16.), 0.0, 4.3, 1.0)); // 23 degrees
        table.add(new ShooterTableEntry(Feet.of(19.), 0.0, 3.25, 1.0)); // 21.5 degrees
        table.add(new ShooterTableEntry(Feet.of(22.), 0.0, 2.24, 1.0)); // 20.5 degrees
        table.add(new ShooterTableEntry(Feet.of(27.), 0.0, 0.25, 0.95)); // 20 degrees
    }

    static {
        fillInTable();
    }

    public static ShooterTableEntry calcShooterTableEntryByPercent(double percent) {
        Measure<Distance> size = table.get(table.size() - 1).Distance.minus(table.get(0).Distance);
        return calcShooterTableEntry(size.times(percent));
    }

    public static ShooterTableEntry calcShooterTableEntryCamera(double cameraDistance) {
        ShooterTableEntry cL = table.get(0);
        ShooterTableEntry cH = table.get(table.size() - 1);

        if (cameraDistance <= cL.CameraDistance)
            return cL;
        if (cameraDistance >= cH.CameraDistance)
            return cH;

        for (var entry : table) {
            if (entry.CameraDistance < cameraDistance && (Math.abs(cameraDistance - cL.CameraDistance) > Math
                    .abs(cameraDistance - entry.CameraDistance))) {
                cL = entry;
            } else if (entry.CameraDistance > cameraDistance && (Math.abs(cameraDistance - cH.CameraDistance) > Math
                    .abs(cameraDistance - entry.CameraDistance))) {
                cH = entry;
            } else if (entry.CameraDistance == cameraDistance) {
                return entry;
            }
        }

        double scaleFactor = (cameraDistance - cL.CameraDistance) / (cH.CameraDistance - cL.CameraDistance);

        return new ShooterTableEntry(cH.Distance.minus(cL.Distance).times(scaleFactor).plus(cL.Distance),
                cameraDistance, scaleFactor * (cH.Angle - cL.Angle) + cL.Angle,
                scaleFactor * (cH.Percent - cL.Percent)
                        + cL.Percent);
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

        double calculatedCameraDist = scaleFactor * (closestHigher.CameraDistance - closestLower.CameraDistance)
                + closestLower.CameraDistance;

        double calculatedPercent = scaleFactor * (closestHigher.Percent - closestLower.Percent)
                + closestLower.Percent;

        double calculatedAngle = scaleFactor * (closestHigher.Angle - closestLower.Angle) + closestLower.Angle;

        return new ShooterTableEntry(distance, calculatedCameraDist, calculatedAngle, calculatedPercent);
    }

}
