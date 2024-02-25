package frc.robot.utils;

import java.util.ArrayList;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;

public class ShooterTable {
    // TODO: We may want P values here, or just tune kF
    public static final class ShooterTableEntry {
        public Measure<Distance> distance;
        public double angle;
        public double percent;

        /**
         * Construct a shooter table entry.
         * 
         * @param distance   Distance from the target, in feet (?)
         * @param angle      Angle of the shooter pivot, in rotations.
         * @param leftSpeed  Speed of the left shooter motor, in RPM.
         * @param rightSpeed Speed of the right shooter motor, in RPM.
         */
        public ShooterTableEntry(Measure<Distance> distance, double angle, double percent) {
            this.angle = angle;
            this.percent = percent;
            this.distance = distance;
        }
    }

    private static ArrayList<ShooterTableEntry> table = new ArrayList<>();

    private static void fillInTable() {
        // put entries here
        // Distances must go from top to bottom: shortest to longest
        // table.add(new ShooterTableEntry(2, 3, 4));
    }

    static {
        fillInTable();
    }

    public static ShooterTableEntry calcShooterTableEntry(Measure<Distance> distance) {
        ShooterTableEntry closestLower = table.get(0);
        ShooterTableEntry closestHigher = table.get(table.size() - 1);

        if (distance.lt(closestLower.distance))
            return closestLower;
        if (distance.gt(closestHigher.distance))
            return closestHigher;

        // loop thru all of the entrys of the shootertable
        for (ShooterTableEntry entry : table) {
            if (entry.distance.lt(distance)
                    && (Math.abs(distance.baseUnitMagnitude() - closestLower.distance.baseUnitMagnitude()) > Math
                            .abs(distance.baseUnitMagnitude() - entry.distance.baseUnitMagnitude()))) {
                closestLower = entry;
            } else if (entry.distance.gt(distance)
                    && (Math.abs(closestHigher.distance.baseUnitMagnitude() - distance.baseUnitMagnitude()) > Math
                            .abs(entry.distance.baseUnitMagnitude() - distance.baseUnitMagnitude()))) {
                closestHigher = entry;
            } else if (entry.distance.isEquivalent(distance)) {
                closestHigher = entry;
                closestLower = entry;
                break;
            }
        }

        
        double scaleFactor = (distance.magnitude() - closestLower.distance.magnitude())
                / (closestHigher.distance.magnitude() - closestLower.distance.magnitude());

        double calculatedPercent = scaleFactor * (closestHigher.percent - closestLower.percent)
                + closestLower.percent;

        double calculatedAngle = scaleFactor * (closestHigher.angle - closestLower.angle) + closestLower.angle;

        return new ShooterTableEntry(distance, calculatedAngle, calculatedPercent);
    }

}
