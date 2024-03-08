package frc.robot.utils;

import static edu.wpi.first.units.Units.Feet;

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
        table.add(new ShooterTableEntry(Feet.of(3.1), 36.0, .95));
        table.add(new ShooterTableEntry(Feet.of(6.0), 15.4, .95));
        table.add(new ShooterTableEntry(Feet.of(9.0), 14.2, .95));
        table.add(new ShooterTableEntry(Feet.of(12.), 9.2, .95));
        table.add(new ShooterTableEntry(Feet.of(15.), 7.0, 1.));
        table.add(new ShooterTableEntry(Feet.of(20.), 4.8, 1.));
    }

    static {
        fillInTable();
    }

    public static ShooterTableEntry calcShooterTableEntry(Measure<Distance> distance) {
        ShooterTableEntry closestLower = table.get(0);
        ShooterTableEntry closestHigher = table.get(table.size() - 1);

        if (distance.lte(closestLower.distance))
            return closestLower;
        if (distance.gte(closestHigher.distance))
            return closestHigher;

        // loop thru all of the entrys of the shootertable
        for (ShooterTableEntry entry : table) {
            if (entry.distance.lt(distance)
                    && (Math.abs(distance.minus(closestLower.distance).baseUnitMagnitude()) > Math
                            .abs(distance.minus(entry.distance).baseUnitMagnitude()))) {
                closestLower = entry;
            } else if (entry.distance.gt(distance)
                    && (Math.abs(closestHigher.distance.minus(distance).baseUnitMagnitude()) > Math
                            .abs(entry.distance.minus(distance).baseUnitMagnitude()))) {
                closestHigher = entry;
            } else if (entry.distance.isEquivalent(distance)) {
                return entry;
            }
        }

        double scaleFactor = (distance.minus(closestLower.distance).baseUnitMagnitude())
                / (closestHigher.distance.minus(closestLower.distance).baseUnitMagnitude());

        double calculatedPercent = scaleFactor * (closestHigher.percent - closestLower.percent)
                + closestLower.percent;

        double calculatedAngle = scaleFactor * (closestHigher.angle - closestLower.angle) + closestLower.angle;

        return new ShooterTableEntry(distance, calculatedAngle, calculatedPercent);
    }

}
