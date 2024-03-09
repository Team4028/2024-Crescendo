package frc.robot.utils;

import static edu.wpi.first.units.Units.Feet;

import java.util.ArrayList;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;

public class ShooterTable {
    // TODO: We may want P values here, or just tune kF
    public static final class ShooterTableEntry {
        public Measure<Distance> Distance;
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
        public ShooterTableEntry(Measure<Distance> distance, double angle, double percent) {
            this.Angle = angle;
            this.Percent = percent;
            this.Distance = distance;
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

    public static ShooterTableEntry calcShooterTableEntryByPercent(double percent) {
        Measure<Distance> size = table.get(table.size() - 1).Distance.minus(table.get(0).Distance);
        return calcShooterTableEntry(size.times(percent));
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

        double calculatedPercent = scaleFactor * (closestHigher.Percent - closestLower.Percent)
                + closestLower.Percent;

        double calculatedAngle = scaleFactor * (closestHigher.Angle - closestLower.Angle) + closestLower.Angle;

        return new ShooterTableEntry(distance, calculatedAngle, calculatedPercent);
    }

}
