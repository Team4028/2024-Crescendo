package frc.robot.utils;

import java.util.ArrayList;

public class ShooterTable {
    public static final class ShooterTableEntry {
        public double Distance;
        public double angle;
        public double LeftSpeed;
        public double RightSpeed;

        /**
         * Construct a shooter table entry.
         * 
         * @param distance   Distance from the target, in feet (?)
         * @param angle      Angle of the shooter pivot, in rotations.
         * @param leftSpeed  Speed of the left shooter motor, in RPM.
         * @param rightSpeed Speed of the right shooter motor, in RPM.
         */
        public ShooterTableEntry(double distance, double angle, double leftSpeed, double rightSpeed) {
            this.angle = angle;
            this.LeftSpeed = leftSpeed;
            this.RightSpeed = rightSpeed;
            this.Distance = distance;
        }
    }

    private static ArrayList<ShooterTableEntry> table = new ArrayList<>();

    private static void fillInTable() {
        // put entries here
        // Distances must go from top to bottom: shortest to longest
        table.add(new ShooterTableEntry(4., 40., 1500., 1500.));
        table.add(new ShooterTableEntry(9., 15., 2000., 2820.));
        table.add(new ShooterTableEntry(19., 2, 2500, 3400.));
    }

    static {
        fillInTable();
    }

    public static ShooterTableEntry calcShooterTableEntry(double distance) {
        ShooterTableEntry closestLower = table.get(0);
        ShooterTableEntry closestHigher = table.get(table.size() - 1);

        // loop thru all of the entrys of the shootertable
        for (ShooterTableEntry entry : table) {
            if (entry.Distance < distance
                    && (Math.abs(distance - closestLower.Distance) > Math.abs(distance - entry.Distance))) {
                closestLower = entry;
            } else if (entry.Distance > distance
                    && (Math.abs(closestHigher.Distance - distance) > Math.abs(entry.Distance - distance))) {
                closestHigher = entry;
            } else if (entry.Distance == distance) {
                closestHigher = entry;
                closestLower = entry;
                break;
            }
        }

        double scaleFactor = (distance - closestLower.Distance) / (closestHigher.Distance - closestLower.Distance);

        double calculatedLeftSpeed = scaleFactor * (closestHigher.LeftSpeed - closestLower.LeftSpeed)
                + closestLower.LeftSpeed;
        double calculatedRightSpeed = scaleFactor * (closestHigher.RightSpeed - closestLower.RightSpeed)
                + closestLower.RightSpeed;

        double calculatedAngle = scaleFactor * (closestHigher.angle - closestLower.angle) + closestLower.angle;

        return new ShooterTableEntry(calculatedAngle, calculatedLeftSpeed, calculatedRightSpeed, distance);
    }

}
