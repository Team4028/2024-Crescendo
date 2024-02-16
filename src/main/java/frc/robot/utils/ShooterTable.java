package frc.robot.utils;

import java.util.ArrayList;

public class ShooterTable {
    public static final class ShooterTableEntry {
        public double angle;
        public double leftSpeed;
        public double rightSpeed;
        public double distance;

        public ShooterTableEntry(double angle, double leftSpeed, double distance, double rightSpeed) {
            this.angle = angle;
            this.leftSpeed = leftSpeed;
            this.rightSpeed = rightSpeed;
            this.distance = distance;

        }

    }

    private static ArrayList<ShooterTableEntry> table = new ArrayList<>();

    private static void fillInTable() {
        // put entries here
        // Distances must go from top to bottom: shortest to longest
        table.add(new ShooterTableEntry(0, 6, 0, 3));
        table.add(new ShooterTableEntry(0, 2, 1, 0));
        table.add(new ShooterTableEntry(0, 5, 4, 2));
        table.add(new ShooterTableEntry(0, 7, 5, 3));
        table.add(new ShooterTableEntry(0, 5, 3, 6));
    }

    static {
        fillInTable();
    }

    public static ShooterTableEntry calcShooterTableEntry(double distance) {
        ShooterTableEntry closestLower = table.get(0);
        ShooterTableEntry closestHigher = table.get(table.size() - 1);

        // loop thru all of the entrys of the shootertable
        for (ShooterTableEntry entry : table) {
            if (entry.distance < distance
                    && (Math.abs(distance - closestLower.distance) > Math.abs(distance - entry.distance))) {
                closestLower = entry;
            } else if (entry.distance > distance
                    && (Math.abs(closestHigher.distance - distance) > Math.abs(entry.distance - distance))) {
                closestHigher = entry;
            } else if (entry.distance == distance) {
                closestHigher = entry;
                closestLower = entry;
                break;
            }
        }

        double scaleFactor = (distance - closestLower.distance) / (closestHigher.distance - closestLower.distance);

        double calculatedLeftSpeed = scaleFactor * (closestHigher.leftSpeed - closestLower.leftSpeed)
                + closestLower.leftSpeed;
        double calculatedRightSpeed = scaleFactor * (closestHigher.rightSpeed - closestLower.rightSpeed)
                + closestLower.rightSpeed;

        double calculatedAngle = scaleFactor * (closestHigher.angle - closestLower.angle) + closestLower.angle;

        // System.out.println("Lower: " + closestLower.distance);
        // System.out.println("Higher: " + closestHigher.distance);
        // System.out.println(scaleFactor);

        return new ShooterTableEntry(calculatedAngle, calculatedLeftSpeed, calculatedRightSpeed, distance);

    }

}
