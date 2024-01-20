package frc.robot.utils;

import java.util.ArrayList;

public class ShooterTable {
    public static final class ShooterTableEntry {
        public double angle;
        public double speed;
        public double distance;

        public ShooterTableEntry(double angle, double speed, double distance) {
            this.angle = angle;
            this.speed = speed;
            this.distance = distance;
        }
    }

    ArrayList<ShooterTableEntry> table = new ArrayList<>();

    private void fillInTable() {
        // put entries here
    }

    public ShooterTable() {
        fillInTable();
    }

    public ShooterTableEntry calcShooterTableEntry(double distance) {
        ShooterTableEntry closestLower = table.get(0);
        ShooterTableEntry closestHigher = table.get(table.size() - 1);

        // loop thru all of the entrys of the shootertable
        for (ShooterTableEntry entry : table) {
            if (entry.distance < distance && (distance - closestLower.distance > distance - entry.distance)) {
                closestLower = entry;
            } else if (entry.distance > distance && (closestHigher.distance - distance > entry.distance - distance)) {
                closestHigher = entry;
            } else if (entry.distance == distance) {
                closestHigher = entry;
                closestLower = entry;
                break;
            }
        }

        double scaleFactor = (distance - closestLower.distance) / (closestHigher.distance - closestLower.distance);

        double calculatedSpeed = scaleFactor * (closestHigher.speed - closestLower.speed);

        double calculatedAngle = scaleFactor * (closestHigher.angle - closestLower.angle);

        return new ShooterTableEntry(calculatedAngle, calculatedSpeed, distance);

    }
}
