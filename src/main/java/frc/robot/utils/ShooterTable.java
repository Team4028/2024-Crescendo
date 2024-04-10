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
        public double PhotonStationaryDistance;
        public double LLTY;
        public double LLTA;
        public double LLTAMulti;
        public double Angle;
        public double Percent;

        public static enum CameraLerpStrat {
            LimelightTY,
            LimeLightTYDistance,
            LimelightArea,
            LimelightMultiTagArea,
            PhotonVisionDistance,
            PhotonVisionStationaryDistance;
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
        public ShooterTableEntry(Measure<Distance> distance, double photonDistance, double photonStationaryDistance,
                double llTY, double llTA,
                double llTAMulti,
                double angle,
                double percent) {
            Angle = angle;
            Percent = percent;
            Distance = distance;
            PhotonDistance = photonDistance;
            PhotonStationaryDistance = photonStationaryDistance;
            LLTY = llTY;
            LLTA = llTA;
            LLTAMulti = llTAMulti;
        }
    }

    private static ArrayList<ShooterTableEntry> table = new ArrayList<>();

    private static void fillInTable() { // TODO: fill in photonStationaryDistance values
        // put entries here
        // Distances must go from top to bottom: shortest to longest
        table.add(new ShooterTableEntry(Feet.of(4.2), 4.34, 0, 21.588, 0.97, 4.5, 30.9, 0.6)); // 55 degrees
        table.add(new ShooterTableEntry(Feet.of(5.0), 5.09, 0, 15.065, 0.785, 3.757, 27.0, 0.7)); // 50 degrees
        table.add(new ShooterTableEntry(Feet.of(6.0), 6.094, 0, 7.79, 0.598, 2.796, 22.8, 0.8)); // 45 degrees
        table.add(new ShooterTableEntry(Feet.of(8.0), 7.799, 0, -1.295, 0.368, 1.761, 16., 1.0)); // 36 degrees
        table.add(new ShooterTableEntry(Feet.of(10.1), 9.17, 0, -6.465, 0.257, 1.195, 12.1, 1.0)); // 31 degrees
        // Was: -12.115 New Red: -11.62
        table.add(new ShooterTableEntry(Feet.of(13.1), 11.34, 0, -12.115, 0.166, 0.78, 6.7, 1.0)); // 25 degrees
        // Was: -15.395 New Blue: -14.93 New Red: -14.535
        table.add(new ShooterTableEntry(Feet.of(16.75), 13.115, 0, -15.395, 0.114, 0.528, 4.3, 1.0)); // 23 degrees
        // Was: -16.53 New Blue: -16.77 New Red: -16.68
        table.add(new ShooterTableEntry(Feet.of(18.6), 13.845, 0, -16.53, 0.082, 0.38, 3.25, 1.0)); // 21.5 degrees
        // Was: -18.2 New Blue: -18.3 New Red: -18.39
        table.add(new ShooterTableEntry(Feet.of(21.7), 15.095, 0, -18.202, 0.064, 0.29, 2.24, 1.0)); // 20.5 degrees
        // Was: -20.43 New Blue: -20.3 New Red: -19.63
        table.add(new ShooterTableEntry(Feet.of(27.3), 17.24, 0, -20.43, 0.045, 0.25, 0.25, 1.0)); // 20 degrees
    }

    static {
        fillInTable();
    }

    public static ShooterTableEntry calcShooterTableEntryByPercent(double percent) {
        Measure<Distance> size = table.get(table.size() - 1).Distance.minus(table.get(0).Distance);
        return calcShooterTableEntry(size.times(percent));
    }

    public static ShooterTableEntry calcShooterTableEntryCamera(double cameraValue, CameraLerpStrat strategy) {
        ShooterTableEntry closestLower = table.get(0);
        ShooterTableEntry closestHigher = table.get(table.size() - 1);

        Function<ShooterTableEntry, Double> cameraValueGetter;

        // SmartDashboard.putNumber("epic ty", cameraValue);

        switch (strategy) {
            case LimelightTY:
                // negate everything because TY is inversely proportional to distance
                cameraValueGetter = (ste) -> -ste.LLTY;
                cameraValue = -cameraValue;
                break;
            case LimeLightTYDistance:
                // redo bc this is lazy and will destroy runtime performance
                cameraValueGetter = (ste) -> ste.LLTA;
                // cameraValueGetter = (ste) -> Units
                // .metersToFeet(RobotContainer.SPEAKER_TAG_HEIGHT -
                // RobotContainer.SHOOTER_CAM_HEIGHT)
                // / Math.tan(RobotContainer.SHOOTER_CAM_PITCH +
                // Units.degreesToRadians(ste.LLTY));
                break;
            case LimelightArea:
                // negate everything because TA is inversely proportional to distance
                cameraValueGetter = (ste) -> -ste.LLTA;
                cameraValue = -cameraValue;
                break;
            case LimelightMultiTagArea:
                cameraValueGetter = (ste) -> -ste.LLTAMulti;
                cameraValue = -cameraValue;
                break;
            case PhotonVisionStationaryDistance:
                cameraValueGetter = (ste) -> ste.PhotonStationaryDistance;
                break;
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

        double scaleFactor = (cameraValue - cameraValueGetter.apply(closestLower))
                / (cameraValueGetter.apply(closestHigher) - cameraValueGetter.apply(closestLower));

        scaleFactor = Math.abs(scaleFactor);

        Measure<Distance> interpolatedDistance = closestHigher.Distance.minus(closestLower.Distance).times(scaleFactor)
                .plus(closestLower.Distance);

        double interpolatedAngle = scaleFactor * (closestHigher.Angle - closestLower.Angle) + closestLower.Angle;

        double interpolatedShooterPercent = scaleFactor * (closestHigher.Percent - closestLower.Percent)
                + closestLower.Percent;

        return new ShooterTableEntry(interpolatedDistance, cameraValue, cameraValue, cameraValue, cameraValue, cameraValue,
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

        return new ShooterTableEntry(distance, calculatedCameraDist, calculatedCameraDist, calculatedCameraDist, calculatedCameraDist,
                calculatedCameraDist,
                calculatedAngle, calculatedPercent);
    }

}
