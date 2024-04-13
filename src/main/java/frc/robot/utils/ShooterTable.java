package frc.robot.utils;

import static edu.wpi.first.units.Units.Feet;

import java.util.ArrayList;
import java.util.function.Function;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.utils.ShooterTable.VisionTableEntry.CameraLerpStrat;

public class ShooterTable {
    public static final class ShooterTableEntry {
        public Measure<Distance> Distance;
        public double Angle;
        public double Percent;

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
        public ShooterTableEntry(Measure<Distance> distance, double angle, double percent) {
            Angle = angle;
            Percent = percent;
            Distance = distance;
        }

        public ShooterTableEntry average(ShooterTableEntry other) {
            return new ShooterTableEntry(Distance.plus(other.Distance).divide(2.), (Angle + other.Angle) / 2,
                    (Percent + other.Percent) / 2);
        }
    }

    public static final class VisionTableEntry {
        public Measure<Distance> Distance;
        public double PhotonDistance;
        public double PhotonStationaryDistance;
        public double LLTY;
        public double LLTA;
        public double LLTAMulti;
        public double LLGLeftTY;
        public double LLGRightTY;

        public static enum CameraLerpStrat {
            LimelightTY,
            LimelightArea,
            LimelightMultiTagArea,
            LeftChasisLimelightTY,
            RightChassisLimelightTY,
            PhotonVisionDistance,
            PhotonVisionStationaryDistance;
        }

        public VisionTableEntry(Measure<Distance> distance, double photonDistance, double photonStationaryDistance,
                double llTY, double llTA, double llTAMulti, double llGLeftTY, double llGRightTY) {
            Distance = distance;
            PhotonDistance = photonDistance;
            PhotonStationaryDistance = photonStationaryDistance;
            LLTY = llTY;
            LLTA = llTA;
            LLTAMulti = llTAMulti;
            LLGLeftTY = llGLeftTY;
            LLGRightTY = llGRightTY;
        }
    }

    private static ArrayList<ShooterTableEntry> shooterTable = new ArrayList<>();
    private static ArrayList<VisionTableEntry> visionTable = new ArrayList<>();

    private static void fillInTable() { // TODO: fill in photonStationaryDistance values
        // put entries here
        // Distances must go from top to bottom: shortest to longest
        // IF YOU INCREASE DISTANCE, SHOOTER ANGLE GOES UP
        shooterTable.add(new ShooterTableEntry(Feet.of(4.2), 30.9, 0.6)); // 55 degrees
        shooterTable.add(new ShooterTableEntry(Feet.of(5), 27.0, 0.7)); // 50 degrees
        shooterTable.add(new ShooterTableEntry(Feet.of(6), 22.8, 0.8)); // 45 degrees
        shooterTable.add(new ShooterTableEntry(Feet.of(8), 16., 1.0)); // 36 degrees
        shooterTable.add(new ShooterTableEntry(Feet.of(10), 12.1, 1.0)); // 31 degrees
        // Was: -12.115 New Red: -11.62
        shooterTable.add(new ShooterTableEntry(Feet.of(13), 6.7, 1.0)); // 25 degrees
        // Was: -15.395 New Blue: -14.93 New Red: -14.535
        shooterTable.add(new ShooterTableEntry(Feet.of(16), 4.3, 1.0)); // 23
                                                                        // degrees
        // Was: -16.53 New Blue: -16.77 New Red: -16.68
        shooterTable.add(new ShooterTableEntry(Feet.of(19), 3.25, 1.0)); // 21.5
                                                                         // degrees
        // Was: -18.2 New Blue: -18.3 New Red: -18.39
        shooterTable.add(new ShooterTableEntry(Feet.of(22), 2.24, 1.0)); // 20.5
                                                                         // degrees
        // Was: -20.43 New Blue: -20.3 New Red: -19.63
        shooterTable.add(new ShooterTableEntry(Feet.of(27), 0.25, 1.0)); // 20 degrees

        // vision table entries
        visionTable.add(new VisionTableEntry(Feet.of(4.2), 4.34, 0, 21.588, 0.97, 4.5, 0, 0));
        visionTable.add(new VisionTableEntry(Feet.of(5), 5.09, 1.565, 15.065, 0.785, 3.757, 0, 0));
        visionTable.add(new VisionTableEntry(Feet.of(6), 6.094, 1.884, 7.79, 0.598, 2.796, 0, 0));
        visionTable.add(new VisionTableEntry(Feet.of(8), 7.799, 2.392, -1.295, 0.368, 1.761, 0, 0));
        visionTable.add(new VisionTableEntry(Feet.of(10), 9.17, 2.821, -6.465, 0.257, 1.195, 0, 0));
        visionTable.add(new VisionTableEntry(Feet.of(13), 11.34, 3.443, -12.115, 0.166, .78, 0, 0));
        visionTable.add(new VisionTableEntry(Feet.of(16), 13.115, 3.954, -15.395, 0.114, .528, 0, 0));
        visionTable.add(new VisionTableEntry(Feet.of(19), 13.845, 4.168, -16.53, 0.082, .38, 0, 0));
        visionTable.add(new VisionTableEntry(Feet.of(22), 15.095, 4.504, -18.202, 0.064, .29, 0, 0));
        visionTable.add(new VisionTableEntry(Feet.of(27), 17.24, 5.111, -20.43, 0.045, .25, 0, 0));
    }

    static {
        fillInTable();
    }

    // public static ShooterTableEntry calcShooterTableEntryByPercent(double
    // percent) {
    // Measure<Distance> size = shooterTable.get(shooterTable.size() -
    // 1).Distance.minus(shooterTable.get(0).Distance);
    // return calcShooterTableEntry(size.times(percent));
    // }

    public static ShooterTableEntry calcShooterTableEntryCamera(double cameraValue, CameraLerpStrat strategy) {
        return calcShooterTableEntry(calcVisionTableEntryCamera(cameraValue, strategy).Distance);
    }

    public static VisionTableEntry calcVisionTableEntryCamera(double cameraValue, CameraLerpStrat strategy) {
        VisionTableEntry closestLower = visionTable.get(0);
        VisionTableEntry closestHigher = visionTable.get(visionTable.size() - 1);

        Function<VisionTableEntry, Double> cameraValueGetter;

        // SmartDashboard.putNumber("epic ty", cameraValue);

        switch (strategy) {
            case LimelightTY:
                // negate everything because TY is inversely proportional to distance
                cameraValueGetter = (vte) -> -vte.LLTY;
                cameraValue = -cameraValue;
                break;
            case LimelightArea:
                // negate everything because TA is inversely proportional to distance
                cameraValueGetter = (vte) -> -vte.LLTA;
                cameraValue = -cameraValue;
                break;
            case LimelightMultiTagArea:
                cameraValueGetter = (vte) -> -vte.LLTAMulti;
                cameraValue = -cameraValue;
                break;
            case PhotonVisionStationaryDistance:
                cameraValueGetter = (vte) -> vte.PhotonStationaryDistance;
                break;
            case LeftChasisLimelightTY:
                cameraValueGetter = (vte) -> -vte.LLGLeftTY;
                cameraValue = -cameraValue;
                break;
            case RightChassisLimelightTY:
                cameraValueGetter = (vte) -> -vte.LLGRightTY;
                cameraValue = -cameraValue;
                break;
            default:
                cameraValueGetter = (vte) -> vte.PhotonDistance;
                break;
        }

        if (cameraValue <= cameraValueGetter.apply(closestLower))
            return closestLower;

        if (cameraValue >= cameraValueGetter.apply(closestHigher))
            return closestHigher;

        for (var entry : visionTable) {
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
                / (cameraValueGetter.apply(closestHigher) -
                        cameraValueGetter.apply(closestLower));

        scaleFactor = Math.abs(scaleFactor);

        Measure<Distance> interpolatedDistance = closestHigher.Distance.minus(closestLower.Distance).times(scaleFactor)
                .plus(closestLower.Distance);

        return new VisionTableEntry(interpolatedDistance, cameraValue, cameraValue, cameraValue, cameraValue,
                cameraValue, cameraValue, cameraValue);
    }

    public static ShooterTableEntry calcShooterTableEntry(Measure<Distance> distance) {
        SmartDashboard.putNumber("Index", distance.in(Feet));
        ShooterTableEntry closestLower = shooterTable.get(0);
        ShooterTableEntry closestHigher = shooterTable.get(shooterTable.size() - 1);

        if (distance.lte(closestLower.Distance))
            return closestLower;
        if (distance.gte(closestHigher.Distance))
            return closestHigher;

        // loop thru all of the entrys of the shootertable
        for (ShooterTableEntry entry : shooterTable) {
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

        double calculatedPercent = scaleFactor * (closestHigher.Percent -
                closestLower.Percent)
                + closestLower.Percent;

        double calculatedAngle = scaleFactor * (closestHigher.Angle -
                closestLower.Angle) + closestLower.Angle;

        return new ShooterTableEntry(distance, calculatedAngle, calculatedPercent);
    }

}
