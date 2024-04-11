package frc.robot.utils;

import static edu.wpi.first.units.Units.Feet;

import java.util.TreeMap;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import frc.robot.utils.ShooterTable.VisionTableEntry.CameraLerpStrat;

public class EPICShooterTable {

    /// @formatter:off
    static { populateShooterTable(); }
    /// @formatter:on

    public static final class VisionStruct {
        public double PhotonDistance;
        public double PhotonStationaryDistance;
        public double LLTY;
        public double LLTA;
        public double LLTAMulti;

        @FunctionalInterface
        private static interface VisionInverseInterpolator {
            double inverseInterpolate(VisionStruct startValue, VisionStruct endValue, VisionStruct q,
                    CameraLerpStrat strategy);
        }

        private static final class VisionInterpolatingTreeMap {
            CameraLerpStrat strategy = CameraLerpStrat.LimelightTY;
            private final TreeMap<VisionStruct, Measure<Distance>> map;
            private final VisionInverseInterpolator inverseInterpolator;
            private final Interpolator<Measure<Distance>> interpolator;

            public VisionInterpolatingTreeMap(VisionInverseInterpolator inverseInterpolator,
                    Interpolator<Measure<Distance>> interpolator) {

                map = new TreeMap<VisionStruct, Measure<Distance>>((vte1, vte2) -> {
                    switch (((Supplier<CameraLerpStrat>) this::getStrategy).get()) {
                        case PhotonVisionStationaryDistance:
                            return Double.compare(vte1.PhotonStationaryDistance, vte2.PhotonStationaryDistance);
                        case LimelightTY:
                            return Double.compare(vte1.LLTY, vte2.LLTY);
                        case LimelightArea:
                            return Double.compare(vte1.LLTA, vte2.LLTA);
                        case LimelightMultiTagArea:
                            return Double.compare(vte1.LLTAMulti, vte2.LLTAMulti);
                        default:
                            return Double.compare(vte1.PhotonDistance, vte2.PhotonDistance);
                    }
                });
                this.inverseInterpolator = inverseInterpolator;
                this.interpolator = interpolator;
            }

            public CameraLerpStrat getStrategy() {
                return strategy;
            }

            public void clear() {
                map.clear();
            }

            public void put(VisionStruct key, Measure<Distance> value) {
                map.put(key, value);
            }

            public Measure<Distance> get(VisionStruct cameraMeasure, CameraLerpStrat strategy) {
                this.strategy = strategy;
                Measure<Distance> value = map.get(cameraMeasure);
                if (value != null)
                    return value;

                VisionStruct ceilKey = map.ceilingKey(cameraMeasure);
                VisionStruct floorKey = map.floorKey(cameraMeasure);
                if (ceilKey == null && floorKey == null)
                    return null;
                if (ceilKey == null)
                    return map.get(floorKey);
                if (floorKey == null)
                    return map.get(ceilKey);

                var floor = map.get(floorKey);
                var ceil = map.get(ceilKey);

                return interpolator.interpolate(floor, ceil,
                        inverseInterpolator.inverseInterpolate(floorKey, ceilKey, cameraMeasure, strategy));
            }
        }

        public VisionStruct(double photonDistance, double photonStationaryDistance, double llTY, double llTA,
                double llTAMulti) {
            PhotonDistance = photonDistance;
            PhotonStationaryDistance = photonStationaryDistance;
            LLTY = llTY;
            LLTA = llTA;
            LLTAMulti = llTAMulti;
        }

        private static final double inverseInterpolate(VisionStruct startValue, VisionStruct endValue, VisionStruct q,
                CameraLerpStrat strategy) {
            switch (strategy) {
                case PhotonVisionStationaryDistance:
                    return MathUtil.inverseInterpolate(startValue.PhotonStationaryDistance,
                            endValue.PhotonStationaryDistance, q.PhotonStationaryDistance);
                case LimelightTY:
                    return MathUtil.inverseInterpolate(startValue.LLTY, endValue.LLTY, q.LLTY);
                case LimelightArea:
                    return MathUtil.inverseInterpolate(startValue.LLTA, endValue.LLTA, q.LLTA);
                case LimelightMultiTagArea:
                    return MathUtil.inverseInterpolate(startValue.LLTAMulti, endValue.LLTAMulti, q.LLTAMulti);
                default:
                    return MathUtil.inverseInterpolate(startValue.PhotonDistance, endValue.PhotonDistance,
                            q.PhotonDistance);
            }
        }
    }

    public static final class ShooterStruct {
        public double Angle;
        public double Percent;

        public ShooterStruct(double angle, double percent) {
            Angle = angle;
            Percent = percent;
        }

        private static final ShooterStruct interpolate(ShooterStruct startValue, ShooterStruct endValue, double t) {
            return new ShooterStruct(MathUtil.interpolate(startValue.Angle, endValue.Angle, t),
                    MathUtil.interpolate(startValue.Percent, endValue.Percent, t));
        }
    }

    private static final class DistanceMeasureInterpolation {
        private static final Measure<Distance> interpolate(Measure<Distance> startValue,
                Measure<Distance> endValue,
                double t) {
            return endValue.minus(startValue).times(MathUtil.clamp(t, 0, 1)).plus(startValue);
        }

        private static final double inverseInterpolate(Measure<Distance> startValue, Measure<Distance> endValue,
                Measure<Distance> q) {
            return MathUtil.inverseInterpolate(startValue.in(Feet), endValue.in(Feet), q.in(Feet));
        }
    }

    public static final VisionStruct.VisionInterpolatingTreeMap visionMap = new VisionStruct.VisionInterpolatingTreeMap(
            VisionStruct::inverseInterpolate, DistanceMeasureInterpolation::interpolate);

    public static final InterpolatingTreeMap<Measure<Distance>, ShooterStruct> shooterMap = new InterpolatingTreeMap<>(
            DistanceMeasureInterpolation::inverseInterpolate, ShooterStruct::interpolate);

    public static final void populateShooterTable() {
        // vision map
        visionMap.put(new VisionStruct(4.34, 0, 21.588, 0.97, 4.5), Feet.of(4.2));
        visionMap.put(new VisionStruct(5.09, 0, 15.065, 0.785, 3.757), Feet.of(5));
        visionMap.put(new VisionStruct(6.094, 0, 7.79, 0.598, 2.796), Feet.of(6));
        visionMap.put(new VisionStruct(7.799, 0, -1.295, 0.368, 1.761), Feet.of(8));
        visionMap.put(new VisionStruct(9.17, 0, -6.465, 0.257, 1.195), Feet.of(10));
        visionMap.put(new VisionStruct(11.34, 0, -12.115, 0.166, .78), Feet.of(13));
        visionMap.put(new VisionStruct(13.115, 0, -15.395, 0.114, .528), Feet.of(16));
        visionMap.put(new VisionStruct(13.845, 0, -16.53, 0.082, .38), Feet.of(19));
        visionMap.put(new VisionStruct(15.095, 0, -18.202, 0.064, .29), Feet.of(22));
        visionMap.put(new VisionStruct(17.24, 0, -20.43, 0.045, .25), Feet.of(27));

        // shooter map
        shooterMap.put(Feet.of(4.2), new ShooterStruct(30.9, 0.6));
        shooterMap.put(Feet.of(5), new ShooterStruct(27.0, 0.7));
        shooterMap.put(Feet.of(6), new ShooterStruct(22.8, 0.8));
        shooterMap.put(Feet.of(8), new ShooterStruct(16., 1.0));
        shooterMap.put(Feet.of(10), new ShooterStruct(12.1, 1.0));
        shooterMap.put(Feet.of(13), new ShooterStruct(6.7, 1.0));
        shooterMap.put(Feet.of(16), new ShooterStruct(4.3, 1.0));
        shooterMap.put(Feet.of(19), new ShooterStruct(3.25, 1.0));
        shooterMap.put(Feet.of(22), new ShooterStruct(2.24, 1.0));
        shooterMap.put(Feet.of(27), new ShooterStruct(0.25, 1.0));
    }

    public static ShooterTable.ShooterTableEntry getShooterTableEntryCamera(double cameraMeasure, CameraLerpStrat strategy) {
        VisionStruct struct = new VisionStruct(cameraMeasure, cameraMeasure, cameraMeasure, cameraMeasure,
                cameraMeasure);
        var dist = visionMap.get(struct, strategy);
        var ss = shooterMap.get(dist);
        return new ShooterTable.ShooterTableEntry(dist, ss.Angle, ss.Percent);
    }

}
