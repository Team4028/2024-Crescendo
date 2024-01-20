package frc.robot.subsystems;

import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

public class PhotonPrototype extends SubsystemBase {
    static PhotonPrototype instance;

    PhotonCamera PhotonFront = new PhotonCamera("Front_AprilTag_Camera");
    PhotonCamera PhotonBack = new PhotonCamera("Back_AprilTag_Camera");

    private final Pose3d backCameraToRobot = new Pose3d(Units.inchesToMeters(-5.875),
            Units.inchesToMeters(5.625), 0.,
            new Rotation3d(0., Units.degreesToRadians(0.), Units.degreesToRadians(0.)));

    private static final Pose3d frontCameraToRobot = new Pose3d(Units.inchesToMeters(-3.3125),
            Units.inchesToMeters(5.5625), 0.,
            new Rotation3d(0., Units.degreesToRadians(0.),
                    Units.degreesToRadians(180.)));

    private PhotonPrototype() {
    }

    private void getCameraResultFront() {
        var resultFront = PhotonFront.getLatestResult();
        boolean checkHasResults = resultFront.hasTargets();
        List<PhotonTrackedTarget> frontTargets = resultFront.getTargets();
        PhotonTrackedTarget frontTarget = resultFront.getBestTarget();

        if (!checkHasResults) {
            return;
        }

        double frontYaw = frontTarget.getYaw();
        double frontPitch = frontTarget.getPitch();
        double frontArea = frontTarget.getArea();
        double frontSkew = frontTarget.getSkew();

        Transform3d threeDt = frontTarget.getBestCameraToTarget();
        Transform2d pose = new Transform2d(new Translation2d(threeDt.getX(), threeDt.getY()), new Rotation2d());
        List<TargetCorner> corners = frontTarget.getDetectedCorners();

        int targetID = frontTarget.getFiducialId();
        double poseAmbiguity = frontTarget.getPoseAmbiguity();
        Transform3d bestCameraToTarget = frontTarget.getBestCameraToTarget();
        Transform3d alternateCameraToTarget = frontTarget.getAlternateCameraToTarget();

        SmartDashboard.putNumber("Front X distance", pose.getX());
        SmartDashboard.putNumber("Front Y distance", pose.getY());
        SmartDashboard.putNumber("Front yaw", frontYaw);
        SmartDashboard.putNumber("Front pitch", frontPitch);
        SmartDashboard.putNumber("Front area", frontArea);
        SmartDashboard.putNumber("Front skew", frontSkew);

        
        // Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(frontTarget.getBestCameraToTarget(),
        //         AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile)
        //                 .getTagPose(frontTarget.getFiducialId().get()),
        //         frontCameraToRobot).get();
    }

    private void getCameraResultBack() {
        var resultBack = PhotonBack.getLatestResult();
        boolean checkHasResults = resultBack.hasTargets();
        List<PhotonTrackedTarget> backTargets = resultBack.getTargets();
        PhotonTrackedTarget backTarget = resultBack.getBestTarget();

        if (!checkHasResults) {
            return;
        }

        double backYaw = backTarget.getYaw();
        double backPitch = backTarget.getPitch();
        double backArea = backTarget.getArea();
        double backSkew = backTarget.getSkew();

        Transform3d threeDt = backTarget.getBestCameraToTarget();
        Transform2d pose = new Transform2d(new Translation2d(threeDt.getX(), threeDt.getY()), new Rotation2d());
        List<TargetCorner> corners = backTarget.getDetectedCorners();

        int targetID = backTarget.getFiducialId();
        double poseAmbiguity = backTarget.getPoseAmbiguity();
        Transform3d bestCameraToTarget = backTarget.getBestCameraToTarget();
        Transform3d alternateCameraToTarget = backTarget.getAlternateCameraToTarget();

        SmartDashboard.putNumber("Back X distance", pose.getX());
        SmartDashboard.putNumber("Back Y distance", pose.getY());
        SmartDashboard.putNumber("Back yaw", backYaw);
        SmartDashboard.putNumber("Back pitch", backPitch);
        SmartDashboard.putNumber("Back area", backArea);
        SmartDashboard.putNumber("Back skew", backSkew);

        // Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(backTarget.getBestCameraToTarget(),
        //         AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile)
        //                 .getTagPose(backTarget.getFiducialId().get()),
        //         backCameraToRobot);
    }

    public static PhotonPrototype getInstance() {
        if (instance == null) {
            instance = new PhotonPrototype();
        }

        return instance;
    }

    @Override
    public void periodic() {
        getCameraResultFront();
        getCameraResultBack();

    }

}
