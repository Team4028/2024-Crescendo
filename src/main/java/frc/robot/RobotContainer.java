// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Conveyor;
// import frc.robot.subsystems.Fan;
import frc.robot.subsystems.Infeed;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;
import frc.robot.utils.ShooterTable;
import frc.robot.utils.ShooterTable.ShooterTableEntry;

public class RobotContainer {
    // =============================================== //
    /* Magic numbers, Vbus constants, and OI constants */
    // =============================================== //
    private static final double CLIMBER_VBUS = 0.7;
    private static final double INFEED_VBUS = 0.8;
    private static final double SLOW_INFEED_VBUS = 0.5;
    private static final double VERY_SLOW_INFEED_VBUS = 0.15;

    private static final double PIVOT_VBUS = 0.3;
    private static final double VERY_SLOW_CONVEYOR_VBUS = 0.15;
    private static final double SLOW_CONVEYOR_VBUS = 0.5;
    private static final double FAST_CONVEYOR_VBUS = 0.85;

    private static final double FAN_VBUS = 1.0;
    private static final double SHOOTER_BACKOUT_VBUS = -0.4;

    private static final int OI_DRIVER_CONTROLLER = 0;
    private static final int OI_OPERATOR_CONTROLLER = 1;

    // ======================== //
    /* Controllers & Subsystems */
    // ======================== //
    private final CommandXboxController driverController = new CommandXboxController(OI_DRIVER_CONTROLLER);
    // private final CommandXboxController operaterController = new
    // CommandXboxController(OI_OPERATOR_CONTROLLER);

    private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain;
    private final Infeed infeed = new Infeed();
    private final Shooter shooter = new Shooter();
    private final Conveyor conveyor = new Conveyor();
    private final Climber m_climber = new Climber();
    // private final Fan m_fan = new Fan();

    private final Vision m_rightVision = new Vision("Right_AprilTag_Camera", Vision.RIGHT_ROBOT_TO_CAMERA);
    private final Vision m_leftVision = new Vision("Left_AprilTag_Camera", Vision.LEFT_ROBOT_TO_CAMERA);

    // ====================== //
    /* Auton & Other Commands */
    // ====================== //
    private final Command smartInfeedCommand;
    private SendableChooser<Command> autonChooser;

    // ====================================================== //
    /* Drivetrain Constants, Magic numbers, and Slew Limiters */
    // ====================================================== //
    private final SlewRateLimiter xLimiter = new SlewRateLimiter(4.);
    private final SlewRateLimiter yLimiter = new SlewRateLimiter(4.);
    private final SlewRateLimiter thetaLimiter = new SlewRateLimiter(4.);
    private final SlewRateLimiter xLimeAquireLimiter = new SlewRateLimiter(4.);

    private static final double MAX_SPEED = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
    private static final double MAX_ANGULAR_SPEED = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity
    private static final double BASE_SPEED = 0.25;

    // ======================== //
    /* Swerve Control & Logging */
    // ======================== //
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MAX_SPEED * 0.1).withRotationalDeadband(MAX_ANGULAR_SPEED * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                                     // driving in open loop
    private final Telemetry logger = new Telemetry(MAX_SPEED);

    // ====================== //
    /* Auton & Named Commands */
    // ====================== //
    private void initAutonChooser() {
        autonChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autonChooser);
    }

    private void initNamedCommands() {
        NamedCommands.registerCommand("startShooter",
                shooter.setSlotCommand(1).andThen(shooter.setPivotPositionCommand())
                        .andThen(shooter.runVelocityCommand()));
        NamedCommands.registerCommand("4pinfeed", infeed.runInfeedMotorCommand(INFEED_VBUS)
                .alongWith(conveyor.runMotorCommand(FAST_CONVEYOR_VBUS)).repeatedly());// .withTimeout(1.5));
        NamedCommands.registerCommand("farShot", Commands.runOnce(() -> shooter.runPivotToPosition(14.25)));
    }

    // TODO: this stuff needs cleaned up
    // Likely time for a operator controller,
    // or at least getting rid of useless stuff

    // =========================== //
    /* Bindings & Default Commands */
    // =========================== //
    private void configureBindings() {
        drivetrain.setDefaultCommand(
                drivetrain.applyRequest(() -> drive
                        .withVelocityX(scaleDriverController(-driverController.getLeftY(),
                                xLimiter,
                                BASE_SPEED) * MAX_SPEED)
                        .withVelocityY(scaleDriverController(-driverController.getLeftX(),
                                yLimiter,
                                BASE_SPEED) * MAX_SPEED)
                        .withRotationalRate(
                                scaleDriverController(-driverController.getRightX(),
                                        thetaLimiter, BASE_SPEED) *
                                        MAX_SPEED)));

        conveyor.setDefaultCommand(conveyor.runMotorCommand(0.));
        infeed.setDefaultCommand(infeed.runInfeedMotorCommand(0.));

        // ========================= //
        /* Infeed & Conveyor Control */
        // ========================= //

        /* Dumb Infeed */
        driverController.leftTrigger().onTrue(
                infeed.runInfeedMotorCommand(INFEED_VBUS).alongWith(
                        conveyor.runMotorCommand(SLOW_CONVEYOR_VBUS)).repeatedly())
                .onFalse(infeed.runInfeedMotorCommand(0.).alongWith(
                        conveyor.runMotorCommand(0.)));

        /* Conveyance */
        driverController.a()
                .onTrue(conveyor.runXRotations(10).alongWith(infeed.runInfeedMotorCommand(SLOW_INFEED_VBUS)))
                .onFalse(infeed.runInfeedMotorCommand(SLOW_INFEED_VBUS));

        /* Smart Infeed */
        driverController.b().toggleOnTrue(smartInfeedCommand);

        // ======================= //
        /* Shooter & Pivot Control */
        // ======================= //

        /* Run Shooter */
        driverController.x().and(driverController.povCenter())
                .toggleOnTrue(shooter.runVelocityCommand());

        /* Cycle Down (Trap -> Long) */
        driverController.x().and(driverController.povDown())
                .onTrue(shooter.cycleDownCommand());

        /* Cycle Up (Long -> Trap) */
        driverController.x().and(driverController.povUp())
                .onTrue(shooter.cycleUpCommand());

        /* Run Pivot */
        // driverController.x().and(driverController.povLeft().or(driverController.povRight()))
        // .onTrue(shooter.runPivotPositionCommand(shooter.getPivotPosition()));
        driverController.x().and(driverController.povRight())
                .toggleOnTrue(shooter.runEntryCommand(() -> printSTVals()));

        // driverController.y().and(driverController.povCenter()).onTrue(shooter.pivotZeroCommand());
        // driverController.y().toggleOnTrue(m_fan.runMotorCommand(FAN_VBUS));

        // ========================= //
        /* Climber & Zeroing Control */
        // ========================= //

        /* Zero Climber & Pivot */
        driverController.y().and(driverController.povCenter()).onTrue(zeroCommand());

        /* Run Climber to "Home" */
        driverController.y().and(driverController.povDown())
                .onTrue(m_climber.runToPositionCommand(Climber.ClimberPositions.HOME));

        /* Run Climber to "Catch" */
        driverController.y().and(driverController.povLeft().or(driverController.povRight()))
                .onTrue(m_climber.runToPositionCommand(Climber.ClimberPositions.CATCH));

        /* Run Climber to "Ready" */
        driverController.y().and(driverController.povUp())
                .onTrue(m_climber.runToPositionCommand(Climber.ClimberPositions.READY));

        // ========================== //
        /* Drivetain & Vision Control */
        // ========================== //

        /* Reset Field-Centric Heading */
        driverController.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative(new Pose2d())));

        /* Add Vision Measurement */
        driverController.back().and(driverController.povCenter())
                .onTrue(drivetrain.addMeasurementCommand(() -> getBestPose()));

        /* Reset Pose & Test ShooterTable */
        driverController.back().and(driverController.povDown()).onTrue(Commands.runOnce(() -> {
            var pose = getBestPose();
            if (pose.isPresent())
                drivetrain.seedFieldRelative(pose.get().estimatedPose.toPose2d());

            printSTVals();
        }));

        driverController.rightBumper().onTrue(shooter.runPivotCommand(PIVOT_VBUS))
                .onFalse(shooter.runPivotCommand(0.0));
        driverController.leftBumper().onTrue(shooter.runPivotCommand(-PIVOT_VBUS))
                .onFalse(shooter.runPivotCommand(0.0));

        // driverController.povRight()
        // .onTrue(shooter.run(() ->
        // shooter.runPivotToPosition(shooter.getPivotPosition())));

        if (Utils.isSimulation()) {
            drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
        }

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public RobotContainer() {
        // better solution:
        // run fast until infeed tripped
        // run slow until conveyor tripped
        // run for like 0.5 more seconds and stop
        smartInfeedCommand = infeed.runInfeedMotorCommand(INFEED_VBUS)
        .alongWith(conveyor.runMotorCommand(SLOW_CONVEYOR_VBUS))
        .repeatedly().until(conveyor.hasInfedSupplier())
        .andThen(infeed.runInfeedMotorCommand(0.).alongWith(conveyor.runMotorCommand(0.))
        .repeatedly().withTimeout(0.1))
        .andThen(shooter.spinMotorRightCommand(SHOOTER_BACKOUT_VBUS).repeatedly()
        .raceWith(conveyor.runXRotations(-1.).withTimeout(0.5)
        .alongWith(infeed.runInfeedMotorCommand(0.))))
        .andThen(shooter.spinMotorRightCommand(0.).raceWith(new WaitCommand(0.1)));

        // smartInfeedCommand = runBoth(SLOW_CONVEYOR_VBUS, INFEED_VBUS)
        //         .repeatedly().until(infeed.hasGamePieceSupplier())
        //         .andThen(runBoth(SLOW_CONVEYOR_VBUS, SLOW_INFEED_VBUS).repeatedly())
        //         .until(conveyor.hasGamePieceSupplier())
        //         .andThen(runBoth(VERY_SLOW_CONVEYOR_VBUS, SLOW_INFEED_VBUS).repeatedly().withTimeout(0.35))
        //         .andThen(shooter.spinMotorRightCommand(-0.4).alongWith(runBoth(-VERY_SLOW_CONVEYOR_VBUS, -VERY_SLOW_INFEED_VBUS)).repeatedly());
        // .alongWith(shooter.spinMotorRightCommand(SHOOTER_BACKOUT_VBUS)).repeatedly().withTimeout(0.5))
        // .andThen(shooter.spinMotorRightCommand(0.));
        // .until(() -> !conveyor.hasGamePiece()))
        // .andThen(conveyor.runXRotations(0.).alongWith(infeed.runInfeedMotorCommand(0.)));

        initNamedCommands();
        initAutonChooser();
        configureBindings();
    }

    // =========================================== //
    /* Additional Commands, Getters, and Utilities */
    // =========================================== //

    /* Run both Conveyor and Infeed */
    private Command runBoth(double conveyorVbus, double infeedVbus) {
        return infeed.runInfeedMotorCommand(conveyorVbus).alongWith(conveyor.runMotorCommand(conveyorVbus));
    }

    /* Auton Command */
    public Command getAutonomousCommand() {
        return new InstantCommand(() -> drivetrain.seedFieldRelative(new Pose2d()))
                .alongWith(shooter.pivotZeroCommand()).andThen(autonChooser.getSelected());
    }

    /* Zeroing Command */
    public Command zeroCommand() {
        return m_climber.zeroCommand().andThen(shooter.pivotZeroCommand());
    }

    /* Asynchronous Zero */
    public void zero() {
        zeroCommand().schedule();
    }

    /* Joystick Scaling */
    private double scaleDriverController(double controllerInput, SlewRateLimiter limiter, double baseSpeedPercent) {
        return limiter.calculate(
                controllerInput * (baseSpeedPercent
                        + driverController.getRightTriggerAxis() * (1 - baseSpeedPercent)));
    }

    // ======= //
    /* Logging */
    // ======= //
    public void logValues() {
        drivetrain.logValues();
        conveyor.logValues();
        infeed.logValues();
        shooter.logValues();
        m_climber.logValues();
        // m_fan.logValues();
    }

    // ================ //
    /* Vision Utilities */
    // ================ //

    /* Return Approx. 2d yaw */
    private double getBestYaw(int tagID) {
        Optional<Double> leftYaw = m_leftVision.getTagYaw(tagID);
        Optional<Double> rightYaw = m_rightVision.getTagYaw(tagID);

        int numYaws = 0;
        numYaws += leftYaw.isPresent() ? 1 : 0;
        numYaws += rightYaw.isPresent() ? 1 : 0;

        switch (numYaws) {
            case 1:
                return (leftYaw.isPresent() ? leftYaw : rightYaw).get();
            case 2:
                return (leftYaw.get() + rightYaw.get()) / 2.;
            case 0:
            default:
                return 0.;
        }
    }

    /* Return approx. 2d distance */
    private double getBestDistance(int tagID) {
        Optional<Double> leftDistance = m_leftVision.getTagDistance(tagID);
        Optional<Double> rightDistance = m_rightVision.getTagDistance(tagID);

        int numDistances = 0;
        numDistances += leftDistance.isPresent() ? 1 : 0;
        numDistances += rightDistance.isPresent() ? 1 : 0;

        switch (numDistances) {
            case 1:
                return (leftDistance.isPresent() ? leftDistance : rightDistance).get();
            case 2:
                return (leftDistance.get() + rightDistance.get()) / 2.;
            case 0:
            default:
                return 0.;
        }
    }

    /* Return approx. 3d pose */
    private Optional<EstimatedRobotPose> getBestPose() {
        Pose2d drivetrainPose = drivetrain.getState().Pose;

        Optional<EstimatedRobotPose> front = m_rightVision.getCameraResult(drivetrainPose);
        Optional<EstimatedRobotPose> back = m_leftVision.getCameraResult(drivetrainPose);

        int numPoses = 0;

        numPoses += front.isPresent() ? 1 : 0;
        numPoses += back.isPresent() ? 1 : 0;

        Optional<Pose2d> pose = Optional.empty();
        SmartDashboard.putNumber("nuMPoses", numPoses);

        if (numPoses == 1) {
            pose = Optional
                    .of(new Pose2d((front.isEmpty() ? back : front).get().estimatedPose.toPose2d().getTranslation(),
                            drivetrainPose.getRotation()));
        } else if (numPoses == 2) {
            // average the poses
            Pose3d frontP = front.get().estimatedPose;
            Pose3d backP = back.get().estimatedPose;

            Translation3d frontT = frontP.getTranslation();
            Translation3d backT = backP.getTranslation();

            Pose3d dPose = new Pose3d(drivetrainPose);
            Rotation3d bruh = (dPose.getRotation());

            pose = Optional.of(
                    new Pose3d(frontT.plus(backT),
                            bruh.times(2.)).toPose2d().div(2.));
        }

        if (pose.isPresent()) {
            return Optional.of(new EstimatedRobotPose(
                    new Pose3d(pose.get()),
                    (front.isEmpty() ? back : front).get().timestampSeconds,
                    null, null));
        }

        return Optional.empty();
    }

    /* Test Shooter Table */
    private ShooterTableEntry printSTVals() {
        Pose2d pose = drivetrain.getState().Pose;

        Transform2d dist = pose.minus(Constants.SPEAKER_DISTANCE_TARGET);
        Translation2d translation = dist.getTranslation().div(12.).div(0.0254); // get everything in feet

        ShooterTableEntry entryPicked = ShooterTable.calcShooterTableEntry(translation.getNorm());

        SmartDashboard.putNumber("Distance", translation.getNorm());

        SmartDashboard.putNumber("ST Angle", entryPicked.Angle);
        SmartDashboard.putNumber("ST Left", entryPicked.LeftSpeed);
        SmartDashboard.putNumber("ST Right", entryPicked.RightSpeed);

        return entryPicked;
    }
}
