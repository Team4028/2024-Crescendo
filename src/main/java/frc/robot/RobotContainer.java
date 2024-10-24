// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Feet;

import java.util.Map;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Candle;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Fan;
import frc.robot.subsystems.FanPivot;
import frc.robot.subsystems.Infeed;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShotSpeeds;
import frc.robot.subsystems.Whippy;
import frc.robot.utils.BeakUtils;
import frc.robot.utils.DriverCamera;
import frc.robot.utils.Limelight;
import frc.robot.utils.NoteSensing;
import frc.robot.utils.PPAutonChooser;
import frc.robot.utils.ShooterTable;
import frc.robot.utils.ShooterTable.VisionTableEntry.CameraLerpStrat;
import frc.robot.utils.ShootingStrategy;
import frc.robot.utils.SubAutos;
import frc.robot.utils.SubsystemContainer;

public class RobotContainer {
    // =============================================== //
    /** Magic numbers, Vbus constants, and OI constants */
    // =============================================== //

    private static final int OI_DRIVER_CONTROLLER = 0;
    private static final int OI_OPERATOR_CONTROLLER = 1;
    private static final int OI_EMERGENCY_CONTROLLER = 2;

    // ======================== //
    /** Controllers & Subsystems */
    // ======================== //
    private final CommandXboxController driverController = new CommandXboxController(OI_DRIVER_CONTROLLER);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain;
    private final Infeed infeed = new Infeed();
    private final Shooter shooter = new Shooter();
    private final Conveyor conveyor = new Conveyor();
    private final Climber climber = new Climber();
    private final Pivot pivot = new Pivot();
    private final Fan fan = new Fan();
    private final FanPivot fanPivot = new FanPivot();
    private final Whippy whippy = new Whippy();
    private final SubsystemContainer subsystems;
    private final CommandFactory commandFactory;
    private final DashboardLogging dashboardLogging;
    private final SubAutos autos;

    private final Candle candle = new Candle();
    private final NoteSensing noteSensing = new NoteSensing();
    private final DriverCamera driverCamera = new DriverCamera();

    private final Limelight shooterLimelight = new Limelight(VisionConstants.SHOOTER_LIMELIGHT,
            VisionConstants.SHOOTER_LIMELIGHT_TRANSFORM);

    private final Limelight chassisLimelight = new Limelight(VisionConstants.CHASSIS_LIMELIGHT,
            VisionConstants.CHASSIS_LIMELIGHT_TRANSFORM);
    private final Limelight infeedLimelight3G = new Limelight(VisionConstants.INFEED_LIMELIGHT_3G,
            VisionConstants.INFEED_LIMELIGHT_3G_TRANSFORM);

    /** Shooting Strategies */
    private final ShootingStrategy shooterLimelightStrategy = new ShootingStrategy(shooterLimelight,
            CameraLerpStrat.LimelightTY);

    private final ShootingStrategy odometryStrategy = new ShootingStrategy(drivetrain);

    private final ShootingStrategy chassisLimelight2dStrategy = new ShootingStrategy(chassisLimelight,
            CameraLerpStrat.Limelight3GTY);

    // ====================== //
    /** Auton & Other Commands */
    // ====================== //
    // private SendableChooser<Command> autonChooser;
    private PPAutonChooser autonChooser;

    // ====================================================== //
    /** Drivetrain Constants, Magic numbers, and ` Limiters */
    // ====================================================== //
    private final SlewRateLimiter xLimiter = new SlewRateLimiter(4.);
    private final SlewRateLimiter yLimiter = new SlewRateLimiter(4.);
    private final SlewRateLimiter thetaLimiter = new SlewRateLimiter(4.);

    // ======================== //
    /** Swerve Control & Logging */
    // ======================== //

    public RobotContainer() {
        autos = new SubAutos(noteSensing);

        subsystems = new SubsystemContainer(drivetrain, infeed, shooter, conveyor, climber, pivot, fan, fanPivot,
                whippy, candle, noteSensing, driverCamera, shooterLimelight, chassisLimelight, infeedLimelight3G,
                shooterLimelightStrategy, odometryStrategy, chassisLimelight2dStrategy);

        commandFactory = new CommandFactory(subsystems, xLimiter, yLimiter, thetaLimiter, autos, this);

        dashboardLogging = new DashboardLogging(subsystems, xLimiter, yLimiter, thetaLimiter, autos, this);

        ShooterTable.setHeckinessLevel(() -> drivetrain.getRotation());

        new NamedCommandsReg(subsystems, xLimiter, yLimiter, thetaLimiter, autos, this);

        new ControllerBindings(new int[] { OI_DRIVER_CONTROLLER, OI_OPERATOR_CONTROLLER, OI_EMERGENCY_CONTROLLER },
                subsystems, xLimiter, yLimiter, thetaLimiter, autos, this);

        initAutonChooser();
        dashboardLogging.initDashboardLogging();
    }

    // ====================== //
    /** Auton & Named Commands */
    // ====================== //
    private void initAutonChooser() {
        autonChooser.addOptions(Map.of("Zero", commandFactory.zeroCommand(), "Shoot Note", commandFactory.zeroCommand()
                .andThen(commandFactory.runEntryCommand(() -> ShooterTable.calcShooterTableEntry(Feet.of(4.2)),
                        () -> ShotSpeeds.FAST))
                .andThen(Commands.waitUntil(commandFactory.shooterAndPivotReady()))
                .andThen(commandFactory.conveyCommand()).andThen(Commands.waitSeconds(1.0))
                .andThen(pivot.runToHomeCommand()).andThen(shooter.stopCommand())));

        autonChooser.put();
    }

    // =========================================== //
    /* Joystick & Driving Values */
    // =========================================== //

    public double getRotationSpeed() {
        return scaleDriverController(-driverController.getRightX(), thetaLimiter, commandFactory.getCurrentSpeed())
                * DrivetrainConstants.MAX_ANGULAR_SPEED;
    }

    public double getYSpeed(boolean flip) {
        return (flip ? getDriveSignum() : 1)
                * scaleDriverController(-driverController.getLeftX(), yLimiter, commandFactory.getCurrentSpeed())
                * DrivetrainConstants.MAX_SPEED;
    }

    public double getXSpeed(boolean flip) {
        return (flip ? getDriveSignum() : 1)
                * scaleDriverController(-driverController.getLeftY(), xLimiter, commandFactory.getCurrentSpeed())
                * DrivetrainConstants.MAX_SPEED;
    }

    /** Invert drivetrain based on alliance */
    public int getDriveSignum() {
        return BeakUtils.allianceIsBlue() ? 1 : -1;
    }

    /** Joystick Scaling */
    public double scaleDriverController(double controllerInput, SlewRateLimiter limiter, double baseSpeedPercent) {
        return limiter.calculate(
                controllerInput * (baseSpeedPercent + driverController.getRightTriggerAxis() * (1 - baseSpeedPercent)));
    }

    //

    // ====================== //
    /* Miscellaneous Stuff */
    // ====================== //

    public Command getAutonChooserCommand() {
        return autonChooser.getSelected();
    }

    /** Auton Command */
    public Command getAutonomousCommand() {
        return new InstantCommand(() -> drivetrain.seedFieldRelative(new Pose2d()))
                .andThen(NamedCommands.getCommand("AprilTag Zero")).andThen(getAutonChooserCommand());
    }

    public CommandFactory getCommandFactory() {
        return commandFactory;
    }

    public DashboardLogging getDashboardLogging() {
        return dashboardLogging;
    }
}
