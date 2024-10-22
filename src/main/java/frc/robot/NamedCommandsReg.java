package frc.robot;

import static edu.wpi.first.units.Units.Feet;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.VBusConstants;
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
import frc.robot.utils.ShooterTable;
import frc.robot.utils.ShootingStrategy;
import frc.robot.utils.SubAutos;

public class NamedCommandsReg {
    private final CommandSwerveDrivetrain drivetrain;
    private final Infeed infeed;
    private final Shooter shooter;
    private final Conveyor conveyor;
    private final Climber climber;
    private final Pivot pivot;
    private final Fan fan;
    private final FanPivot fanPivot;
    private final Whippy whippy;
    private final Candle candle;
    private final NoteSensing noteSensing;
    private final DriverCamera driverCamera;
    private final Limelight shooterLimelight, chassisLimelight, infeedLimelight3G;
    private final ShootingStrategy shooterLimelightStrategy, odometryStrategy, chassisLimelight2dStrategy;
    private final SlewRateLimiter xLimiter, yLimiter, thetaLimiter;
    private final SubAutos autos;
    private final CommandFactory commandFactory;
    private final RobotContainer robotContainer;
    public NamedCommandsReg(CommandSwerveDrivetrain drivetrain, Infeed infeed, Shooter shooter, Conveyor conveyor,
            Climber climber, Pivot pivot, Fan fan, FanPivot fanPivot, Whippy whippy, Candle candle,
            NoteSensing noteSensing, DriverCamera driverCamera, Limelight shooterLimelight, Limelight chassisLimelight,
            Limelight infeedLimelight3G, ShootingStrategy shooterLimelightStrategy, ShootingStrategy odometryStrategy,
            ShootingStrategy chassisLimelight2dStrategy, SlewRateLimiter xLimiter, SlewRateLimiter yLimiter,
            SlewRateLimiter thetaLimiter, SubAutos autos, RobotContainer robotContainer) {
        this.drivetrain = drivetrain;
        this.infeed = infeed;
        this.shooter = shooter;
        this.conveyor = conveyor;
        this.climber = climber;
        this.pivot = pivot;
        this.fan = fan;
        this.fanPivot = fanPivot;
        this.whippy = whippy;
        this.candle = candle;
        this.noteSensing = noteSensing;
        this.driverCamera = driverCamera;
        this.shooterLimelight = shooterLimelight;
        this.chassisLimelight = chassisLimelight;
        this.infeedLimelight3G = infeedLimelight3G;
        this.shooterLimelightStrategy = shooterLimelightStrategy;
        this.odometryStrategy = odometryStrategy;
        this.chassisLimelight2dStrategy = chassisLimelight2dStrategy;
        this.xLimiter = xLimiter;
        this.yLimiter = yLimiter;
        this.thetaLimiter = thetaLimiter;
        this.autos = autos;
        this.robotContainer = robotContainer;
        commandFactory = robotContainer.getCommandFactory();

        initNamedCommands();
    }

    private void initNamedCommands() {
        /* ... */
        NamedCommands.registerCommand("Pivot Zero", pivot.zeroCommand());

        /* Infeed & Spit */
        NamedCommands.registerCommand("Smart Infeed", commandFactory.smartInfeedCommand());

        NamedCommands.registerCommand("Dumb Infeed", commandFactory
                .runBoth(true, VBusConstants.SLOW_CONVEYOR_VBUS, VBusConstants.INFEED_VBUS).withTimeout(.25));

        NamedCommands.registerCommand("Infeed", infeed.runMotorCommand(VBusConstants.INFEED_VBUS)
                .alongWith(conveyor.runMotorCommand(VBusConstants.FAST_CONVEYOR_VBUS)).repeatedly());// .withTimeout(1.5));

        NamedCommands.registerCommand("C Infeed", infeed.runMotorCommand(VBusConstants.INFEED_VBUS)
                .alongWith(conveyor.runMotorCommand(VBusConstants.FAST_CONVEYOR_VBUS)).repeatedly().withTimeout(1.0));

        NamedCommands.registerCommand("First Spit Note",
                infeed.runMotorCommand(VBusConstants.INFEED_VBUS)
                        .alongWith(conveyor.runMotorCommand(VBusConstants.FAST_CONVEYOR_VBUS))
                        .alongWith(shooter.spinBothCommand(0.20)).repeatedly());
        NamedCommands.registerCommand("Spit Note",
                infeed.runMotorCommand(VBusConstants.INFEED_VBUS)
                        .alongWith(conveyor.runMotorCommand(VBusConstants.FAST_CONVEYOR_VBUS))
                        .alongWith(shooter.spinBothCommand(0.11)).repeatedly());

        NamedCommands.registerCommand("Prepare Spit", shooter.spinBothCommand(0.15));

        NamedCommands.registerCommand("Fix Note", commandFactory.fixNoteCommand());

        NamedCommands.registerCommand("Finish Infeed",
                commandFactory.smartInfeedAutoCommand().andThen(shooter.runShotCommand(ShotSpeeds.FAST)));

        NamedCommands.registerCommand("Magic Shoot",
                Commands.waitSeconds(0.1).andThen(commandFactory.magicShootCommand(() -> odometryStrategy)));// updateDrivePoseMT2Command().repeatedly().withTimeout(0.1).andThen(magicShootCommand(()
        // -> odometryStrategy)));

        NamedCommands.registerCommand("Magic Shoot++", commandFactory.magicShootCommand(() -> odometryStrategy));

        NamedCommands.registerCommand("Mega Tag Shoot", commandFactory.magicShootCommand(() -> odometryStrategy));
        NamedCommands.registerCommand("Choose Shoot", commandFactory.magicShootCommand());

        NamedCommands.registerCommand("Spit Wipe Note 5-3",
                drivetrain.staticAlign(() -> Constants.AutoPoses.DOWNWARD_ROTATION).withTimeout(0.5)
                        .andThen(commandFactory.conveyCommand().withTimeout(0.5))
                        .andThen(drivetrain.staticAlign(() -> Rotation2d.fromDegrees(90.0)).withTimeout(0.5))
                        .onlyIf(noteSensing.hasInfedSupplier()));

        NamedCommands.registerCommand("Spit Wipe Note 2",
                drivetrain.staticAlign(() -> Constants.AutoPoses.UPWARD_ROTATION).withTimeout(0.5)
                        .andThen(commandFactory.runThree(() -> VBusConstants.FAST_CONVEYOR_VBUS,
                                () -> VBusConstants.INFEED_VBUS, () -> 0.2).withTimeout(0.25))
                        .andThen(drivetrain.staticAlign(() -> Rotation2d.fromDegrees(90.0)).withTimeout(0.5))
                        .onlyIf(noteSensing.hasInfedSupplier()));

        /* Shooter & Pivot */
        NamedCommands.registerCommand("Fast Shooter", shooter.runShotCommand(ShotSpeeds.FAST));

        NamedCommands.registerCommand("Shooter", shooter.runShotCommand(ShotSpeeds.FAST, 0.85));

        NamedCommands.registerCommand("Stop Shooter", shooter.stopCommand());

        NamedCommands.registerCommand("Home Pivot", pivot.runToHomeCommand());

        NamedCommands.registerCommand("Rotate To Speaker Source PC3",
                drivetrain.staticAlign(() -> Rotation2d.fromDegrees(BeakUtils.allianceIsBlue() ? -63 : -117)));

        /* 4 piece pivots */
        NamedCommands.registerCommand("Preload Note",
                pivot.runToPositionCommand(16.0).alongWith(driverCamera.setShooterCameraCommand())); // 17

        NamedCommands.registerCommand("Note A", pivot.runToPositionCommand(11)); // 11
        NamedCommands.registerCommand("Note B", pivot.runToPositionCommand(14.625)); // 15
        NamedCommands.registerCommand("Note C", pivot.runToPositionCommand(13.5)); // 13.5
        NamedCommands.registerCommand("Preload Note at sub", pivot.runToPositionCommand(30));

        NamedCommands.registerCommand("Note 1", pivot.runToPositionCommand(17.5)); // 17.5

        /* Pathfinding Shots */
        NamedCommands.registerCommand("Amp Shot",
                commandFactory.pathfindingShotCommand(15.5, Constants.LEFT_SHOT, 0.875, 0));

        NamedCommands.registerCommand("Stationary Amp Shot", commandFactory.shootCommand(17.0));
        NamedCommands.registerCommand("Rojo stationary Amp Shot", commandFactory.shootCommand(17.5));
        NamedCommands.registerCommand("Sad Stationary Amp Shot 2", commandFactory.shootCommand(16.5));
        NamedCommands.registerCommand("Epic Amp Shot", commandFactory.shootCommand(7.5)); // 7.5

        NamedCommands.registerCommand("Center Pathfinding Shot",
                commandFactory.pathfindingShotCommand(13.0, Constants.CENTER_SHOT, 0.8, 0.));

        NamedCommands.registerCommand("Stationary Source Shot", commandFactory.shootCommand(22.1));
        NamedCommands.registerCommand("Magic Source Shot",
                commandFactory.magicShootCommand(21.6, () -> chassisLimelight2dStrategy, true));

        NamedCommands.registerCommand("Spitless Source Shot 1", commandFactory.shootCommand(15.8));
        NamedCommands.registerCommand("Spitless Source Shot 2", commandFactory.shootCommand(17));
        NamedCommands.registerCommand("Spitless Source Shot 3", commandFactory.shootCommand(17.2));

        NamedCommands.registerCommand("Right Preload", pivot.runOnce(pivot::zeroEncoder)
                .andThen(commandFactory.shootCommand(() -> ShooterTable.calcShooterTableEntry(Feet.of(5.2)))));

        NamedCommands.registerCommand("Center Preload", pivot.runOnce(pivot::zeroEncoder)
                .andThen(commandFactory.shootCommand(() -> ShooterTable.calcShooterTableEntry(Feet.of(4.2)))));

        NamedCommands.registerCommand("Note C Pathfinding",
                commandFactory.mirroredPathfindingShotCommand(12.35, Constants.NOTE_C_SHOT, 0.85, 0.));
        // PBAC Auto Commands
        NamedCommands.registerCommand("Preload Stationary", commandFactory.shootCommand(4.2));
        NamedCommands.registerCommand("Stationary Shot B", commandFactory.shootCommand(10));
        NamedCommands.registerCommand("Stationary Shot A", commandFactory.shootCommand(10));
        NamedCommands.registerCommand("Stationary Shot C", commandFactory.shootCommand(9.6));

        NamedCommands.registerCommand("P Amp Shot", commandFactory.shootCommand(13.5));
        NamedCommands.registerCommand("Stationary Shot Amp", commandFactory.shootCommand(13.5));

        NamedCommands.registerCommand("Source Pivot", pivot.runToPositionCommand(5.0));
        NamedCommands.registerCommand("Source Pivot 4", pivot.runToPositionCommand(6.0));
        NamedCommands.registerCommand("Source Pivot Red", pivot.runToPositionCommand(5.0));

        NamedCommands.registerCommand("Amp Pivot", pivot.runToPositionCommand(4.75));
        NamedCommands.registerCommand("Amp Pivot Red", pivot.runToPositionCommand(4.75));

        NamedCommands.registerCommand("Convey", commandFactory.conveyCommand());
        NamedCommands.registerCommand("4 Piece Pivot", pivot.runToPositionCommand(14));
        NamedCommands.registerCommand("P Pivot", pivot.runToPositionCommand(16));
        NamedCommands.registerCommand("Stationary Source Shot", commandFactory.shootCommand(15.3));

        NamedCommands.registerCommand("4 Piece A", pivot.runToPositionCommand(13.75));
        NamedCommands.registerCommand("4 Piece C", pivot.runToPositionCommand(14.0));

        /* Sub Autos */
        NamedCommands.registerCommand("5 Or 4", autos.note5or4());
        NamedCommands.registerCommand("4 Or 3", autos.note4or3());

        NamedCommands.registerCommand("1 Or 2", autos.note1or2());
        NamedCommands.registerCommand("2 Or 3", autos.note2or3());

        NamedCommands.registerCommand("4 Or 5", autos.note4or5());
        NamedCommands.registerCommand("5 Or 3", autos.note5or3());

        NamedCommands.registerCommand("2 Or 1", autos.note2or1());
        NamedCommands.registerCommand("1 Or 3", autos.note1or3());

        NamedCommands.registerCommand("3 Or None", autos.note3orStop());
        NamedCommands.registerCommand("3 Or None Amp", autos.note3orStopAmp());

        NamedCommands.registerCommand("MT2 Loc ON", Commands.runOnce(() -> commandFactory.setUseMT2(true)));
        NamedCommands.registerCommand("MT2 Loc OFF", Commands.runOnce(() -> commandFactory.setUseMT2(false)));

        NamedCommands.registerCommand("PB3AC skip shot",
                (Commands.waitSeconds(0.2).andThen(commandFactory.magicShootCommand(() -> odometryStrategy)))
                        .onlyIf(noteSensing.getHasInfedCache()));

        NamedCommands.registerCommand("Cache hasInfed", Commands.runOnce(noteSensing::cacheInfeedState));
    }
}
