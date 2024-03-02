package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import static edu.wpi.first.units.Units.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Infeed;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShotSpeeds;
import frc.robot.utils.ShooterTable;

public class Autons {

    private final CommandSwerveDrivetrain drive;
    private final Shooter shooter;
    private final Conveyor conveyor;
    private final Infeed infeed;
    private final Command smartInfeedCommand;

    public static enum StartPoses {
        TOP(new Pose2d(1.4, 6.2, new Rotation2d())),
        BTM(new Pose2d(1.4, 1.8, new Rotation2d()));

        public final Pose2d pose;

        private StartPoses(Pose2d pose) {
            this.pose = pose;
        }
    }

    public static enum Notes {
        ONE(new Pose2d(7.9, 7.4, new Rotation2d())),
        TWO(new Pose2d(7.9, 5.8, new Rotation2d())),
        THREE(new Pose2d(7.9, 4.1, new Rotation2d())),
        FOUR(new Pose2d(7.9, 2.45, new Rotation2d())),
        FIVE(new Pose2d(7.9, 0.8, new Rotation2d()));

        public final Pose2d pose;

        private Notes(Pose2d pose) {
            this.pose = pose;
        }
    }

    private static final Pose2d SAFE_TOP = new Pose2d(4.5, 6.2, new Rotation2d());
    private static final Pose2d SAFE_BTM = new Pose2d(4.5, 1.5, new Rotation2d());

    private static final Pose2d SHOOT_TOP = new Pose2d(4.2, 6.7, Rotation2d.fromDegrees(14));
    private static final Pose2d SHOOT_BTM = new Pose2d(4.3, 1.9, Rotation2d.fromDegrees(-40));

    public Command twoPieceAutonDynamic(StartPoses startPos, double scale, Notes... notesToVisit) {
        SequentialCommandGroup cmd = new SequentialCommandGroup(
                drive.runOnce(() -> drive.seedFieldRelative(startPos.pose)),
                drive.pathFindCommand(startPos == StartPoses.TOP ? SAFE_TOP : SAFE_BTM, scale, 3));

        for (var note : notesToVisit) {
            Pose2d poseToShoot = note.pose.getTranslation().getY() > 4 ? SHOOT_TOP : SHOOT_BTM;

            try {
                cmd.addCommands(
                        drive.pathFindCommand(note.pose, scale, 0).alongWith(new InstantCommand(
                                smartInfeedCommand::schedule, infeed, conveyor, shooter)),
                        drive.pathFindCommand(poseToShoot, scale, 0)
                                .alongWith(shooter.run(
                                        () -> shooter.runEntry(ShooterTable.calcShooterTableEntry(
                                                Meters.of(poseToShoot.minus(Constants.SPEAKER_DISTANCE_TARGET)
                                                        .getTranslation()
                                                        .getNorm())),
                                                ShotSpeeds.FAST)),
                                        infeed.runInfeedMotorCommand(0.8).alongWith(conveyor.runMotorCommand(0.85))
                                                .repeatedly()
                                                .withTimeout(0.5)),
                        infeed.runInfeedMotorCommand(0.)
                                .alongWith(conveyor.runMotorCommand(0.).alongWith(shooter.stopCommand())));
            } catch (Exception e) {
                e.printStackTrace();
            }
        }

        return cmd;
    }

    public Autons(CommandSwerveDrivetrain drive, Shooter shooter, Conveyor conveyor, Infeed infeed,
            Command smartInfeedCommand) {
        this.drive = drive;
        this.conveyor = conveyor;
        this.shooter = shooter;
        this.infeed = infeed;
        this.smartInfeedCommand = smartInfeedCommand;
    }
}
