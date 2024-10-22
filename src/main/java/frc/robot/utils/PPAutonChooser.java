package frc.robot.utils;

import java.util.Map;
import java.util.Map.Entry;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class PPAutonChooser {

    private final SendableChooser<Command> autonChooser = AutoBuilder.buildAutoChooser();

    public void addOption(String name, Command object) {
        autonChooser.addOption(name, object);
    }

    public void addOptions(Map<String, Command> options) {
        for (Entry<String, Command> e : options.entrySet()) {
            autonChooser.addOption(e.getKey(), e.getValue());
        }
    }

    public void setDefaultOption(String name, Command objeect) {
        autonChooser.setDefaultOption(name, objeect);
    }

    public void initSendable(SendableBuilder builder) {
        autonChooser.initSendable(builder);
    }

    public Command getSelected() {
        return autonChooser.getSelected();
    }

    public void close() {
        autonChooser.close();
    }

    public void put() {
        SmartDashboard.putData("Auton Chooser", autonChooser);
    }
}
