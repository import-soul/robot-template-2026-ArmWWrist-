package org.tahomarobotics.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.tahomarobotics.robot.chassis.Chassis;
import org.tahomarobotics.robot.elevator.Elevator;
import org.tahomarobotics.robot.util.SubsystemIF;

import java.util.ArrayList;
import java.util.List;

public class Robot extends TimedRobot {
    private final List<SubsystemIF> subsystems = new ArrayList<>();

    // Robot

    public Robot() {
        subsystems.add(OI.getInstance().initialize());
        subsystems.add(Chassis.getInstance().initialize());
        subsystems.add(Elevator.getInstance().initialize());
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    // Disabled

    @Override
    public void disabledInit() {
        subsystems.forEach(SubsystemIF::onDisabledInit);
    }

    @Override
    public void disabledPeriodic() {
    }

    // Autonomous

    @Override
    public void autonomousInit() {
        subsystems.forEach(SubsystemIF::onAutonomousInit);
    }

    @Override
    public void autonomousPeriodic() {
    }

    // Teleop

    @Override
    public void teleopInit() {
        subsystems.forEach(SubsystemIF::onTeleopInit);
    }

    @Override
    public void teleopPeriodic() {
    }

    // Test

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
    }

    // Simulation

    @Override
    public void simulationInit() {
    }

    @Override
    public void simulationPeriodic() {
    }
}
