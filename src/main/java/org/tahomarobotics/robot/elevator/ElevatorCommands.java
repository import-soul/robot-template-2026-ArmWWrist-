package org.tahomarobotics.robot.elevator;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.tinylog.Logger;

public class ElevatorCommands {
    public static Command createCalibrateElevator(Elevator elevator) {
        String FINALIZE_KEY = "Finalize";

        Command cmd = (
            Commands.waitUntil(() -> SmartDashboard.getBoolean(FINALIZE_KEY, false))
                    .beforeStarting(() -> {
                        SmartDashboard.putBoolean(FINALIZE_KEY, false);
                        elevator.initializeCalibration();
                        Logger.info("Calibrating elevator...");
                    }).finallyDo(interrupted -> {
                        if (interrupted) {
                            Logger.info("Cancelling elevator calibration.");
                            elevator.applyOffset();
                        } else {
                            Logger.info("Elevator calibrated!");
                            elevator.finalizeCalibration();
                        }
                    })
        ).onlyWhile(RobotState::isDisabled).ignoringDisable(true);
        cmd.addRequirements(elevator);

        return cmd;
    }
}
