/*
 * Copyright 2025 Tahoma Robotics
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

package org.tahomarobotics.robot.Arm;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static org.tahomarobotics.robot.Arm.ArmConstants.*;

public class Arm {
    //name triggers up here
    public Trigger armHighBound;
    public Trigger armLowBound;
    public Trigger wristHighBound;
    public Trigger wristLowBound;
    public Trigger deployIsStopped;
    public Trigger wristIsStopped;

    private final ArmSubsystem armSubsystem;

    //define triggers in the constructor
    public Arm() {
        this(new ArmSubsystem());
    }

    public Arm(ArmSubsystem armSubsystem) {
        this.armSubsystem = armSubsystem;
        armHighBound = new Trigger(() -> { return armSubsystem.getDeployAngle().gt(Degree.of(ARM_HIGH_BOUND - BOUND_TRIGGER_TOLERANCE)); })
            .debounce(0.1);
        armLowBound = new Trigger(() -> { return armSubsystem.getDeployAngle().lt(Degree.of(ARM_LOW_BOUND + BOUND_TRIGGER_TOLERANCE)); })
            .debounce(0.1);
        wristHighBound = new Trigger(() -> { return armSubsystem.getWristAngle().gt(Degree.of(WRIST_HIGH_BOUND - BOUND_TRIGGER_TOLERANCE)); })
            .debounce(0.1);
        wristLowBound = new Trigger(() -> { return armSubsystem.getWristAngle().lt(Degree.of(WRIST_LOW_BOUND + BOUND_TRIGGER_TOLERANCE)); })
            .debounce(0.1);
        wristIsStopped = new Trigger(
            () -> armSubsystem.getWristVelocity().lt(RotationsPerSecond.of(0.01)))
            .debounce(0.1);
    }
    //create command getters (with commands class) and trigger getters

    public Command stopDeploy() {
        return armSubsystem.runOnce(() -> armSubsystem.setDeployVelocity(0))
                           .withName("Stop deploy");
    }

    public Command stopWrist() {
        return armSubsystem.runOnce(() -> armSubsystem.setWristVelocity(0))
                           .withName("Stop wrist");
    }

    public Command deployMove(int direction) {
        return armSubsystem.runOnce(() -> armSubsystem.setDeployVelocity(ARM_SPEED * direction))
                           .withName("Deploy move");
    }

    public Command wristMove(int direction) {
        // direction should be -1 (counterclockwise) or +1 (clockwise)
        return armSubsystem.runOnce(() -> armSubsystem.setWristVelocity(WRIST_SPEED * direction))
                           .withName("Wrist move");
    }

    // Wrist Zeroing Command
    private Command zeroWrist() {
        return new FunctionalCommand(
            () -> armSubsystem.setWristVoltage(),
            () -> {},
            // onEnd: stop and set zero
            interrupted -> {
                armSubsystem.zeroWrist();
            },
            // isFinished: stop when wrist is not moving
            () -> armSubsystem.isWristNotMoving(),

            armSubsystem
        ).withName("ArmSubsystem::zeroWrist");
    }

    // Arm Zeroing Command
    private Command zeroArm() {
        return new FunctionalCommand(
            () -> armSubsystem.setArmVoltage(),
            () -> {},
            // onEnd: stop and set zero
            interrupted -> {
                armSubsystem.zeroWrist();
            },
            // isFinished: stop when arm is not moving
            () -> armSubsystem.isArmNotMoving(),

            armSubsystem
        ).withName("ArmSubsystem::zeroArm");
    }
}



