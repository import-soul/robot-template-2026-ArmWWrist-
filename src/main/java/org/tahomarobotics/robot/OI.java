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

package org.tahomarobotics.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.littletonrobotics.junction.Logger;
import org.tahomarobotics.robot.Arm.Arm;

import java.util.function.Supplier;

import static org.tahomarobotics.robot.Arm.ArmConstants.RotationDirection.*;


public class OI {

    // -- Constants --

    private final Arm arm;

    private static final double ROTATIONAL_SENSITIVITY = 2.0;
    private static final double TRANSLATIONAL_SENSITIVITY = 1.3;

    private static final double DEADBAND = 0.09;
    private static final double TRIGGER_DEADBAND = 0.05;


    private final CommandXboxController controller = new CommandXboxController(0);
    private final CommandXboxController lessImportantController = new CommandXboxController(1);


    public OI(RobotContainer robotContainer) {
        this.arm = robotContainer.arm;
        DriverStation.silenceJoystickConnectionWarning(true);

        configureControllerBindings();
        configureLessImportantControllerBindings();

        setDefaultCommands();
    }








    // -- Bindings --

    public void configureControllerBindings() {

//      arm will only move when it's not at its high bound and the trigger is up, and will stop if either of those things change
        controller.axisGreaterThan(XboxController.Axis.kRightY.value, 0.2).negate().or(arm.armAtHighBound).onTrue(arm.stopDeploy())
                  .onFalse(arm.deployMove(CLOCKWISE));
//      arm will only move when it's not at its low bound and the trigger is down, and will stop if either of those things change
        controller.axisLessThan(XboxController.Axis.kRightY.value, -0.2).negate().or(arm.armAtLowBound).onTrue(arm.stopDeploy())
                  .onFalse(arm.deployMove(COUNTERCLOCKWISE));
        controller.leftTrigger().negate().or(arm.wristAtLowBound).onTrue(arm.stopWrist()).onFalse(arm.wristMove(COUNTERCLOCKWISE));
        controller.rightTrigger().negate().or(arm.wristAtHighBound).onTrue(arm.stopWrist()).onFalse(arm.wristMove(CLOCKWISE));
        controller.a().onTrue(arm.zeroArm());
        controller.b().onTrue(arm.zeroWrist());
    }






    public void configureLessImportantControllerBindings() {
    }

    @SuppressWarnings("SuspiciousNameCombination")
    public void setDefaultCommands() {
    }

    // -- Inputs --

    public double getLeftX() {
        return -desensitizePowerBased(controller.getLeftX(), TRANSLATIONAL_SENSITIVITY);
    }

    public double getLeftY() {
        return -desensitizePowerBased(controller.getLeftY(), TRANSLATIONAL_SENSITIVITY);
    }

    public double getRightX() {
        return -desensitizePowerBased(controller.getRightX(), ROTATIONAL_SENSITIVITY);
    }

    // -- Helper Methods --

    public double desensitizePowerBased(double value, double power) {
        value = MathUtil.applyDeadband(value, DEADBAND);
        value *= Math.pow(Math.abs(value), power - 1);
        return value;
    }
}
