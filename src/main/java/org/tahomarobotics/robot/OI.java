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
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.littletonrobotics.junction.Logger;
import org.tahomarobotics.robot.Arm.Arm;

import static org.tahomarobotics.robot.Arm.ArmConstants.RotationDirection.*;


public class OI {

    // -- Constants --

    private final Arm arm;

    private static final double ROTATIONAL_SENSITIVITY = 2.0;
    private static final double TRANSLATIONAL_SENSITIVITY = 1.3;

    private static final double DEADBAND = 0.09;
    private static final double TRIGGER_DEADBAND = 0.05;

    //MAKE PRIVATE AFTER TESTING!!!
    public final CommandXboxController controller = new CommandXboxController(0);
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
        // decide when commands should be run with triggers
        //arm moves when joystick is up, and will stop when the arm is at its limit or the joystick isn't up
        controller.axisGreaterThan(XboxController.Axis.kRightY.value, 0.2).onTrue(arm.deployMove(CLOCKWISE))
                  .and(arm.armAtHighBound.negate()).onFalse(arm.stopDeploy());
        //arm moves when joystick is down, and will stop when the arm is at its limit or the joystick isn't down
        controller.axisLessThan(XboxController.Axis.kRightY.value, -0.2).onTrue(arm.deployMove(COUNTERCLOCKWISE))
                  .and(arm.armAtLowBound.negate()).onFalse(arm.stopDeploy());
        // when the right trigger is pressed, the wrist moves clockwise, and will stop if the arm reaches its high bound or the trigger stops being pressed.
        controller.rightTrigger().onTrue(arm.wristMove(CLOCKWISE)).and(arm.wristAtHighBound.negate()).onFalse(arm.stopWrist());
        // when the left trigger is pressed, the wrist moves counterclockwise, and will stop if the arm reaches its low bound or the trigger stops being pressed.
        controller.leftTrigger().onTrue(arm.wristMove(COUNTERCLOCKWISE)).and(arm.wristAtLowBound.negate()).onFalse(arm.stopWrist());
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
