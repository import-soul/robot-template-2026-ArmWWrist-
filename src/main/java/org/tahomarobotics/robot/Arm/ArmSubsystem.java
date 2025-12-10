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

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.VelocityVoltage;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import org.tahomarobotics.robot.util.AbstractSubsystem;
import com.ctre.phoenix6.hardware.TalonFX;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.util.RobustConfigurator;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static org.tahomarobotics.robot.Arm.ArmConstants.*;


public class ArmSubsystem extends AbstractSubsystem {
    //add StatusSignals and motor objects
    private final TalonFX armMotor;
    private final TalonFX wristMotor;

    private final StatusSignal<Angle> deployAngle;
    private final StatusSignal<Angle> wristAngle;

    private final VelocityVoltage velocityControl = new VelocityVoltage(RotationsPerSecond.of(0));

    ArmSubsystem() {
        armMotor = new TalonFX(RobotMap.ARM_MOTOR);
        wristMotor = new TalonFX(RobotMap.WRIST_MOTOR);

        RobustConfigurator.tryConfigureTalonFX("ArmDeployMotor", armMotor, ARM_MOTOR_CONFIG);
        RobustConfigurator.tryConfigureTalonFX("ArmWristMotor", wristMotor, WRIST_MOTOR_CONFIG);

        deployAngle = armMotor.getPosition();
        wristAngle = wristMotor.getPosition();
    }

    //add basic methods to control the arm and to get values
    Angle getDeployAngle() {
        return deployAngle.getValue();
    }

    Angle getWristAngle() {
        return wristAngle.getValue();
    }

    void setDeployVelocity(double rotPerSec) {
        armMotor.setControl(velocityControl.withVelocity(RotationsPerSecond.of(rotPerSec)));
    }

    void setDeployVelocity(AngularVelocity angularVelocity) {
        armMotor.setControl(velocityControl.withVelocity(angularVelocity));
    }

    void setWristVelocity(double rotPerSec) {
        wristMotor.setControl(velocityControl.withVelocity(RotationsPerSecond.of(rotPerSec)));
    }

    void setWristVelocity(AngularVelocity angularVelocity) {
        wristMotor.setControl(velocityControl.withVelocity(angularVelocity));
    }





    @Override
    public void subsystemPeriodic() {
        StatusSignal.refreshAll(deployAngle, wristAngle);

    }

}
