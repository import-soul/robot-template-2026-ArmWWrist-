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

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;

import static edu.wpi.first.units.Units.Degrees;

public class ArmConstants {
    //constants, of course
    //public final static ___
    public final static double ARM_SPEED = 0.2; //rotations per second
    public final static double WRIST_SPEED = 0.4; //rotations per second
    public final static Angle ARM_ZEROED_BOUND = Degrees.of(185);
    public final static Angle ARM_HIGH_BOUND = Degrees.of(180);
    public final static Angle ARM_LOW_BOUND = Degrees.of(0);
    public final static Angle WRIST_ZEROED_BOUND = Degrees.of(305);
    public final static Angle WRIST_HIGH_BOUND = Degrees.of(300);
    public final static Angle WRIST_LOW_BOUND = Degrees.of(0);
    public final static Angle BOUND_TRIGGER_TOLERANCE = Degrees.of(0.5);

    public enum RotationDirection {
        CLOCKWISE(1),
        COUNTERCLOCKWISE(-1);
        public final int sign;
        RotationDirection(int sign) {
            this.sign = sign;
        }
    }

    //motor configurations, mostly just apply reverse/hold copied from robot-with-sim-2025
    private static final MotorOutputConfigs ARM_OUTPUT_CONFIG = new MotorOutputConfigs()
        .withInverted(InvertedValue.Clockwise_Positive)
        .withNeutralMode(NeutralModeValue.Brake);

    private static final MotorOutputConfigs WRIST_OUTPUT_CONFIG = new MotorOutputConfigs()
        .withInverted(InvertedValue.Clockwise_Positive)
        .withNeutralMode(NeutralModeValue.Brake);

    public static final TalonFXConfiguration ARM_MOTOR_CONFIG = new TalonFXConfiguration()
        .withMotorOutput(ARM_OUTPUT_CONFIG);

    public static final TalonFXConfiguration WRIST_MOTOR_CONFIG = new TalonFXConfiguration()
        .withMotorOutput(WRIST_OUTPUT_CONFIG);
}
