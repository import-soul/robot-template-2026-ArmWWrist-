package org.tahomarobotics.robot.elevator;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;


public class ElevatorConstants {

    public static final double GEAR_REDUCTION = 12d / 72d * 30d / 60d;
    private static final double MAIN_PULLEY_CIRCUMFERENCE = 0.22742; // Meters
    public static final double SENSOR_COEFFICIENT = GEAR_REDUCTION * MAIN_PULLEY_CIRCUMFERENCE; // Meters
    public static final double ELEVATOR_MAX_POSE = 1.089600; // Meters
    public static final double ELEVATOR_MIN_POSE = 0.01; // Meters
    public static final double ELEVATOR_HIGH_POSE = 1.089600; // Meters
    public static final double ELEVATOR_MID_POSE = 0.884033; // Meters
    public static final double ELEVATOR_LOW_POSE = 0.184082; // Meters

    public static final double POSITION_TOLERANCE = 0.005; // Meters
    public static final double VELOCITY_TOLERANCE = 0.01; // Meters / second

    public static final double ELEVATOR_MAX_VELOCITY = 1; // Meters / sec
    public static final double ELEVATOR_MAX_ACCELERATION = ELEVATOR_MAX_VELOCITY * 4.0; // Meters / sec^2

    public static final double ZEROING_VOLTAGE = -1;

    static final TalonFXConfiguration elevatorConfig = new TalonFXConfiguration()
            .withSlot0(new Slot0Configs()
                    .withGravityType(GravityTypeValue.Elevator_Static)
                    .withKP(75.0)
                    .withKI(1.0)
                    .withKD(0.0)
                    .withKS(0.0)
                    .withKV(0.0)
                    .withKA(0.0)
                    .withKG(0.0)
            ).withMotorOutput(new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Coast)
                    .withInverted(InvertedValue.CounterClockwise_Positive))
            .withMotionMagic(new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(ELEVATOR_MAX_VELOCITY)
                    .withMotionMagicAcceleration(ELEVATOR_MAX_ACCELERATION)
                    .withMotionMagicJerk(0.0))
            .withClosedLoopGeneral(new ClosedLoopGeneralConfigs() {{
                ContinuousWrap = false;
            }})
            .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(1 / SENSOR_COEFFICIENT))
            .withAudio(new AudioConfigs().withBeepOnBoot(true).withBeepOnConfig(true));
}
