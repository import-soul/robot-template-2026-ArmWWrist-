package org.tahomarobotics.robot.elevator;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import org.tahomarobotics.robot.RobotMap;

public class ElevatorConstants {
    // Gearing

    public static final double GEAR_REDUCTION = 12d / 52d;

    // Pulley

    public static final double MAIN_PULLEY_RADIUS = Units.inchesToMeters(1.1056);
    public static final double MAIN_PULLEY_CIRCUMFERENCE = 2 * Math.PI * MAIN_PULLEY_RADIUS;

    // Poses

    public static final double ELEVATOR_MAX_POSE = 1; // Meters
    public static final double ELEVATOR_MIN_POSE = 0.01; // Meters

    public static final double ELEVATOR_HIGH_POSE = 0.98; // Meters
    public static final double ELEVATOR_MID_POSE = 0.45; // Meters
    public static final double ELEVATOR_LOW_POSE = 0.1; // Meters

    public static final double ELEVATOR_LOW_STAGE_MAX = 0.5461; // Meters

    // Tolerances

    public static final double POSITION_TOLERANCE = 0.005; // Meters
    public static final double VELOCITY_TOLERANCE = 0.01; // Meters / sec

    // Motion

    public static final double ELEVATOR_MAX_VELOCITY = 4; // Meters / sec
    public static final double ELEVATOR_MAX_ACCELERATION = ELEVATOR_MAX_VELOCITY; // Meters / sec^2
    public static final double ELEVATOR_MAX_JERK = ELEVATOR_MAX_ACCELERATION * 4.0; // Meters / sec^3

    // Configuration

    static final TalonFXConfiguration motorConfiguration = new TalonFXConfiguration()
        .withSlot0(
            new Slot0Configs()
                .withGravityType(GravityTypeValue.Elevator_Static)
                // SysId'd 1/17/2025
                .withKP(50.438)
                .withKI(100)
                .withKS(0.097499)
                .withKV(3.3441)
                .withKA(0.16116)
                .withKG(0.084958)
        ).withMotorOutput(
            new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Coast) // TODO
                .withInverted(InvertedValue.CounterClockwise_Positive)
        ).withCurrentLimits(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimitEnable(false)
                .withSupplyCurrentLimitEnable(false)
        ).withMotionMagic(
            new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(ELEVATOR_MAX_VELOCITY)
                .withMotionMagicAcceleration(ELEVATOR_MAX_ACCELERATION)
                .withMotionMagicJerk(ELEVATOR_MAX_JERK)
        ).withClosedLoopGeneral(
            new ClosedLoopGeneralConfigs()
                .withContinuousWrap(false)
        ).withFeedback(new FeedbackConfigs() {{
            SensorToMechanismRatio = 1 / MAIN_PULLEY_CIRCUMFERENCE;

            FeedbackRemoteSensorID = RobotMap.ELEVATOR_ENCODER;
            FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder; // TODO: If we license, use fused.
        }}).withAudio(
            new AudioConfigs().withBeepOnBoot(true).withBeepOnConfig(true)
        );
}
