package org.tahomarobotics.robot.util.sysid;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.tahomarobotics.robot.util.SubsystemIF;

/**
 * A helper class for characterizing motors with SysId using Phoenix 6's {@link SignalLogger}.
 */
public class SysIdTests {
    // -- Characterization --

    public static Test characterize(
        String testName,
        SubsystemIF subsystem,
        TalonFX motor,
        Velocity<VoltageUnit> rampRate,
        Voltage stepSize
    ) {
        return characterize(testName, subsystem, motor, rampRate, stepSize, null, false);
    }

    public static Test characterize(
        String testName,
        SubsystemIF subsystem,
        TalonFX motor,
        Velocity<VoltageUnit> rampRate,
        Voltage stepSize,
        TalonFX follower,
        boolean inverted
    ) {
        if (follower != null) {
            follower.setControl(new Follower(motor.getDeviceID(), inverted));
        }

        // Ensure the signals are updating

        BaseStatusSignal.setUpdateFrequencyForAll(
            250,
            motor.getPosition(),
            motor.getVelocity(),
            motor.getMotorVoltage()
        );

        // Create SysId routine

        VoltageOut voltageControl = new VoltageOut(0);

        SysIdRoutine routine = new SysIdRoutine(
            new SysIdRoutine.Config(
                rampRate,
                stepSize,
                null,
                state -> SignalLogger.writeString("state", state.toString())
            ),
            new SysIdRoutine.Mechanism(
                (Voltage volts) -> motor.setControl(voltageControl.withOutput(volts)),
                null,
                subsystem
            )
        );

        return new Test(
            testName,
            routine.quasistatic(SysIdRoutine.Direction.kForward),
            routine.quasistatic(SysIdRoutine.Direction.kReverse),
            routine.dynamic(SysIdRoutine.Direction.kForward),
            routine.dynamic(SysIdRoutine.Direction.kReverse)
        );
    }

    // -- Record(s) --

    public record Test(String name,
                       Command quasistaticForward, Command quasistaticReverse,
                       Command dynamicForward, Command dynamicReverse) {}
}
