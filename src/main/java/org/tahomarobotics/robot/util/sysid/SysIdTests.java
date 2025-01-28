package org.tahomarobotics.robot.util.sysid;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.tahomarobotics.robot.util.SubsystemIF;

import java.util.ArrayList;
import java.util.List;

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
        Voltage stepSize,
        TalonFX... followers
    ) {
        List<TalonFX> motors = new ArrayList<>(followers.length + 1);
        motors.add(motor);
        motors.addAll(List.of(followers));

        // Ensure the signals are updating

        motors.forEach(m -> {
            BaseStatusSignal.setUpdateFrequencyForAll(
                250,
                m.getPosition(),
                m.getVelocity(),
                m.getMotorVoltage()
            );
        });

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
                (Voltage volts) -> {
                    motors.forEach(m -> m.setControl(voltageControl.withOutput(volts)));
                },
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
