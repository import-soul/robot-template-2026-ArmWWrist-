package org.tahomarobotics.robot.util;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import static edu.wpi.first.units.Units.*;

public class SysIdTest extends SubsystemIF {
    private final TalonFX motor;

    private final SysIdRoutine sysIdRoutine;
    private final VoltageOut control = new VoltageOut(0);

    public SysIdTest(SubsystemIF subsystem, TalonFX motor) {
        this.motor = motor;

        BaseStatusSignal.setUpdateFrequencyForAll(250,
                motor.getPosition(),
                motor.getVelocity(),
                motor.getMotorVoltage()
        );

        motor.optimizeBusUtilization();

        sysIdRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(
                        Volts.of(0.5).per(Second),
                        Volts.of(2),
                        null,
                        null),
                new SysIdRoutine.Mechanism(
                        (Voltage volts) -> motor.setControl(control.withOutput(volts.in(Volts))),
                        log -> log.motor("motor")
                                  .voltage(motor.getMotorVoltage().getValue())
                                  .linearPosition(Meters.of(motor.getPosition().getValueAsDouble()))
                                  .linearVelocity(MetersPerSecond.of(motor.getVelocity().getValueAsDouble())),
                        subsystem
                )
        );
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }

    @Override
    public double getEnergyUsed() {
        return 0;
    }

    @Override
    public double getTotalCurrent() {
        return 0;
    }
}
