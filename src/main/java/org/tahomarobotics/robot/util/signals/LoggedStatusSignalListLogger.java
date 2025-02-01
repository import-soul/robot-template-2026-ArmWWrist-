package org.tahomarobotics.robot.util.signals;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;

@CustomLoggerFor(LoggedStatusSignal.List.class)
public class LoggedStatusSignalListLogger extends ClassSpecificLogger<LoggedStatusSignal.List> {
    public LoggedStatusSignalListLogger() { super(LoggedStatusSignal.List.class); }

    @Override
    protected void update(EpilogueBackend backend, LoggedStatusSignal.List signals) {
        signals.signals().forEach(signal -> {
            EpilogueBackend entry = backend.getNested(signal.name());

            entry.log("value", signal.signal().getValueAsDouble());
            entry.log("status", signal.signal().getStatus());
            entry.log("units", signal.signal().getUnits());
        });
    }
}
