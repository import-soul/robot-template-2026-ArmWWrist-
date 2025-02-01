package org.tahomarobotics.robot.util.signals;

import com.ctre.phoenix6.BaseStatusSignal;

public record LoggedStatusSignal(String name, BaseStatusSignal signal) {
    public record List(java.util.List<LoggedStatusSignal> signals) {
        private BaseStatusSignal[] getBaseSignals() {
            return signals.stream().map(LoggedStatusSignal::signal).toArray(BaseStatusSignal[]::new);
        }

        public void setUpdateFrequencyForAll(double updateFrequency) {
            BaseStatusSignal.setUpdateFrequencyForAll(updateFrequency, getBaseSignals());
        }

        public void refreshAll() {
            BaseStatusSignal.refreshAll(getBaseSignals());
        }
    }
}
