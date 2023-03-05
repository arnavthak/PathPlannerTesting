package frc.lib.subsystem.selfcheck;

import com.ctre.phoenixpro.hardware.TalonFX;
import frc.lib.subsystem.SubsystemFault;
import java.util.ArrayList;
import java.util.List;

public class SelfCheckingTalonFXPro implements SelfChecking {
  private final String label;
  private final TalonFX talon;

  public SelfCheckingTalonFXPro(String label, TalonFX talon) {
    this.label = label;
    this.talon = talon;
  }

  @Override
  public List<SubsystemFault> checkForFaults() {
    List<SubsystemFault> faults = new ArrayList<>();

    if (talon.getFault_Hardware().getValue()) {
      faults.add(new SubsystemFault(String.format("[%s]: Hardware fault detected", label)));
    }
    if (talon.getFault_BootDuringEnable().getValue()) {
      faults.add(new SubsystemFault(String.format("[%s]: Device booted while enabled", label)));
    }
    if (talon.getFault_DeviceTemp().getValue()) {
      faults.add(
          new SubsystemFault(String.format("[%s]: Device temperature too high", label), true));
    }
    if (talon.getFault_ProcTemp().getValue()) {
      faults.add(
          new SubsystemFault(String.format("[%s]: Processor temperature too high", label), true));
    }

    return faults;
  }
}
