package frc.lib.subsystem.selfcheck;

import com.ctre.phoenixpro.hardware.CANcoder;
import frc.lib.subsystem.SubsystemFault;
import java.util.ArrayList;
import java.util.List;

public class SelfCheckingCANCoderPro implements SelfChecking {
  private final String label;
  private final CANcoder canCoder;

  public SelfCheckingCANCoderPro(String label, CANcoder canCoder) {
    this.label = label;
    this.canCoder = canCoder;
  }

  @Override
  public List<SubsystemFault> checkForFaults() {
    List<SubsystemFault> faults = new ArrayList<>();

    if (canCoder.getFault_Hardware().getValue()) {
      faults.add(new SubsystemFault(String.format("[%s]: Hardware fault detected", label)));
    }
    if (canCoder.getFault_BootDuringEnable().getValue()) {
      faults.add(new SubsystemFault(String.format("[%s]: Device booted while enabled", label)));
    }
    if (canCoder.getFault_BadMagnet().getValue()) {
      faults.add(new SubsystemFault(String.format("[%s]: Magnet too weak", label)));
    }

    return faults;
  }
}
