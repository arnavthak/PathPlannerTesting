package frc.lib.subsystem;

import edu.wpi.first.wpilibj.Timer;

public class SubsystemFault {
  public final String description;
  public final double timestamp;
  public final boolean isWarning;

  public SubsystemFault(String description, boolean isWarning) {
    this.description = description;
    this.timestamp = Timer.getFPGATimestamp();
    this.isWarning = isWarning;
  }

  public SubsystemFault(String description) {
    this(description, false);
  }

  @Override
  public boolean equals(Object other) {
    if (other == this) {
      return true;
    }

    if (other instanceof SubsystemFault) {
      SubsystemFault o = (SubsystemFault) other;

      return description.equals(o.description) && isWarning == o.isWarning;
    }
    return false;
  }
}
