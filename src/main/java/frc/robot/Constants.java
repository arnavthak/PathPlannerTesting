package frc.robot;

import com.pathplanner.lib.auto.PIDConstants;

public final class Constants {
  public enum AutomationStrategy {
    ENHANCED,
    MINIMAL
  }

  public static final class Swerve {
    public static final class PathFollowing {
      public static final PIDConstants TRANSLATION_CONSTANTS = new PIDConstants(5.0, 0.0, 0.0);
      public static final PIDConstants ROTATION_CONSTANTS = new PIDConstants(0.5, 0.0, 0.0);
    }
  }
}
