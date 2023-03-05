package frc.lib.input.controllers.rumble;

public class RumbleOn extends RumbleAnimation {
  @Override
  public double getRumbleOutput(double timeSeconds) {
    return 1.0;
  }
}
