package frc.lib.subsystem;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.HashMap;

public class BatteryUsage {
  private static final double BATTERY_WATT_HOURS = 18.0 * 12.0; // 18 Amp hours at 12V
  private static final double startingCharge = getChargeFromRestingVoltage();

  // Nested hash map to track usage per-device
  private static final HashMap<String, HashMap<String, Double>> subsystemUsage = new HashMap<>();

  /**
   * Report battery usage for a single device
   *
   * @param subsystem The subsystem the device belongs to
   * @param device The name of the device
   * @param amps Current supply current
   * @param volts Current supply voltage
   * @param seconds Time step
   */
  public static void reportUsage(
      String subsystem, String device, double amps, double volts, double seconds) {
    subsystemUsage.putIfAbsent(subsystem, new HashMap<>());

    subsystemUsage.get(subsystem).putIfAbsent(device, 0.0);
    subsystemUsage
        .get(subsystem)
        .put(
            device,
            subsystemUsage.get(subsystem).get(device)
                + Math.abs(amps * volts) * (seconds / 3600.0));
  }

  /**
   * Report battery usage for a single device
   *
   * @param subsystem The subsystem the device belongs to
   * @param device The name of the device
   * @param amps Current supply current
   * @param volts Current supply voltage
   */
  public static void reportUsage(String subsystem, String device, double amps, double volts) {
    reportUsage(subsystem, device, amps, volts, 0.02);
  }

  /**
   * Get the total battery usage in watt hours
   *
   * @return Total battery usage
   */
  protected static double getTotalUsage() {
    double totalUsage = 0.0;
    for (HashMap<String, Double> subsystem : subsystemUsage.values()) {
      for (double deviceUsage : subsystem.values()) {
        totalUsage += deviceUsage;
      }
    }
    return totalUsage;
  }

  /**
   * Get the estimated percent charge remaining on the battery
   *
   * @return Percent charge remaining
   */
  public static double getPercentageRemaining() {
    return getPercentageRemaining(getTotalUsage());
  }

  protected static double getPercentageRemaining(double totalUsage) {
    return Math.max(0, startingCharge - totalUsage) / BATTERY_WATT_HOURS;
  }

  /**
   * Put all usage information on SmartDashboard. This should only be called once from
   * robotPeriodic.
   */
  public static void publishUsage() {
    double totalUsage = 0.0;
    for (String subsystem : subsystemUsage.keySet()) {
      double subsystemTotal = 0.0;
      for (String device : subsystemUsage.get(subsystem).keySet()) {
        subsystemTotal += subsystemUsage.get(subsystem).get(device);
        SmartDashboard.putNumber(
            "BatteryUsage/" + subsystem + "/" + device, subsystemUsage.get(subsystem).get(device));
        SmartDashboard.putNumber("BatteryUsage/" + subsystem + "/Total", subsystemTotal);
      }
      totalUsage += subsystemTotal;
    }
    SmartDashboard.putNumber("BatteryUsage/Total", totalUsage);
    SmartDashboard.putNumber(
        "BatteryUsage/PercentRemaining", getPercentageRemaining(totalUsage) * 100);
  }

  /**
   * Estimate the starting charge percent of the battery from its resting voltage. This will be
   * called when robot code starts.
   *
   * @return The estimated starting charge in watt hours
   */
  protected static double getChargeFromRestingVoltage() {
    double currentVoltage = RobotController.getBatteryVoltage();

    // Is this correct? No idea. Close enough? Maybe.
    double estimatedChargePercent = Math.max((0.71429 * currentVoltage) - 7.99286, 0.0);
    return estimatedChargePercent * BATTERY_WATT_HOURS;
  }

  /** Reset the battery usage tracking */
  protected static void reset() {
    subsystemUsage.clear();
  }
}
