package frc.lib.subsystem;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class BatteryUsageTest {
  private static final double DELTA = 1E-3;

  @BeforeEach
  public void setup() {
    HAL.initialize(500, 0);
  }

  @Test
  public void testStartingCharge() {
    RoboRioSim.setVInVoltage(12.0);
    assertEquals(124.982, BatteryUsage.getChargeFromRestingVoltage(), DELTA);

    RoboRioSim.setVInVoltage(12.6);
    assertEquals(217.554, BatteryUsage.getChargeFromRestingVoltage(), DELTA);

    RoboRioSim.setVInVoltage(11.7);
    assertEquals(78.696, BatteryUsage.getChargeFromRestingVoltage(), DELTA);
  }

  @Test
  public void testUsageReporting() {
    RoboRioSim.setVInVoltage(12.0);
    BatteryUsage.reset();

    BatteryUsage.reportUsage("TestSubsystem", "TestDevice", 1.0, 12.0, 1.0);
    assertEquals(0.003, BatteryUsage.getTotalUsage(), DELTA);
    assertEquals(0.578, BatteryUsage.getPercentageRemaining(), DELTA);

    BatteryUsage.reportUsage("TestSubsystem", "TestDevice2", 40.0, 12.0, 60.0);
    assertEquals(8.003, BatteryUsage.getTotalUsage(), DELTA);
    assertEquals(0.541, BatteryUsage.getPercentageRemaining(), DELTA);

    BatteryUsage.reportUsage("TestSubsystem", "TestDevice3", 20.0, 12.0);
    assertEquals(8.004, BatteryUsage.getTotalUsage(), DELTA);
    assertEquals(0.541, BatteryUsage.getPercentageRemaining(), DELTA);

    BatteryUsage.publishUsage();
    assertEquals(54.156, SmartDashboard.getNumber("BatteryUsage/PercentRemaining", 0.0), DELTA);
    assertEquals(8.004, SmartDashboard.getNumber("BatteryUsage/Total", 0.0), DELTA);
    assertEquals(8.004, SmartDashboard.getNumber("BatteryUsage/TestSubsystem/Total", 0.0), DELTA);
    assertEquals(
        0.003, SmartDashboard.getNumber("BatteryUsage/TestSubsystem/TestDevice", 0.0), DELTA);
    assertEquals(
        8.0, SmartDashboard.getNumber("BatteryUsage/TestSubsystem/TestDevice2", 0.0), DELTA);
    assertEquals(
        0.001, SmartDashboard.getNumber("BatteryUsage/TestSubsystem/TestDevice3", 0.0), DELTA);
  }
}
