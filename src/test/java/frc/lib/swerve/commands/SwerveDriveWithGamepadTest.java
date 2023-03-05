package frc.lib.swerve.commands;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj.simulation.XboxControllerSim;
import frc.lib.input.controllers.XboxControllerWrapper;
import java.util.concurrent.atomic.AtomicReference;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.parallel.ResourceLock;

public class SwerveDriveWithGamepadTest {
  private static final double DELTA = 1E-2;

  @BeforeEach
  public void setup() {
    HAL.initialize(500, 0);
    SimHooks.pauseTiming();
  }

  @AfterEach
  public void cleanup() {
    SimHooks.resumeTiming();
  }

  @Test
  @ResourceLock("timing")
  public void testSwerveDriveWithGamepad() {
    AtomicReference<ChassisSpeeds> out = new AtomicReference<>(new ChassisSpeeds());
    XboxControllerWrapper controller = new XboxControllerWrapper(0);
    XboxControllerSim controllerSim = new XboxControllerSim(controller);

    controllerSim.setLeftX(-0.5);
    controllerSim.setLeftY(-0.5);
    controllerSim.setRightX(-0.5);
    controllerSim.notifyNewData();

    SwerveDriveWithGamepad cmd =
        new SwerveDriveWithGamepad(
            controller,
            out::set,
            Pose2d::new,
            () -> 4.0,
            () -> Units.degreesToRadians(450),
            8.0,
            Units.degreesToRadians(720),
            0);

    // Check that command doesn't end
    assertFalse(cmd.isFinished());

    // Run one step and check that everything is limited properly
    cmd.initialize();
    SimHooks.stepTiming(0.25);
    cmd.execute();

    assertEquals(2.0, out.get().vxMetersPerSecond, DELTA);
    assertEquals(2.0, out.get().vyMetersPerSecond, DELTA);
    assertEquals(Units.degreesToRadians(180), out.get().omegaRadiansPerSecond, DELTA);

    // End the command and check that 0 speeds are output
    cmd.end(true);

    assertEquals(0.0, out.get().vxMetersPerSecond, DELTA);
    assertEquals(0.0, out.get().vyMetersPerSecond, DELTA);
    assertEquals(0.0, out.get().omegaRadiansPerSecond, DELTA);

    // Initialize and run the command again to check that the limiters were reset
    // Since we do not step the timing here, we can expect a 0 output
    cmd.initialize();
    cmd.execute();

    assertEquals(0.0, out.get().vxMetersPerSecond, DELTA);
    assertEquals(0.0, out.get().vyMetersPerSecond, DELTA);
    assertEquals(0.0, out.get().omegaRadiansPerSecond, DELTA);
  }

  @Test
  @ResourceLock("timing")
  public void testSwerveDriveWithGamepadStationary() {
    AtomicReference<ChassisSpeeds> out = new AtomicReference<>(new ChassisSpeeds());
    XboxControllerWrapper controller = new XboxControllerWrapper(0);
    XboxControllerSim controllerSim = new XboxControllerSim(controller);

    controllerSim.setLeftX(0.0);
    controllerSim.setLeftY(0.0);
    controllerSim.setRightX(0.0);
    controllerSim.notifyNewData();

    SwerveDriveWithGamepad cmd =
        new SwerveDriveWithGamepad(
            controller,
            out::set,
            Pose2d::new,
            () -> 4.0,
            () -> Units.degreesToRadians(450),
            8.0,
            Units.degreesToRadians(720),
            0);

    cmd.initialize();
    SimHooks.stepTiming(0.25);
    cmd.execute();

    assertEquals(0.0, out.get().vxMetersPerSecond, DELTA);
    assertEquals(0.0, out.get().vyMetersPerSecond, DELTA);
    assertEquals(0.0, out.get().omegaRadiansPerSecond, DELTA);
  }
}
