package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.subsystem.BatteryUsage;

public class Robot extends TimedRobot {
  private static Robot instance;
  private Command autoCommand;
  private boolean hasMatchName = false;

  @Override
  public void robotInit() {
    instance = this;

    // Start data logging
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());

    // Set the match name to practice initially
    SmartDashboard.putString("matchName", "practice");

    // Initialize all robot systems
    RobotContainer.init();

    // Start the pathplanner server. Comment this out if it is not needed.
    //    PathPlannerServer.startServer(5812);

    addPeriodic(BatteryUsage::publishUsage, 1.0);
    addPeriodic(
        () -> {
          // We do this in periodic because robot code can start before connecting to the FMS
          if (DriverStation.isFMSAttached() && !hasMatchName) {
            String eventName = DriverStation.getEventName();
            DriverStation.MatchType matchType = DriverStation.getMatchType();
            int matchNumber = DriverStation.getMatchNumber();
            int replayNumber = DriverStation.getReplayNumber();
            SmartDashboard.putString(
                "matchName",
                String.format(
                    "%s_%s_%d_%d", eventName, matchType.name(), matchNumber, replayNumber));
            hasMatchName = true;
          }
        },
        1.0);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    // Report battery usage for misc hardware not belonging to a specific subsystem
    BatteryUsage.reportUsage(
        "Misc", "RoboRIO", RobotController.getInputCurrent(), RobotController.getInputVoltage());
    BatteryUsage.reportUsage(
        "Misc",
        "Compressor",
        RobotContainer.pneumaticHub.getCompressorCurrent(),
        RobotContainer.pneumaticHub.getInputVoltage());
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    autoCommand = RobotContainer.getAutonomousCommand();

    if (autoCommand != null) {
      autoCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    if (autoCommand != null) {
      autoCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}

  public static void addPeriodicCallback(Runnable callback, double periodSeconds) {
    // Don't add the callback if the instance is null. This is pretty much just to make unit tests
    // work
    if (instance == null) {
      return;
    }

    instance.addPeriodic(callback, periodSeconds);
  }
}
