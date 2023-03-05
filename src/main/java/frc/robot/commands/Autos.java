package frc.robot.commands;

import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import java.util.HashMap;
import java.util.Map;

public final class Autos {
  private static HashMap<String, Command> eventMap;
  private static SwerveAutoBuilder autoBuilder;

  public static void init() {
    eventMap = buildEventMap();

    autoBuilder =
        new SwerveAutoBuilder(
            RobotContainer.swerve::getRobotPose,
            RobotContainer.swerve::resetRobotPose,
            Constants.Swerve.PathFollowing.TRANSLATION_CONSTANTS,
            Constants.Swerve.PathFollowing.ROTATION_CONSTANTS,
            RobotContainer.swerve::driveFieldRelative,
            eventMap,
            RobotContainer.swerve);
  }

  public static CommandBase none() {
    return Commands.none();
  }

  private static HashMap<String, Command> buildEventMap() {
    return new HashMap<>(
        Map.ofEntries(
            Map.entry("event1", Commands.print("event1")),
            Map.entry("event2", Commands.print("event2"))));
  }
}
