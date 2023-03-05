package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.subsystem.AdvancedSubsystem;

public class Swerve extends AdvancedSubsystem {
  public Swerve() {}

  @Override
  public void periodic() {}

  @Override
  public void simulationPeriodic() {}

  public Pose2d getRobotPose() {
    // TODO
    return new Pose2d();
  }

  public void resetRobotPose(Pose2d pose) {
    // TODO
  }

  public void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds) {
    // TODO
  }

  @Override
  public CommandBase systemCheckCommand() {
    // TODO
    return Commands.none();
  }
}
