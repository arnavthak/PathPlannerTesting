package frc.lib.swerve.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.input.controllers.XboxControllerWrapper;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class SwerveDriveWithGamepad extends CommandBase {
  private final SlewRateLimiter xVelLimiter;
  private final SlewRateLimiter yVelLimiter;
  private final SlewRateLimiter angularVelLimiter;
  private final XboxControllerWrapper driverController;
  private final Consumer<ChassisSpeeds> output;
  private final Supplier<Pose2d> poseSupplier;
  private final DoubleSupplier maxLinearVelSupplier;
  private final DoubleSupplier maxAngularVelSupplier;

  private Rotation2d rotationTarget = null;
  private final double rotationHoldFactor;

  public SwerveDriveWithGamepad(
      XboxControllerWrapper driverController,
      Consumer<ChassisSpeeds> outputFieldRelative,
      Supplier<Pose2d> poseSupplier,
      DoubleSupplier maxLinearVelSupplier,
      DoubleSupplier maxAngularVelSupplier,
      double maxLinearAccel,
      double maxAngularAccel,
      double rotationHoldFactor,
      SubsystemBase... requirements) {
    this.xVelLimiter = new SlewRateLimiter(maxLinearAccel);
    this.yVelLimiter = new SlewRateLimiter(maxLinearAccel);
    this.angularVelLimiter = new SlewRateLimiter(maxAngularAccel);
    this.driverController = driverController;
    this.output = outputFieldRelative;
    this.poseSupplier = poseSupplier;
    this.maxLinearVelSupplier = maxLinearVelSupplier;
    this.maxAngularVelSupplier = maxAngularVelSupplier;
    this.rotationHoldFactor = rotationHoldFactor;

    addRequirements(requirements);
  }

  @Override
  public void initialize() {
    this.xVelLimiter.reset(0);
    this.yVelLimiter.reset(0);
    this.angularVelLimiter.reset(0);
  }

  @Override
  public void execute() {
    double x = -driverController.getLeftY();
    double y = -driverController.getLeftX();
    double rot = -driverController.getRightX();

    if (x == 0 && y == 0 && rot == 0) {
      double xVel = this.xVelLimiter.calculate(0);
      double yVel = this.yVelLimiter.calculate(0);
      double angularVel = this.angularVelLimiter.calculate(0);

      this.output.accept(new ChassisSpeeds(xVel, yVel, angularVel));
    } else {
      Rotation2d heading = new Rotation2d(x, y);
      double targetLinearVel =
          Math.sqrt((x * x) + (y * y)) * this.maxLinearVelSupplier.getAsDouble();
      double targetAngularVel = rot * this.maxAngularVelSupplier.getAsDouble();

      double xVel = this.xVelLimiter.calculate(targetLinearVel * heading.getCos());
      double yVel = this.yVelLimiter.calculate(targetLinearVel * heading.getSin());
      double angularVel = this.angularVelLimiter.calculate(targetAngularVel);

      if (Math.abs(angularVel) <= 0.01) {
        if (rotationTarget == null) {
          rotationTarget = poseSupplier.get().getRotation();
        }

        Rotation2d error = rotationTarget.minus(poseSupplier.get().getRotation());
        angularVel += error.getRadians() * rotationHoldFactor;
      } else {
        rotationTarget = null;
      }

      this.output.accept(new ChassisSpeeds(xVel, yVel, angularVel));
    }
  }

  @Override
  public void end(boolean interrupted) {
    this.output.accept(new ChassisSpeeds());
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
