package frc.robot;

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.lib.input.controllers.XboxControllerWrapper;
import frc.lib.input.controllers.rumble.*;
import frc.lib.jetson.JetsonClient;
import frc.robot.commands.Autos;
import frc.robot.subsystems.Swerve;

public class RobotContainer {
  // Controllers
  public static final XboxControllerWrapper driver = new XboxControllerWrapper(0);
  public static final XboxControllerWrapper coDriver = new XboxControllerWrapper(1);

  // Subsystems
  public static final Swerve swerve = new Swerve();

  // Other Hardware
  public static final PowerDistribution powerDistribution = new PowerDistribution();
  public static final PneumaticHub pneumaticHub = new PneumaticHub();

  private static final SendableChooser<Command> autoChooser = new SendableChooser<>();
  private static final SendableChooser<Constants.AutomationStrategy> automationChooser =
      new SendableChooser<>();

  public static final JetsonClient jetson = new JetsonClient();

  public static void init() {
    configureButtonBindings();

    Autos.init();

    autoChooser.setDefaultOption("Example", Commands.print("Example Auto"));
    autoChooser.addOption("None", Autos.none());

    automationChooser.setDefaultOption("Enhanced", Constants.AutomationStrategy.ENHANCED);
    automationChooser.addOption("Minimal", Constants.AutomationStrategy.MINIMAL);

    SmartDashboard.putData("Autonomous Mode", autoChooser);
    SmartDashboard.putData("Automation Strategy", automationChooser);

    SmartDashboard.putData("SystemStatus/AllSystemsCheck", allSystemsCheckCommand());
  }

  /** Configure all button bindings */
  private static void configureButtonBindings() {
    driver.A().onTrue(new InstantCommand(() -> driver.setRumbleAnimation(new RumbleOn())));
    driver.B().onTrue(new InstantCommand(() -> driver.setRumbleAnimation(new RumbleOff())));
    driver.X().onTrue(new InstantCommand(() -> driver.setRumbleAnimation(new RumblePulse(1))));
    driver.Y().onTrue(new InstantCommand(() -> driver.setRumbleAnimation(new RumbleSinWave(2))));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public static Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  /**
   * Get the selected automation strategy from the dashboard
   *
   * @return The selected automation strategy
   */
  public static Constants.AutomationStrategy getAutomationStrategy() {
    return automationChooser.getSelected();
  }

  /**
   * This method creates a command group for running all system check commands
   *
   * @return The full-robot system check command
   */
  public static CommandBase allSystemsCheckCommand() {
    return Commands.sequence(
        swerve.systemCheckCommand(), new PrintCommand("Some other system check"));
  }
}
