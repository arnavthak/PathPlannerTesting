package frc.lib.swerve;

import com.ctre.phoenixpro.StatusCode;
import com.ctre.phoenixpro.configs.CANcoderConfiguration;
import com.ctre.phoenixpro.configs.TalonFXConfiguration;
import com.ctre.phoenixpro.controls.NeutralOut;
import com.ctre.phoenixpro.controls.PositionVoltage;
import com.ctre.phoenixpro.controls.VelocityVoltage;
import com.ctre.phoenixpro.hardware.CANcoder;
import com.ctre.phoenixpro.hardware.TalonFX;
import com.ctre.phoenixpro.signals.CANcoder_AbsoluteSensorRangeValue;
import com.ctre.phoenixpro.signals.CANcoder_SensorDirectionValue;
import com.ctre.phoenixpro.signals.InvertedValue;
import com.ctre.phoenixpro.sim.CANcoderSimState;
import com.ctre.phoenixpro.sim.TalonFXSimState;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.subsystem.AdvancedSubsystem;
import java.util.EnumSet;

/** Implementation for an SDS Mk4 swerve module using Falcon 500s and phoenix pro */
public class Mk4SwerveModulePro extends AdvancedSubsystem {
  public enum ModuleCode {
    FL,
    FR,
    BL,
    BR
  }

  // Volts to meters/sec
  private static final double DRIVE_KV = 2.5;
  // Volts to meters/sec^2
  private static final double DRIVE_KA = 0.27;

  // Volts to deg/sec
  private static final double ROTATION_KV = 12.0 / 900;
  // Volts to deg/sec^2
  private static final double ROTATION_KA = 0.00006;

  // Stage 2 gearing (4 inch diameter wheel)
  private static final double DRIVE_METERS_PER_ROTATION =
      (1.0 / 6.75) * Math.PI * Units.inchesToMeters(4.0);
  private static final double ROTATION_DEGREES_PER_ROTATION = (1.0 / 12.8) * 360.0;

  // M/s - Tune (Apply 12V and measure max vel. Adjust KV/KA for sim if needed)
  public static final double DRIVE_MAX_VEL = 4.5;

  private static final double DRIVE_KP = 1.0; // Tune
  private static final double DRIVE_KD = 0.0; // Tune

  private static final double ROTATION_KP = 8.0; // Tune
  private static final double ROTATION_KD = 4.0; // Tune

  private final LinearSystemSim<N1, N1, N1> driveSim;
  private final LinearSystemSim<N2, N1, N1> rotationSim;

  private final TalonFX driveMotor;
  private final TalonFXConfiguration driveConfig;
  private final TalonFXSimState driveSimState;

  private final TalonFX rotationMotor;
  private final TalonFXConfiguration rotationConfig;
  private final TalonFXSimState rotationSimState;

  private final CANcoder rotationEncoder;
  private final CANcoderConfiguration rotationEncoderConfig;
  private final CANcoderSimState rotationEncoderSimState;

  // Use boolean class so it can be null
  private static Boolean useFOC = null;

  /**
   * Create a Mk4 swerve module
   *
   * @param moduleCode The code representing this module
   * @param driveMotorCanID The CAN ID of the drive motor
   * @param rotationMotorCanID The CAN ID of the rotation motor
   * @param encoderCanID The CAN ID of the rotation CANCoder
   * @param canBus The name of the can bus the devices are connected to.
   */
  public Mk4SwerveModulePro(
      ModuleCode moduleCode,
      int driveMotorCanID,
      int rotationMotorCanID,
      int encoderCanID,
      String canBus) {
    super(moduleCode.name() + "SwerveModule");

    driveMotor = new TalonFX(driveMotorCanID, canBus);
    driveConfig = new TalonFXConfiguration();
    driveConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    driveConfig.Slot0.kP = DRIVE_KP;
    driveConfig.Slot0.kD = DRIVE_KD;
    driveConfig.Slot0.kV = 12.0 / (DRIVE_MAX_VEL / DRIVE_METERS_PER_ROTATION);
    driveConfig.Slot0.PeakOutput = 12.0;
    driveMotor.getConfigurator().apply(driveConfig);
    driveSimState = driveMotor.getSimState();

    rotationMotor = new TalonFX(rotationMotorCanID, canBus);
    rotationConfig = new TalonFXConfiguration();
    rotationConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    rotationConfig.Slot0.kP = ROTATION_KP;
    rotationConfig.Slot0.kD = ROTATION_KD;
    rotationConfig.Slot0.PeakOutput = 12.0;
    rotationMotor.getConfigurator().apply(rotationConfig);
    rotationSimState = rotationMotor.getSimState();

    rotationEncoder = new CANcoder(encoderCanID, canBus);
    rotationEncoderConfig = new CANcoderConfiguration();
    rotationEncoderConfig.MagnetSensor.AbsoluteSensorRange =
        CANcoder_AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    rotationEncoderConfig.MagnetSensor.SensorDirection =
        CANcoder_SensorDirectionValue.CounterClockwise_Positive;
    // Phoenix pro has built in functionality for handling rotation offsets.
    rotationEncoderConfig.MagnetSensor.MagnetOffset =
        Preferences.getDouble(getName() + "RotationOffset", 0.0) / 360.0;
    rotationEncoder.getConfigurator().apply(rotationEncoderConfig);
    rotationEncoderSimState = rotationEncoder.getSimState();

    driveSim = new LinearSystemSim<>(LinearSystemId.identifyVelocitySystem(DRIVE_KV, DRIVE_KA));
    rotationSim =
        new LinearSystemSim<>(LinearSystemId.identifyPositionSystem(ROTATION_KV, ROTATION_KA));

    // Only create one listener since there is no need to have 4 of them doing the exact same thing
    if (useFOC == null) {
      useFOC = Preferences.getBoolean("SwerveUseFOC", true);
      // Set the value in preferences to make sure it shows up on SD even if it was never changed
      Preferences.setBoolean("SwerveUseFOC", useFOC);

      // Does not need to be closed because the SwerveModule class will exist for the lifetime of
      // robot program
      BooleanSubscriber focPrefSubscriber =
          NetworkTableInstance.getDefault()
              .getBooleanTopic("Preferences/SwerveUseFOC")
              .subscribe(true);
      NetworkTableInstance.getDefault()
          .addListener(
              focPrefSubscriber,
              EnumSet.of(NetworkTableEvent.Kind.kValueAll),
              event -> useFOC = event.valueData.value.getBoolean());
    }

    syncRotationEncoders();

    registerHardware("Drive Motor", driveMotor);
    registerHardware("Rotation Motor", rotationMotor);
    registerHardware("Rotation Encoder", rotationEncoder);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber(getName() + "/Vel", getDriveVelocityMetersPerSecond());
    SmartDashboard.putNumber(getName() + "/Rotation", getRelativeRotationDegrees());
    SmartDashboard.putNumber(getName() + "/RotationAbsolute", getAbsoluteRotationDegrees());
    SmartDashboard.putNumber(getName() + "/DriveTemp", driveMotor.getDeviceTemp().getValue());
    SmartDashboard.putNumber(getName() + "/RotationTemp", rotationMotor.getDeviceTemp().getValue());

    reportPowerUsage(
        "Drive Motor",
        driveMotor.getSupplyCurrent().getValue(),
        driveMotor.getSupplyVoltage().getValue());
    reportPowerUsage(
        "Rotation Motor",
        rotationMotor.getSupplyCurrent().getValue(),
        rotationMotor.getSupplyVoltage().getValue());
  }

  @Override
  public void simulationPeriodic() {
    driveSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
    rotationSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
    rotationEncoderSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

    driveSim.setInput(driveSimState.getMotorVoltage());
    rotationSim.setInput(rotationSimState.getMotorVoltage());

    driveSim.update(0.02);
    rotationSim.update(0.02);

    double driveVel = driveSim.getOutput(0) / DRIVE_METERS_PER_ROTATION;
    driveSimState.setRotorVelocity(driveVel);
    driveSimState.addRotorPosition(driveVel * 0.02);

    double rotationPos = rotationSim.getOutput(0) / ROTATION_DEGREES_PER_ROTATION;
    double rotationDeltaPos = rotationPos - rotationMotor.getPosition().getValue();
    rotationSimState.addRotorPosition(rotationDeltaPos);
    rotationSimState.setRotorVelocity(rotationDeltaPos / 0.02);
    rotationEncoderSimState.setRawPosition(rotationSim.getOutput(0) / 360);
  }

  /**
   * Set the desired state of this module
   *
   * @param desiredState Desired state of the module
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Don't run the motors if the desired speed is less than 5% of the max
    if (Math.abs(desiredState.speedMetersPerSecond) < DRIVE_MAX_VEL * 0.05) {
      stopMotors();
      return;
    }

    SwerveModuleState targetState = SwerveModuleState.optimize(desiredState, getState().angle);

    double deltaRot = targetState.angle.getDegrees() - getAbsoluteRotationDegrees();
    if (deltaRot > 180) {
      deltaRot -= 360;
    } else if (deltaRot < -180) {
      deltaRot += 360;
    }
    double targetAngle = getRelativeRotationDegrees() + deltaRot;

    StatusCode driveStatus =
        driveMotor.setControl(
            new VelocityVoltage(
                targetState.speedMetersPerSecond / DRIVE_METERS_PER_ROTATION,
                // Allow enabling FOC from dashboard. May need different PID constants
                useFOC,
                0.0,
                0));
    if (!driveStatus.isOK()) {
      addFault(
          "[Drive Motor]: Status code: "
              + driveStatus.getName()
              + ". "
              + driveStatus.getDescription(),
          driveStatus.isWarning());
    }

    StatusCode rotationStatus =
        rotationMotor.setControl(
            new PositionVoltage(
                targetAngle / ROTATION_DEGREES_PER_ROTATION,
                // Allow enabling FOC from dashboard. May need different PID constants
                useFOC,
                0.0,
                0));
    if (!rotationStatus.isOK()) {
      addFault(
          "[Rotation Motor]: Status code: "
              + rotationStatus.getName()
              + ". "
              + rotationStatus.getDescription(),
          rotationStatus.isWarning());
    }
  }

  /** Stop all motors */
  public void stopMotors() {
    // NeutralOut request for coast mode
    driveMotor.setControl(new NeutralOut());
    rotationMotor.setControl(new NeutralOut());
  }

  /**
   * Get the position of the drive motor in meters
   *
   * @return How far this module has driven in meters
   */
  public double getDrivePositionMeters() {
    return driveMotor.getPosition().getValue() * DRIVE_METERS_PER_ROTATION;
  }

  /**
   * Get the relative rotation of this module in degrees.
   *
   * @return Relative rotation
   */
  public double getRelativeRotationDegrees() {
    return rotationMotor.getPosition().getValue() * ROTATION_DEGREES_PER_ROTATION;
  }

  /**
   * Get the absolute rotation of this module in degrees.
   *
   * @return Absolute rotation
   */
  public double getAbsoluteRotationDegrees() {
    return (rotationEncoder.getAbsolutePosition().getValue() * 360.0);
  }

  /**
   * Get the velocity of the drive motor in meters/sec
   *
   * @return Drive motor velocity
   */
  public double getDriveVelocityMetersPerSecond() {
    return driveMotor.getVelocity().getValue() * DRIVE_METERS_PER_ROTATION;
  }

  /**
   * Get the velocity of the rotation motor in deg/sec
   *
   * @return Rotation motor velocity
   */
  public double getRotationVelocityDegreesPerSecond() {
    return rotationMotor.getVelocity().getValue() * ROTATION_DEGREES_PER_ROTATION;
  }

  /**
   * Get the rotation of this module as a Rotation2d
   *
   * @return Rotation2d for this module
   */
  public Rotation2d getRotation() {
    return Rotation2d.fromDegrees(getRelativeRotationDegrees());
  }

  /**
   * Get the current state of this module
   *
   * @return Current SwerveModuleState
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveVelocityMetersPerSecond(), getRotation());
  }

  /**
   * Get the current positions of this module
   *
   * @return Current SwerveModulePosition
   */
  public SwerveModulePosition getPositions() {
    return new SwerveModulePosition(getDrivePositionMeters(), getRotation());
  }

  /**
   * Update the rotation offset for this module. This will assume that the current module position
   * should be the new zero.
   */
  public void updateRotationOffset() {
    double currentOffset = rotationEncoderConfig.MagnetSensor.MagnetOffset;
    double offset = (currentOffset - rotationEncoder.getAbsolutePosition().getValue()) % 1.0;
    Preferences.setDouble(getName() + "RotationOffset", offset * 360.0);
    rotationEncoderConfig.MagnetSensor.MagnetOffset = offset;
    rotationEncoder.getConfigurator().apply(rotationEncoderConfig);
    syncRotationEncoders();
  }

  /** Sync the relative rotation encoder (falcon) to the value of the absolute encoder (CANCoder) */
  public void syncRotationEncoders() {
    rotationMotor.setRotorPosition(
        (rotationEncoder.getAbsolutePosition().getValue() * 360.0) / ROTATION_DEGREES_PER_ROTATION);
    rotationSimState.setRawRotorPosition(
        (rotationEncoder.getAbsolutePosition().getValue() * 360.0) / ROTATION_DEGREES_PER_ROTATION);
  }

  @Override
  public CommandBase systemCheckCommand() {
    return Commands.sequence(
            Commands.runOnce(
                () -> {
                  clearFaults();
                  driveMotor.stopMotor();
                  rotationMotor.set(0.3);
                },
                this),
            Commands.waitSeconds(1.0),
            Commands.runOnce(
                () -> {
                  if (rotationMotor.getSupplyCurrent().getValue() < 5) {
                    addFault("[System Check] Rotation motor current draw too low");
                  }
                  if (getRotationVelocityDegreesPerSecond() < 20) {
                    addFault("[System Check] Rotation motor encoder velocity too slow");
                  }
                  if (rotationEncoder.getVelocity().getValue() * ROTATION_DEGREES_PER_ROTATION
                      < 20) {
                    addFault("[System Check] Absolute encoder velocity too slow");
                  }
                },
                this),
            Commands.runOnce(
                () -> {
                  driveMotor.set(0.1);
                  rotationMotor.stopMotor();
                },
                this),
            Commands.waitSeconds(0.5),
            Commands.runOnce(
                () -> {
                  if (driveMotor.getSupplyCurrent().getValue() < 5) {
                    addFault("[System Check] Rotation motor current draw too low");
                  }
                  if (getDriveVelocityMetersPerSecond() < 0.25) {
                    addFault("[System Check] Drive motor encoder velocity too slow");
                  }
                },
                this),
            Commands.runOnce(
                () -> {
                  driveMotor.stopMotor();
                  rotationMotor.setControl(new PositionVoltage(0.0, useFOC, 0.0, 0));
                },
                this),
            Commands.waitSeconds(1.0),
            Commands.runOnce(
                () -> {
                  if (Math.abs(getState().angle.getDegrees()) > 10) {
                    addFault("[System Check] Rotation did not reach target position");
                  }
                },
                this))
        .until(() -> getFaults().size() > 0);
  }
}
