package frc.lib.swerve;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.*;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.subsystem.AdvancedSubsystem;

/** Implementation for an SDS Mk4 swerve module using Falcon 500s */
public class Mk4SwerveModule extends AdvancedSubsystem {
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
  private static final double DRIVE_METERS_PER_PULSE =
      (1.0 / 6.75) * Math.PI * Units.inchesToMeters(4.0) / 2048.0;
  private static final double ROTATION_DEGREES_PER_PULSE = (1.0 / 12.8) * 360.0 / 2048.0;

  // M/s - Tune (Apply 12V and measure max vel. Adjust KV/KA for sim if needed)
  private static final double DRIVE_MAX_VEL = 4.5;

  private static final double DRIVE_KP = 10.0; // Tune
  private static final double DRIVE_KD = 0.0; // Tune
  private static final double DRIVE_KF = 1023.0 / (DRIVE_MAX_VEL / DRIVE_METERS_PER_PULSE / 10.0);

  private static final double ROTATION_KP = 0.2;
  private static final double ROTATION_KD = 0.1;

  private final LinearSystemSim<N1, N1, N1> driveSim;
  private final LinearSystemSim<N2, N1, N1> rotationSim;

  private final WPI_TalonFX driveMotor;
  private final TalonFXSimCollection driveMotorSim;

  private final WPI_TalonFX rotationMotor;
  private final TalonFXSimCollection rotationMotorSim;

  private final WPI_CANCoder rotationEncoder;
  private final CANCoderSimCollection rotationEncoderSim;

  private double rotationOffset;

  /**
   * Create a Mk4 swerve module
   *
   * @param moduleCode The code representing this module
   * @param driveMotorCanID The CAN ID of the drive motor
   * @param rotationMotorCanID The CAN ID of the rotation motor
   * @param encoderCanID The CAN ID of the rotation CANCoder
   * @param canBus The name of the can bus the devices are connected to.
   */
  public Mk4SwerveModule(
      ModuleCode moduleCode,
      int driveMotorCanID,
      int rotationMotorCanID,
      int encoderCanID,
      String canBus) {
    super(moduleCode.name() + "SwerveModule");

    rotationOffset = Preferences.getDouble(getName() + "RotationOffset", 0.0);

    driveMotor = new WPI_TalonFX(driveMotorCanID, canBus);
    TalonFXConfiguration driveConfig = new TalonFXConfiguration();
    driveConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;
    driveConfig.slot0.kP = DRIVE_KP;
    driveConfig.slot0.kD = DRIVE_KD;
    driveConfig.slot0.kF = DRIVE_KF;
    driveMotor.configAllSettings(driveConfig);
    driveMotor.setInverted(TalonFXInvertType.CounterClockwise);
    driveMotor.setNeutralMode(NeutralMode.Coast);
    driveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    driveMotorSim = driveMotor.getSimCollection();

    rotationMotor = new WPI_TalonFX(rotationMotorCanID, canBus);
    rotationMotor.configFactoryDefault();
    TalonFXConfiguration rotationConfig = new TalonFXConfiguration();
    rotationConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;
    rotationConfig.slot0.kP = ROTATION_KP;
    rotationConfig.slot0.kD = ROTATION_KD;
    rotationMotor.configAllSettings(rotationConfig);
    rotationMotor.setInverted(TalonFXInvertType.CounterClockwise);
    rotationMotor.setNeutralMode(NeutralMode.Coast);
    rotationMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    rotationMotorSim = rotationMotor.getSimCollection();

    rotationEncoder = new WPI_CANCoder(encoderCanID, canBus);
    rotationEncoder.configFactoryDefault();
    CANCoderConfiguration angleEncoderConfig = new CANCoderConfiguration();
    angleEncoderConfig.absoluteSensorRange = AbsoluteSensorRange.Signed_PlusMinus180;
    angleEncoderConfig.sensorDirection = false;
    angleEncoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
    angleEncoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
    rotationEncoder.configAllSettings(angleEncoderConfig);
    rotationEncoderSim = rotationEncoder.getSimCollection();

    driveSim = new LinearSystemSim<>(LinearSystemId.identifyVelocitySystem(DRIVE_KV, DRIVE_KA));
    rotationSim =
        new LinearSystemSim<>(LinearSystemId.identifyPositionSystem(ROTATION_KV, ROTATION_KA));

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
    SmartDashboard.putNumber(getName() + "/DriveTemp", driveMotor.getTemperature());
    SmartDashboard.putNumber(getName() + "/RotationTemp", rotationMotor.getTemperature());

    reportPowerUsage("Drive Motor", driveMotor.getSupplyCurrent(), driveMotor.getBusVoltage());
    reportPowerUsage(
        "Rotation Motor", rotationMotor.getSupplyCurrent(), rotationMotor.getBusVoltage());
  }

  @Override
  public void simulationPeriodic() {
    driveMotorSim.setBusVoltage(RobotController.getBatteryVoltage());
    rotationMotorSim.setBusVoltage(RobotController.getBatteryVoltage());
    rotationEncoderSim.setBusVoltage(RobotController.getBatteryVoltage());

    driveMotorSim.setSupplyCurrent(Math.abs(driveMotor.get()) * 30.0);
    rotationMotorSim.setSupplyCurrent(Math.abs(rotationMotor.get()) * 30.0);

    driveSim.setInput(driveMotorSim.getMotorOutputLeadVoltage());
    rotationSim.setInput(rotationMotorSim.getMotorOutputLeadVoltage());

    driveSim.update(0.02);
    rotationSim.update(0.02);

    double driveVel = driveSim.getOutput(0) / DRIVE_METERS_PER_PULSE / 10.0;
    driveMotorSim.setIntegratedSensorVelocity((int) driveVel);
    driveMotorSim.addIntegratedSensorPosition((int) (driveVel * 10.0 * 0.02));

    double rotationSimOut = rotationSim.getOutput(0);
    int rotationPos = (int) (rotationSimOut / ROTATION_DEGREES_PER_PULSE);
    double rotationDeltaPos = rotationPos - rotationMotor.getSelectedSensorPosition();
    rotationMotorSim.setIntegratedSensorRawPosition(rotationPos);
    rotationMotorSim.setIntegratedSensorVelocity((int) (rotationDeltaPos / 0.02 / 10.0));

    double newAbsoluteDeg = (rotationSimOut % 360) + rotationOffset;
    if (newAbsoluteDeg > 180) {
      newAbsoluteDeg -= 360;
    } else if (newAbsoluteDeg < -180) {
      newAbsoluteDeg += 360;
    }
    rotationEncoderSim.setRawPosition((int) Math.round((newAbsoluteDeg / 360.0) * 4096.0));
  }

  /**
   * Set the desired state of this module
   *
   * @param desiredState Desired state of the module
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    if (Math.abs(desiredState.speedMetersPerSecond) < DRIVE_MAX_VEL * 0.1) {
      stopMotors();
      return;
    }

    SwerveModuleState targetState = SwerveModuleState.optimize(desiredState, getState().angle);

    driveMotor.set(
        TalonFXControlMode.Velocity,
        targetState.speedMetersPerSecond / DRIVE_METERS_PER_PULSE / 10.0);

    double deltaRot = targetState.angle.getDegrees() - getAbsoluteRotationDegrees();
    if (deltaRot > 180) {
      deltaRot -= 360;
    } else if (deltaRot < -180) {
      deltaRot += 360;
    }
    double targetAngle = getRelativeRotationDegrees() + deltaRot;
    rotationMotor.set(TalonFXControlMode.Position, targetAngle / ROTATION_DEGREES_PER_PULSE);
  }

  /** Stop all motors */
  public void stopMotors() {
    driveMotor.set(TalonFXControlMode.PercentOutput, 0.0);
    rotationMotor.set(TalonFXControlMode.PercentOutput, 0.0);
  }

  /**
   * Get the current state of this module
   *
   * @return Current SwerveModuleState
   */
  public SwerveModuleState getState() {
    Rotation2d angle = Rotation2d.fromDegrees(getRelativeRotationDegrees());
    return new SwerveModuleState(getDriveVelocityMetersPerSecond(), angle);
  }

  /** Sync the relative rotation encoder (falcon) to the value of the absolute encoder (CANCoder) */
  public void syncRotationEncoders() {
    double absolutePosition = getAbsoluteRotationDegrees() / ROTATION_DEGREES_PER_PULSE;
    rotationMotor.setSelectedSensorPosition(absolutePosition);
  }

  /**
   * Get the position of the drive motor in meters
   *
   * @return How far this module has driven in meters
   */
  public double getDrivePositionMeters() {
    return driveMotor.getSelectedSensorPosition() * DRIVE_METERS_PER_PULSE;
  }

  /**
   * Get the relative rotation of this module in degrees.
   *
   * @return Relative rotation
   */
  public double getRelativeRotationDegrees() {
    return rotationMotor.getSelectedSensorPosition() * ROTATION_DEGREES_PER_PULSE;
  }

  /**
   * Get the absolute rotation of this module in degrees.
   *
   * @return Absolute rotation
   */
  public double getAbsoluteRotationDegrees() {
    double rotation = rotationEncoder.getAbsolutePosition() - rotationOffset;
    if (rotation > 180) {
      rotation -= 360;
    } else if (rotation < -180) {
      rotation += 360;
    }
    return rotation;
  }

  /**
   * Get the velocity of the drive motor in meters/sec
   *
   * @return Drive motor velocity
   */
  public double getDriveVelocityMetersPerSecond() {
    return driveMotor.getSelectedSensorVelocity() * DRIVE_METERS_PER_PULSE * 10.0;
  }

  /**
   * Get the velocity of the rotation motor in deg/sec
   *
   * @return Rotation motor velocity
   */
  public double getRotationVelocityDegreesPerSecond() {
    return rotationMotor.getSelectedSensorVelocity() * ROTATION_DEGREES_PER_PULSE * 10.0;
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
    this.rotationOffset += getAbsoluteRotationDegrees();
    Preferences.setDouble(getName() + "RotationOffset", rotationOffset);
    syncRotationEncoders();
  }

  @Override
  public CommandBase systemCheckCommand() {
    return Commands.sequence(
            Commands.runOnce(
                () -> {
                  clearFaults();
                  driveMotor.set(0);
                  rotationMotor.set(0.3);
                },
                this),
            Commands.waitSeconds(1.0),
            Commands.runOnce(
                () -> {
                  if (rotationMotor.getSupplyCurrent() < 5) {
                    addFault("[System Check] Rotation motor current draw too low");
                  }
                  if (getRotationVelocityDegreesPerSecond() < 20) {
                    addFault("[System Check] Rotation motor encoder velocity too slow");
                  }
                  if (rotationEncoder.getVelocity() < 20) {
                    addFault("[System Check] Absolute encoder velocity too slow");
                  }
                },
                this),
            Commands.runOnce(
                () -> {
                  driveMotor.set(0.1);
                  rotationMotor.set(0);
                },
                this),
            Commands.waitSeconds(0.5),
            Commands.runOnce(
                () -> {
                  if (driveMotor.getSupplyCurrent() < 5) {
                    addFault("[System Check] Rotation motor current draw too low");
                  }
                  if (getDriveVelocityMetersPerSecond() < 0.25) {
                    addFault("[System Check] Drive motor encoder velocity too slow");
                  }
                },
                this),
            Commands.runOnce(
                () -> {
                  driveMotor.set(0);
                  rotationMotor.set(ControlMode.Position, 0);
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
