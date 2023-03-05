package frc.lib.subsystem;

import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenixpro.hardware.CANcoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.subsystem.selfcheck.*;
import frc.robot.Robot;
import java.util.ArrayList;
import java.util.List;

public abstract class AdvancedSubsystem extends SubsystemBase {
  public enum SystemStatus {
    OK,
    WARNING,
    ERROR
  }

  private final List<SubsystemFault> faults = new ArrayList<>();
  private final List<SelfChecking> hardware = new ArrayList<>();
  private final String statusTable;

  public AdvancedSubsystem() {
    this.statusTable = "SystemStatus/" + this.getName();
    SmartDashboard.putData(statusTable + "/SystemCheck", this.systemCheckCommand());

    setupCallbacks();
  }

  public AdvancedSubsystem(String name) {
    this.setName(name);
    this.statusTable = "SystemStatus/" + name;
    SmartDashboard.putData(statusTable + "/SystemCheck", this.systemCheckCommand());

    setupCallbacks();
  }

  private void setupCallbacks() {
    Robot.addPeriodicCallback(this::checkForFaults, 0.25);
    Robot.addPeriodicCallback(this::publishStatus, 1.0);
  }

  private void publishStatus() {
    SmartDashboard.putString(statusTable + "/status", this.getSystemStatus().name());

    String[] faultStrings = new String[this.faults.size()];
    for (int i = 0; i < this.faults.size(); i++) {
      SubsystemFault fault = this.faults.get(i);
      faultStrings[i] = String.format("[%.2f] %s", fault.timestamp, fault.description);
    }
    SmartDashboard.putStringArray(statusTable + "/faults", faultStrings);
  }

  protected void reportPowerUsage(String device, double amps, double volts) {
    BatteryUsage.reportUsage(this.getName(), device, amps, volts);
  }

  protected void addFault(SubsystemFault fault) {
    if (!this.faults.contains(fault)) {
      this.faults.add(fault);
    }
  }

  protected void addFault(String description, boolean isWarning) {
    this.addFault(new SubsystemFault(description, isWarning));
  }

  protected void addFault(String description) {
    this.addFault(description, false);
  }

  public List<SubsystemFault> getFaults() {
    return this.faults;
  }

  public void clearFaults() {
    this.faults.clear();
  }

  public SystemStatus getSystemStatus() {
    if (this.faults.size() > 0) {
      for (SubsystemFault f : this.faults) {
        if (!f.isWarning) {
          return SystemStatus.ERROR;
        }
      }
      return SystemStatus.WARNING;
    }
    return SystemStatus.OK;
  }

  public void registerHardware(String label, BaseMotorController phoenixMotor) {
    hardware.add(new SelfCheckingPhoenixMotor(label, phoenixMotor));
  }

  public void registerHardware(String label, com.ctre.phoenixpro.hardware.TalonFX talon) {
    hardware.add(new SelfCheckingTalonFXPro(label, talon));
  }

  public void registerHardware(String label, PWMMotorController pwmMotor) {
    hardware.add(new SelfCheckingPWMMotor(label, pwmMotor));
  }

  public void registerHardware(String label, com.ctre.phoenix.sensors.Pigeon2 pigeon2) {
    hardware.add(new SelfCheckingPigeon2(label, pigeon2));
  }

  public void registerHardware(String label, com.ctre.phoenixpro.hardware.Pigeon2 pigeon2) {
    hardware.add(new SelfCheckingPigeon2Pro(label, pigeon2));
  }

  public void registerHardware(String label, CANCoder canCoder) {
    hardware.add(new SelfCheckingCANCoder(label, canCoder));
  }

  public void registerHardware(String label, CANcoder canCoder) {
    hardware.add(new SelfCheckingCANCoderPro(label, canCoder));
  }

  // Command to run a full systems check
  public abstract CommandBase systemCheckCommand();

  // Method to check for faults while the robot is operating normally
  private void checkForFaults() {
    if (RobotBase.isReal()) {
      for (SelfChecking device : hardware) {
        for (SubsystemFault fault : device.checkForFaults()) {
          addFault(fault);
        }
      }
    }
  }
}
