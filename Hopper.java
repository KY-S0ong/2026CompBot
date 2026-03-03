package frc.robot.subsystems.structures;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hopper extends SubsystemBase {

  private SparkMax hopperMotor = new SparkMax(60, MotorType.kBrushless);

  // Limits
  private double minPosition = 0.0; // PLZ CHANGE THIS IS NOT RIGHT
  private double maxPosition = 100.0; // PLZ CHANGE THIS IS NOT RIGHT

  // Preset
  private double closedPosition = 5.0; // PLZ CHANGE THIS IS NOT RIGHT
  private double openPosition = 85.0; // PLZ CHANGE THIS IS NOT RIGHT

  public Hopper() {
    hopperMotor.setInverted(false);
    hopperMotor.getEncoder().setPosition(0);
  }

  @Override
  public void periodic() {
    updateDashboard();
  }

  public void openHopper() {
    moveToPosition(openPosition);
  }

  public void closeHopper() {
    moveToPosition(closedPosition);
  }

  public void moveToPosition(double target) {
    double current = getPosition();

    if (target > current && current < maxPosition) {
      hopperMotor.set(0.4);
    } else if (target < current && current > minPosition) {
      hopperMotor.set(-0.4);
    } else {
      hopperMotor.set(0);
    }
  }

  public void manualControl(double speed) {
    if ((speed > 0 && getPosition() >= maxPosition)
        || (speed < 0 && getPosition() <= minPosition)) {
      hopperMotor.set(0);
    } else {
      hopperMotor.set(speed);
    }
  }

  // Voltage override
  public void setHopperVolts(double volts) {
    hopperMotor.setVoltage(volts);
  }

  public void stop() {
    hopperMotor.set(0);
  }

  public double getPosition() {
    return hopperMotor.getEncoder().getPosition();
  }

  public void zeroEncoder() {
    hopperMotor.getEncoder().setPosition(0);
  }

  private void updateDashboard() {
    SmartDashboard.putNumber("Hopper Position", getPosition());
  }
}
