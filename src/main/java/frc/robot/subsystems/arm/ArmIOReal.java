package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class ArmIOReal implements ArmIO {
  // TODO: Gear ratio of the arm?
  private static final double GEAR_RATIO = 100.0;
  // TODO: Confirm CANID of armmotor
  private final CANSparkMax armMotor = new CANSparkMax(11, MotorType.kBrushless);
  private final RelativeEncoder armEncoder = armMotor.getEncoder();
  private double armSetpointPosition = armEncoder.getPosition();

  // Private helper functions
  private void motorsetup(CANSparkMax motor) {
    // Setup Settings for motors
    motor.restoreFactoryDefaults();
    motor.setCANTimeout(250);
    motor.enableVoltageCompensation(12.0);
    motor.setSmartCurrentLimit(20);
    motor.setIdleMode(CANSparkBase.IdleMode.kCoast);

    // Recommended by REV in order to ensure that new settings are not lost
    // during a brown-out scenario where the Spark Max loses power but the
    // RoboRio does not
    motor.burnFlash();
  }

  private double getPosition() {
    return Units.rotationsToRadians(armEncoder.getPosition() / GEAR_RATIO);
  }

  // Class function
  public ArmIOReal() {

    motorsetup(armMotor);
  }

  // Override functions

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    inputs.armPositionRad = getPosition();
    inputs.armVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(armEncoder.getVelocity() / GEAR_RATIO);
    inputs.armSetpointPosition = this.armSetpointPosition;
    inputs.armAppliedVolts = armMotor.getAppliedOutput() * armMotor.getBusVoltage();
    inputs.armCurrentAmps = armMotor.getOutputCurrent();
  }

  @Override
  public void setPosition(double SetPosition) {
    this.armSetpointPosition = SetPosition;
  }

  /** Run open loop at the specified voltage. */
  @Override
  public void setVoltage(double volts) {
    // setMotorVoltage(volts);
    // No-op for now
  }

  @Override
  public void periodic() {
    /* Since the WPILib bang bang controller only operates in the forwards direction, let's roll our own */
    double error = this.getPosition() - this.armSetpointPosition;
    if (error > Constants.armDeadZone) {
      armMotor.setVoltage(Constants.armAppliedVolts);
    } else if (error < Constants.armDeadZone) {
      armMotor.setVoltage(-Constants.armAppliedVolts);
    } else {
      armMotor.setVoltage(0);
    }
  }
}
