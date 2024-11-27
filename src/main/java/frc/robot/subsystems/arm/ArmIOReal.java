package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class ArmIOReal implements ArmIO {

  // Changed gear ratio from 100 --> 90 * 100 / 130 as 130 was true 90 degrees
  private static final double GEAR_RATIO = (90 * 100) / 130;
  private final CANSparkMax armMotor = new CANSparkMax(9, MotorType.kBrushless);
  private final RelativeEncoder armEncoder = armMotor.getEncoder();
  private double armSetpointPosition = armEncoder.getPosition();

  // Private helper functions
  private void motorsetup(CANSparkMax motor) {
    // Setup Settings for motors
    motor.restoreFactoryDefaults();
    motor.setCANTimeout(250);
    motor.enableVoltageCompensation(12.0);
    motor.setSmartCurrentLimit(4);
    motor.setIdleMode(CANSparkBase.IdleMode.kBrake);
    motor.setInverted(true);

    // Recommended by REV in order to ensure that new settings are not lost
    // during a brown-out scenario where the Spark Max loses power but the
    // RoboRio does not
    motor.burnFlash();
  }

  // This gets the motors current position in Degrees
  private double getPosition() {
    return Units.rotationsToDegrees(armEncoder.getPosition() / GEAR_RATIO);
  }

  // This gets the motors current distance away from the goal position (AKA Error)
  private double getError() {
    return this.armSetpointPosition - this.getPosition();
  }

  // Class function
  public ArmIOReal() {
    motorsetup(armMotor);
  }

  // Override functions
  // Update inputs for logger
  @Override
  public void updateInputs(ArmIOInputs inputs) {
    inputs.armPositionDeg = getPosition();
    inputs.armErrorDeg = getError();
    inputs.armVelocityDegPerSec =
        (Units.rotationsToDegrees(armEncoder.getVelocity() / GEAR_RATIO)) / 60;
    inputs.armSetpointPosition = this.armSetpointPosition;
    inputs.armAppliedVolts = armMotor.getAppliedOutput() * armMotor.getBusVoltage();
    inputs.armCurrentAmps = armMotor.getOutputCurrent();
  }

  // Sets the goal position for the bangbang controller
  @Override
  public void setPosition(double position) {
    this.armSetpointPosition = position;
  }

  // Updates periodically, lets use this to move the arm!
  @Override
  public void periodic() {
    /* Since the WPILib bang bang controller only operates in the forwards direction, let's roll our own */
    double error = getError();
    if (error > Constants.armDeadZone) {
      armMotor.setVoltage(Constants.armAppliedVolts);
    } else if (error < -Constants.armDeadZone) {
      armMotor.setVoltage(-Constants.armAppliedVolts);
    } else {
      armMotor.setVoltage(0);
    }
  }
}
