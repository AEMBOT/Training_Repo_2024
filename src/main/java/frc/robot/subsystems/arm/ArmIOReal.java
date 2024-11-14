package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class ArmIOReal implements ArmIO {
  private static final double GEAR_RATIO = 100.0;
  private final CANSparkMax armMotor = new CANSparkMax(9, MotorType.kBrushless);
  private final RelativeEncoder armEncoder = armMotor.getEncoder();
  private double armSetpointPosition = armEncoder.getPosition();

  // Private helper functions
  private void motorsetup(CANSparkMax motor) {
    // Setup Settings for motors
    motor.restoreFactoryDefaults();
    motor.setCANTimeout(250);
    motor.enableVoltageCompensation(12.0);
    motor.setSmartCurrentLimit(2);
    // TODO: Lets change the idle mode from coast to break!
    motor.setIdleMode(CANSparkBase.IdleMode.kCoast);
    motor.setInverted(true);

    // Recommended by REV in order to ensure that new settings are not lost
    // during a brown-out scenario where the Spark Max loses power but the
    // RoboRio does not
    motor.burnFlash();
  }
  // This gets the motors current position in Radians
  private double getPosition() {
    return Units.rotationsToRadians(armEncoder.getPosition() / GEAR_RATIO);
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
    inputs.armPositionRad = getPosition();
    inputs.armErrorRad = getError();
    inputs.armVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(armEncoder.getVelocity() / GEAR_RATIO);
    inputs.armSetpointPosition = this.armSetpointPosition;
    inputs.armAppliedVolts = armMotor.getAppliedOutput() * armMotor.getBusVoltage();
    inputs.armCurrentAmps = armMotor.getOutputCurrent();
  }

  // Sets the goal position for the bangbang controller
  @Override
  public void setPosition(double position) {
    this.armSetpointPosition = position;
  }

  /** Run open loop at the specified voltage. */
  @Override
  public void setVoltage(double volts) {
    // setMotorVoltage(volts);
    // No-op for now
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
