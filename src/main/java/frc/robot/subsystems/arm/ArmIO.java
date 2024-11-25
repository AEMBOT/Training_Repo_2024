package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {

  @AutoLog
  public static class ArmIOInputs {
    public double armPositionDeg = 0.0;
    public double armErrorDeg = 0.0;
    public double armVelocityDegPerSec = 0.0;
    public double armAppliedVolts = 0.0;
    public double armCurrentAmps = 0.0;
    public double armGoalPosition = 0.0;
    public double armSetpointPosition = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ArmIOInputs inputs) {}

  public default void setPosition(double position) {}

  /** Run open loop at the specified voltage. */
  public default void setVoltage(double volts) {}

  public default void periodic() {}
}
