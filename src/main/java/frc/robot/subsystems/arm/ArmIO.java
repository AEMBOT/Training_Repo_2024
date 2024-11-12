package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;


public interface ArmIO {

    @AutoLog
    public static class ArmIOInputs {
        public double armPositionRad = 0.0;
        public double armVelocityRadPerSec = 0.0;
        public double armAppliedVolts = 0.0;
        public double armCurrentAmps = 0.0;
        public double armGoalPosition = 0.0;
        public double armSetpointPosition = 0.0;

    }


    /** Updates the set of loggable inputs. */
    public default void updateInputs(ArmIOInputs inputs) {}

    /** Run closed loop at the specified velocity. */
    // public default void setVelocity(
    //     double RadPerSec, double FFVolts) {}

    public default void setPosition(double SetPosition) {}

    /** Run open loop at the specified voltage. */
    public default void setVoltage(double volts) {}

    public default void periodic(){}
}
