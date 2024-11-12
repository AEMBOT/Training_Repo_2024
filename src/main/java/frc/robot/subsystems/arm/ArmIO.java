package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;


public interface ArmIO {

    @AutoLog
    public static class ArmIOInputs {
        public double PositionRad = 0.0;
        public double VelocityRadPerSec = 0.0;
        public double AppliedVolts = 0.0;
        public double CurrentAmps = 0.0;

    }


    /** Updates the set of loggable inputs. */
    public default void updateInputs(ArmIOInputs inputs) {}

    /** Run closed loop at the specified velocity. */
    // public default void setVelocity(
    //     double RadPerSec, double FFVolts) {}

    public default void setPosition(double PositionRad) {}

}
