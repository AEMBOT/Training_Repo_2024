package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {

  private final ArmIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

  private final SysIdRoutine sysId;

  public Arm(ArmIO io) {
    this.io = io;

    // Configure SysId
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                Volts.of(4),
                null,
                (state) -> Logger.recordOutput("Arm/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism((voltage) -> runVolts(voltage.in(Volts)), null, this));
  }

  @Override
  public void periodic() {
    io.periodic();
    io.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);
  }

  public void setPosition(double position) {
    io.setPosition(position);
  }

  private void runVolts(double volts) {
    io.setVoltage(volts);
  }
  // This creates a command style request for the scheduler to set goal positions
  public Command setPositionCommand(DoubleSupplier posDeg) {
    return run(() -> runPosition(posDeg.getAsDouble()));
  }

  // This will log the new goal position and set the position
  public void runPosition(double positionDeg) {
    Logger.recordOutput("Arm/GoalDeg", positionDeg);
    io.setPosition(positionDeg);
  }
  
  // SYSID commands
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
  }
}
