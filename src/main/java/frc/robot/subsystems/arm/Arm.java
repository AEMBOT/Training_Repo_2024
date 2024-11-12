package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import frc.robot.Constants;
import frc.robot.Constants.Mode;


// TODO: Base subsystem class
public class Arm extends SubsystemBase {


    private final ArmIO io;
    private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

    private final SysIdRoutine sysId;

    public Arm(ArmIO io){
        this.io = io;

        // Configure SysId
        sysId =
            new SysIdRoutine(
                new SysIdRoutine.Config(
                    null,
                    null,
                    null,
                    (state) -> Logger.recordOutput("Arm/SysIdState", state.toString())),
                new SysIdRoutine.Mechanism(
                    (voltage) -> runVolts(voltage.in(Volts)), null, this));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Arm", inputs);
    }


    private void runVolts(double volts) {
        io.setVoltage(volts);
      }
}
