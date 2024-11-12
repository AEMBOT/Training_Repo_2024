package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.util.Units;

public class ArmIOReal implements ArmIO {
    // TODO: Gear ratio of the arm?
    private static final double GEAR_RATIO = 0.0;
    // TODO: Confirm CANID of armmotor
    private final CANSparkMax armMotor = new CANSparkMax(11, MotorType.kBrushless);
    private final RelativeEncoder armEncoder = armMotor.getEncoder();
    

    private final BangBangController bangController = new BangBangController();

    private void motorsetup(CANSparkMax motor){
        // Setup Settings for motors
        motor.restoreFactoryDefaults();
        motor.setCANTimeout(250);
        motor.enableVoltageCompensation(12.0);
        motor.setSmartCurrentLimit(20);

        // Recommended by REV in order to ensure that new settings are not lost
        // during a brown-out scenario where the Spark Max loses power but the
        // RoboRio does not
        motor.burnFlash();
    }

    private double getPositionRads(){
        return Units.rotationsToRadians(armEncoder.getPosition() / GEAR_RATIO);
    }

    public ArmIOReal() {
        
        motorsetup(armMotor);

    }

    @Override
    public void updateInputs(ArmIOInputs inputs){
        inputs.PositionRad = Units.rotationsToRadians(armEncoder.getPosition() / GEAR_RATIO);
        inputs.VelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(armEncoder.getVelocity() / GEAR_RATIO);
        
        inputs.AppliedVolts = armMotor.getAppliedOutput() * armMotor.getBusVoltage();
        inputs.CurrentAmps = armMotor.getOutputCurrent();
    }

    @Override
    public void setPosition(double PositionRad){
        /*I recognize this might not be directly how bangController.calculate() is working, but I understand if it returns 1
        then it is below the setpoint (EX: setpoint == 75* and current position == 15*), if it returns a 0, then setpoint is either AT point or above
        (EX: setpoint == 75* and current position == 90*) However, am unsure how to actually tell if the motor has hit the correct
        point without it being 0, as it will return 0 if it is correctly set (within degree of error)
         */

        // If the motor is above (Return of bangController == 0), make the motor turn backwards until not 0
        if (bangController.calculate(getPositionRads(), PositionRad) == 0) {
            while(bangController.calculate(getPositionRads(), PositionRad) == 0){
                armMotor.set(-1);
            }
        // If the motor is below (Return of bangController == 1), make the motor turn Forward until not 1
        } else{
            while(bangController.calculate(getPositionRads(), PositionRad) == 1){
                armMotor.set(1);
            }
        }
    }



}
