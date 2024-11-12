package frc.robot.subsystems.arm;

import static edu.wpi.first.math.MathUtil.clamp;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkBase;
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


    // Private helper functions
    private void motorsetup(CANSparkMax motor){
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

    private double getPositionRads(){
        return Units.rotationsToRadians(armEncoder.getPosition() / GEAR_RATIO);
    }

    // Class function
    public ArmIOReal() {
        
        motorsetup(armMotor);

    }

    // Override functions

    @Override
    public void updateInputs(ArmIOInputs inputs){
        inputs.armPositionRad = Units.rotationsToRadians(armEncoder.getPosition() / GEAR_RATIO);
        inputs.armVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(armEncoder.getVelocity() / GEAR_RATIO);
        inputs.armSetpointPosition = 90.0;
        inputs.armAppliedVolts = armMotor.getAppliedOutput() * armMotor.getBusVoltage();
        inputs.armCurrentAmps = armMotor.getOutputCurrent();
    }

    @Override
    public void setPosition(double SetPosition){
        bangController.setSetpoint(SetPosition);
    }

      /** Run open loop at the specified voltage. */
    @Override
    public void setVoltage(double volts) {
        setMotorVoltage(volts);
    }

    private void setMotorVoltage(double volts) {
        if (getPositionRads() < 0.0) {
            volts = clamp(volts, -1, Double.MAX_VALUE);
        }
        if (getPositionRads() > 180.0) {
            volts = clamp(volts, -Double.MAX_VALUE, 1);
        }

        armMotor.setVoltage(volts);
    }

    @Override
    public void periodic() {
        /*I recognize this might not be directly how bangController.calculate() is working, but I understand if it returns 1
        then it is below the setpoint (EX: setpoint == 75* and current position == 15*), if it returns a 0, then setpoint is either AT point or above
        (EX: setpoint == 75* and current position == 90*) However, am unsure how to actually tell if the motor has hit the correct
        point without it being 0, as it will return 0 if it is correctly set (within degree of error)
         */

        // If the motor is above (Return of bangController == 0), make the motor turn backwards until not 0
        if (bangController.calculate(getPositionRads()) == 0) {
            while(bangController.calculate(getPositionRads()) == 0){
                armMotor.set(-1);
            }
        // If the motor is below (Return of bangController == 1), make the motor turn Forward until not 1
        } else{
            while(bangController.calculate(getPositionRads()) == 1){
                armMotor.set(1);
            }
        }
    }


}
