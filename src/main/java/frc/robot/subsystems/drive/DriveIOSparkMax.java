// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.util.Units;

/**
 * NOTE: To use the Spark Flex / NEO Vortex, replace all instances of "CANSparkMax" with
 * "CANSparkFlex".
 */
public class DriveIOSparkMax implements DriveIO {
  private static final double GEAR_RATIO = 10.0;

  // TODO: CONFIRM CAN IDs
  private final CANSparkMax leftFrontLeader = new CANSparkMax(8, MotorType.kBrushless);
  private final CANSparkMax leftCenterFollower = new CANSparkMax(7, MotorType.kBrushless);
  private final CANSparkMax leftBackFollower = new CANSparkMax(6, MotorType.kBrushless);
  private final CANSparkMax rightFrontLeader = new CANSparkMax(1, MotorType.kBrushless);
  private final CANSparkMax rightCenterFollower = new CANSparkMax(2, MotorType.kBrushless);
  private final CANSparkMax rightBackFollower = new CANSparkMax(3, MotorType.kBrushless);

  // Set up some references that are better named
  private final CANSparkMax leftLeader = leftFrontLeader;
  private final CANSparkMax rightLeader = rightFrontLeader;

  private final RelativeEncoder leftEncoder = leftLeader.getEncoder();
  private final RelativeEncoder rightEncoder = rightLeader.getEncoder();

  public DriveIOSparkMax() {
    leftFrontLeader.restoreFactoryDefaults();
    leftCenterFollower.restoreFactoryDefaults();
    leftBackFollower.restoreFactoryDefaults();
    rightFrontLeader.restoreFactoryDefaults();
    rightCenterFollower.restoreFactoryDefaults();
    rightBackFollower.restoreFactoryDefaults();

    leftFrontLeader.setCANTimeout(250);
    leftCenterFollower.setCANTimeout(250);
    leftBackFollower.setCANTimeout(250);
    rightFrontLeader.setCANTimeout(250);
    rightCenterFollower.setCANTimeout(250);
    rightBackFollower.setCANTimeout(250);

    leftFrontLeader.setInverted(false);
    leftCenterFollower.follow(leftFrontLeader, false);
    leftBackFollower.follow(leftFrontLeader, false);
    rightFrontLeader.setInverted(true);
    rightCenterFollower.follow(leftFrontLeader, false);
    rightBackFollower.follow(leftFrontLeader, false);

    leftLeader.enableVoltageCompensation(12.0);
    rightLeader.enableVoltageCompensation(12.0);
    leftLeader.setSmartCurrentLimit(20);
    rightLeader.setSmartCurrentLimit(20);

    // Shouldn't need to do this, pending email with Rev
    /*
    leftLeader.burnFlash();
    rightLeader.burnFlash();
    */
  }

  @Override
  public void updateInputs(DriveIOInputs inputs) {
    inputs.leftPositionRad = Units.rotationsToRadians(leftEncoder.getPosition() / GEAR_RATIO);
    inputs.leftVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(leftEncoder.getVelocity() / GEAR_RATIO);
    inputs.leftAppliedVolts = leftLeader.getAppliedOutput() * leftLeader.getBusVoltage();
    inputs.leftCurrentAmps =
        new double[] {
          leftLeader.getOutputCurrent(),
          leftCenterFollower.getOutputCurrent(),
          leftBackFollower.getOutputCurrent()
        };

    inputs.rightPositionRad = Units.rotationsToRadians(rightEncoder.getPosition() / GEAR_RATIO);
    inputs.rightVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(rightEncoder.getVelocity() / GEAR_RATIO);
    inputs.rightAppliedVolts = rightLeader.getAppliedOutput() * rightLeader.getBusVoltage();
    inputs.rightCurrentAmps =
        new double[] {
          rightLeader.getOutputCurrent(),
          rightCenterFollower.getOutputCurrent(),
          rightBackFollower.getOutputCurrent()
        };
  }

  @Override
  public void setVoltage(double leftVolts, double rightVolts) {
    leftLeader.setVoltage(leftVolts);
    rightLeader.setVoltage(rightVolts);
  }

  @Override
  public void setVelocity(
      double leftRadPerSec, double rightRadPerSec, double leftFFVolts, double rightFFVolts) {
    // TODO!
  }
}
