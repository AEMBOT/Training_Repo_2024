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
  private final CANSparkMax leftFrontLeader = new CANSparkMax(5, MotorType.kBrushless);
  private final CANSparkMax leftCenterFollower = new CANSparkMax(6, MotorType.kBrushless);
  private final CANSparkMax leftBackFollower = new CANSparkMax(4, MotorType.kBrushless);
  private final CANSparkMax rightFrontLeader = new CANSparkMax(1, MotorType.kBrushless);
  private final CANSparkMax rightCenterFollower = new CANSparkMax(2, MotorType.kBrushless);
  private final CANSparkMax rightBackFollower = new CANSparkMax(3, MotorType.kBrushless);

  // Set up some references that are better named
  private final CANSparkMax leftLeader = leftFrontLeader;
  private final CANSparkMax rightLeader = rightFrontLeader;
  private final CANSparkMax[] motors = {
    leftFrontLeader,
    leftCenterFollower,
    leftBackFollower,
    rightFrontLeader,
    rightCenterFollower,
    rightBackFollower
  };

  private final RelativeEncoder leftEncoder = leftLeader.getEncoder();
  private final RelativeEncoder rightEncoder = rightLeader.getEncoder();

  public DriveIOSparkMax() {
    for (CANSparkMax motor : motors) {
      motor.restoreFactoryDefaults();
      motor.setCANTimeout(250);
      motor.enableVoltageCompensation(12.0);
      motor.setSmartCurrentLimit(20);
    }

    leftFrontLeader.setInverted(false);
    leftCenterFollower.follow(leftFrontLeader, false);
    leftBackFollower.follow(leftFrontLeader, false);
    rightFrontLeader.setInverted(true);
    rightCenterFollower.follow(rightFrontLeader, false);
    rightBackFollower.follow(rightFrontLeader, false);

    for (CANSparkMax motor : motors) {
      // Recommended by REV in order to ensure that new settings are not lost
      // during a brown-out scenario where the Spark Max loses power but the
      // RoboRio does not
      motor.burnFlash();
    }
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
