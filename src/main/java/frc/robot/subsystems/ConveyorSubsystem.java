// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ConveyorConstants;

public class ConveyorSubsystem extends SubsystemBase {
  private final WPI_VictorSPX leftConveyorMotor;
  private final WPI_VictorSPX rightConveyorMotor;

  /** Creates a new ConveyorSubsystem. */
  public ConveyorSubsystem() {
    leftConveyorMotor = new WPI_VictorSPX(ConveyorConstants.KConveyorLeftMotor);
    rightConveyorMotor = new WPI_VictorSPX(ConveyorConstants.KConveyorRightMotor);
    rightConveyorMotor.setInverted(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setPosition(boolean upConveyor) {
    if (upConveyor) {
      leftConveyorMotor.set(-ConveyorConstants.KConveyorMotorSpeed);
      rightConveyorMotor.set(-ConveyorConstants.KConveyorMotorSpeed);
    } else {
      leftConveyorMotor.set(ConveyorConstants.KConveyorMotorSpeed);
      rightConveyorMotor.set(ConveyorConstants.KConveyorMotorSpeed);
    }
  }

  public void stopMotors() {
    leftConveyorMotor.stopMotor();
    rightConveyorMotor.stopMotor();
  }
}
