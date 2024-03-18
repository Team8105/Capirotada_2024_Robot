// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ConveyorConstants;

public class ConveyorSubsystem extends SubsystemBase {
  private final WPI_VictorSPX conveyorMotor;

  /** Creates a new ConveyorSubsystem. */
  public ConveyorSubsystem() {
    conveyorMotor =  new WPI_VictorSPX(Constants.ConveyorConstants.KConveyorMotor);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setPosition(boolean upConveyor) {
    if (upConveyor) {
      conveyorMotor.set(-ConveyorConstants.KConveyorMotorSpeed);
    } else {
      conveyorMotor.set(ConveyorConstants.KConveyorMotorSpeed);
    }
  }

  public void stopMotors() {
    conveyorMotor.stopMotor();
  }
}
