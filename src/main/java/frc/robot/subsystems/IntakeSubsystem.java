// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  private final WPI_VictorSPX BigRollMotor;
  private final WPI_VictorSPX SmallRollMotor;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    BigRollMotor = new WPI_VictorSPX(IntakeConstants.KIntakeBigMotor);
    SmallRollMotor = new WPI_VictorSPX(IntakeConstants.KIntakeSmallMotor);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler 
    SmartDashboard.putBoolean("Intake", BigRollMotor.get() > 0.1 || BigRollMotor.get() < -0.1);
  }

  public void setPosition(boolean open) {
    if (open) {
      BigRollMotor.set(IntakeConstants.KBigMotorSpeed);
      SmallRollMotor.set(IntakeConstants.KSmallMotorSpeed);
    } else {
      BigRollMotor.set(-IntakeConstants.KBigMotorSpeed);
      SmallRollMotor.set(-IntakeConstants.KSmallMotorSpeed);
    }
  }

  public void stopIntake(){
    BigRollMotor.stopMotor();
    SmallRollMotor.stopMotor();
  }
}
