// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SmallIntakeConstants;

public class SmallIntakeSubsystem extends SubsystemBase {
  /** Creates a new SmallIntakeSubsystem. */
  private WPI_VictorSPX shooterMotor;
  private Solenoid intakeSolenoid;

  public SmallIntakeSubsystem() {
    shooterMotor = new WPI_VictorSPX(SmallIntakeConstants.KSmallIntakeMotor);
    intakeSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, SmallIntakeConstants.KSmallIntakeSolenoid);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Intake Active", intakeSolenoid.get());
  }

  public void Shoot(boolean shoot) {
    if (shoot) {
      shooterMotor.set(SmallIntakeConstants.KSmallIntakeMotorSpeed);
    } else {
      shooterMotor.set(-SmallIntakeConstants.KSmallIntakeMotorSpeed);
    }
  }

  public void StopMotor(){
    shooterMotor.stopMotor();
  }

  public void ActivateSolenoid(boolean isActive) {
    intakeSolenoid.set(isActive);
  }

  public void ToggleSoleoind() {
    intakeSolenoid.set(!intakeSolenoid.get());
  }
}
