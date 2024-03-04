// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SmallIntakeSubsystem;

public class ShootSmallIntake extends Command {
  SmallIntakeSubsystem smallIntakeSubsystem;
  boolean isShoot;

  /** Creates a new ShootSmallIntake. */
  public ShootSmallIntake(SmallIntakeSubsystem smallIntakeSubsystem, boolean isShoot) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.smallIntakeSubsystem = smallIntakeSubsystem;
    addRequirements(smallIntakeSubsystem);
    this.isShoot = isShoot;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Pequeño intake Activo!");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.smallIntakeSubsystem.ActivateSolenoid(true);
    this.smallIntakeSubsystem.Shoot(isShoot);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.smallIntakeSubsystem.ActivateSolenoid(false);
    this.smallIntakeSubsystem.StopMotor();
    System.out.println("Pequeño intake Desactivado!");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
