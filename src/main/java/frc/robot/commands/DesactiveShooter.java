// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class DesactiveShooter extends Command {
  /** Creates a new DesactiveShooter. */
  ShooterSubsystem shooterSubsystem;
  ConveyorSubsystem conveyorSubsystem;

  public DesactiveShooter(ShooterSubsystem shooterSubsystem, ConveyorSubsystem conveyorSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.conveyorSubsystem = conveyorSubsystem;
    addRequirements(conveyorSubsystem);
    this.shooterSubsystem = shooterSubsystem;
    addRequirements(shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    conveyorSubsystem.stopMotors();
    shooterSubsystem.stopMotors();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
