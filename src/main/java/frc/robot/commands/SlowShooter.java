// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class SlowShooter extends Command {
  /** Creates a new SlowShooter. */
  private final ShooterSubsystem shooterSubsystem;
  private final ConveyorSubsystem conveyorSubsystem;
  public SlowShooter(ShooterSubsystem shooterSubsystem, ConveyorSubsystem conveyorSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooterSubsystem = shooterSubsystem;
    addRequirements(shooterSubsystem);
    this.conveyorSubsystem = conveyorSubsystem;
    addRequirements(conveyorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Intake lento iniciado");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    conveyorSubsystem.setPosition(true);
    shooterSubsystem.SetMotorsSpeed(0.20, 0.20);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Intake lento terminado");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
