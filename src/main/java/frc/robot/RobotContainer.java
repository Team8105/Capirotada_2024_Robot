// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OIConstants;
import frc.robot.commands.ActivateShooter;
import frc.robot.commands.ClimberToogle;
import frc.robot.commands.DesactiveShooter;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.SlowShooter;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  DriveTrainSubsystem m_robotDrive;
  IntakeSubsystem m_intakeSubsystem;
  ConveyorSubsystem m_conveyorSubsystem;
  ShooterSubsystem m_shooterSubsystem;
  ClimberSubsystem m_climberSubsystem;
  PneumaticsSubsystem m_pneumaticsSubsystem;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    m_robotDrive = new DriveTrainSubsystem();
    m_intakeSubsystem = new IntakeSubsystem();
    m_conveyorSubsystem = new ConveyorSubsystem();
    m_climberSubsystem = new ClimberSubsystem();
    m_pneumaticsSubsystem = new PneumaticsSubsystem();
    m_shooterSubsystem = new ShooterSubsystem();

    NamedCommands.registerCommand("useIntake", new IntakeCommand(m_intakeSubsystem, m_conveyorSubsystem, true));
    NamedCommands.registerCommand("ActivateShooter", new ActivateShooter(m_shooterSubsystem, m_conveyorSubsystem));
    NamedCommands.registerCommand("DesactivateShooter", new DesactiveShooter(m_shooterSubsystem, m_conveyorSubsystem));
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    m_robotDrive.setDefaultCommand(new RunCommand(
        () -> m_robotDrive.arcadeDrive(m_driverController.getRawAxis(3) - m_driverController.getRawAxis(2),
            -m_driverController.getLeftX() * 0.7),
        m_robotDrive));

    // m_pneumaticsSubsystem
    // .setDefaultCommand(new RunCommand(() ->
    // m_pneumaticsSubsystem.setCompressor(true), m_pneumaticsSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is
    // pressed,
    // cancelling on release.
    m_driverController.b().whileTrue(new IntakeCommand(m_intakeSubsystem, m_conveyorSubsystem, true));
    m_driverController.a().whileTrue(new IntakeCommand(m_intakeSubsystem, m_conveyorSubsystem, false));
    m_driverController.rightBumper().whileTrue(new ShooterCommand(m_conveyorSubsystem, m_shooterSubsystem, true));
    m_driverController.leftBumper().whileTrue(new ShooterCommand(m_conveyorSubsystem, m_shooterSubsystem, false));
    m_driverController.povUp().onTrue(new ClimberToogle(m_climberSubsystem));
    m_driverController.rightTrigger().whileTrue(new SlowShooter(m_shooterSubsystem, m_conveyorSubsystem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new PathPlannerAuto("izquierda");
  }
}
