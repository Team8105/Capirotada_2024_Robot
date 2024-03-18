// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    // drive motors
    public static final int kLeftMotor1Port = 0;
    public static final int kLeftMotor2Port = 1;
    public static final int kRightMotor1Port = 2;
    public static final int kRightMotor2Port = 3;

    // Trepate Mija solenoid
    // public static final int KTrepateSoleoind = 2;

    public static final int[] kLeftEncoderPorts = new int[] { 0, 1 };
    public static final int[] kRightEncoderPorts = new int[] { 2, 3 };
    public static final boolean kLeftEncoderReversed = false;
    public static final boolean kRightEncoderReversed = true;

    public static final double kTrackwidthMeters = 0.69;
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
        kTrackwidthMeters);

    public static final int kEncoderCPR = 2048; //Señales por revolucion
    public static final double kWheelDiameterMeters = 0.15; //Ancho de la llanta
    //calculo para saber la distancia por señal
    public static final double kEncoderDistancePerPulse =
        // Asumiendo que el encoder esta en la transmicion
        (kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR;

    public static final boolean kGyroReversed = true;

    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or
    // theoretically
    // for *your* robot's drive.
    // The Robot Characterization Toolsuite provides a convenient tool for obtaining
    // these
    // values for your robot.
    public static final double ksVolts = 0.22;
    public static final double kvVoltSecondsPerMeter = 1.98;
    public static final double kaVoltSecondsSquaredPerMeter = 0.2;

    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or

    // theoretically
    // for *your* robot's drive.
    // These two values are "angular" kV and kA
    public static final double kvVoltSecondsPerRadian = 1.5;
    public static final double kaVoltSecondsSquaredPerRadian = 0.3;

    public static final LinearSystem<N2, N2, N2> kDrivetrainPlant = LinearSystemId.identifyDrivetrainSystem(
        kvVoltSecondsPerMeter,
        kaVoltSecondsSquaredPerMeter,
        kvVoltSecondsPerRadian,
        kaVoltSecondsSquaredPerRadian);

    // Example values only -- use what's on your physical robot!
    //Tipos de motores de cada lado del chasis
    public static final DCMotor kDriveGearbox = DCMotor.getCIM(2);
    //Reduccion del motor
    public static final double kDriveGearing = 8;

    // Example value only - as above, this must be tuned for your drive!
    public static final double kPDriveVel = 0.5;
  }

  public static final class ClimberConstants {
    // Id del solenoide del trepador
    public static final int KClimberSolenoid = 1;
  }

  public static final class ShooterConstants {
    // Ids de los motores del shooter
    public static final int KShooterLeftMotor = 4;
    public static final int KShooterRightMotor = 6;

    //Velocidad de los motores de los shooter (del -1 al 1)
    public static final double KShooterMotorSpeed = 1.00;
  }

  public static final class ConveyorConstants {
    // Id del motor del conveyor
    public static final int KConveyorMotor = 5;

    //Potencia del motor del conveyor (del -1 al 1)
    public static final double KConveyorMotorSpeed = 1.00;
  }

  public static final class IntakeConstants {
    // Id de los motores 
    public static final int KIntakeBigMotor = 10;
    public static final int KIntakeSmallMotor = 9;

    //Potencia de los motores del Intake
    public static final double KBigMotorSpeed = 1.00;
    public static final double KSmallMotorSpeed = 1.00;
  }


  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;

      // Reasonable baseline values for a RAMSETE follower in units of meters and
    // seconds
    public static final double kRamseteB = 1.5;
    public static final double kRamseteZeta = 1.6;
    //este valor es para le velocidad lineal del autonomo
    public static final double kPXController = 1.0576;
    public static double kIXController = 0.00019;
    public static double kDXController = 0.00019;

    //este valor es para los giros
    public static final double kPYawController = 0.6219;
    public static final double KIYawController = 0.00018;
    public static double kDYawController = 0.000128;

    public static final double kPTurnController = 0.12;
    public static final double KITurnController = 0.000009;
    public static double kDTurnController = 0.017;
  }

  public static final class FieldConstants {
    public static final Pose2d BLUE_SUB_WOOFER = new Pose2d(new Translation2d(0, 5.60),
        new Rotation2d(Math.toRadians(180)));
    public static final Pose2d RED_SUB_WOOFER = new Pose2d(new Translation2d(16.5, 5.60),
        new Rotation2d(Math.toRadians(0)));
  }
}
