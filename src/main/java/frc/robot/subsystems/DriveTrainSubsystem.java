// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;

public class DriveTrainSubsystem extends SubsystemBase {
  // Motores del lado derecho del chasis
  private final WPI_TalonSRX m_leftLeader = new WPI_TalonSRX(DriveConstants.kLeftMotor1Port);
  private final WPI_TalonSRX m_leftFollower = new WPI_TalonSRX(DriveConstants.kLeftMotor2Port);

  // Motores del lado izquierdo del chasis
  private final WPI_TalonSRX m_rightLeader = new WPI_TalonSRX(DriveConstants.kRightMotor1Port);
  private final WPI_TalonSRX m_rightFollower = new WPI_TalonSRX(DriveConstants.kRightMotor2Port);

  // Modo de conducciòn del robot
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftLeader::set, m_rightLeader::set);

  // Encoder del lado derecho
  private final Encoder m_leftEncoder = new Encoder(DriveConstants.kLeftEncoderPorts[0],
      DriveConstants.kLeftEncoderPorts[1], DriveConstants.kLeftEncoderReversed);
  // Encoder del lado izquierdo
  private final Encoder m_rightEncoder = new Encoder(DriveConstants.kRightEncoderPorts[0],
      DriveConstants.kRightEncoderPorts[1], DriveConstants.kRightEncoderReversed);

  // giroscopio
  private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);

  // Clases de odometria para saber la posición del robot
  private final DifferentialDriveOdometry m_Odometry;
  private final DifferentialDriveKinematics m_Kinematics = new DifferentialDriveKinematics(
      DriveConstants.kTrackwidthMeters);

  // Controles PID para velocidad y rotacion;
  private final PIDController xPidController = new PIDController(AutoConstants.kPXController,
      AutoConstants.kIXController, AutoConstants.kDXController);
  private final PIDController yawPidController = new PIDController(AutoConstants.kPYawController,
      AutoConstants.KIYawController, AutoConstants.kDYawController);

  // Estas clases nos ayudaran a simular la posiciòn de nuestro
  public DifferentialDrivetrainSim m_drivetrainSim;
  private final EncoderSim m_leftEncoderSim;
  private final EncoderSim m_rightEncoderSim;
  // La clase "Field2d" nos muestra el robot simulado en una interfaz
  private final Field2d m_fieldSim;
  double currentAngle = 0;
  int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
  SimDouble angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));

  /** Creates a new DriveTrainSubsystem. */
  public DriveTrainSubsystem() {
    // Enviamos a nuestro sistema de conduccion el estado de cada uno de los motores
    SendableRegistry.addChild(m_drive, m_leftLeader);
    SendableRegistry.addChild(m_drive, m_rightLeader);

    /*
     * Hacemos que el motor del mismo lado
     * haga lo mismo que el lider
     */
    m_leftFollower.follow(m_leftLeader);
    m_rightFollower.follow(m_leftFollower);

    /*
     * Necesitamos invertir un lado del sistema de conducciòn
     * El resultado sera que los dos lados se muevan hacia adelante
     * Esto depende de como este puesta la transmicion del robot
     * "En algunos casos el motor que se debe de invertir es el de la izquierda"
     */
    m_rightLeader.setInverted(true);

    // Indicar la distancia por pulso de los encoders
    m_leftEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
    m_rightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);

    resetEncoders();

    m_Odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()), m_leftEncoder.getDistance(),
        m_rightEncoder.getDistance());
    if (RobotBase.isSimulation()) { // Si tu robot es simulado
      // Esta clase simula el movimento de un sistema de manejo alrededor de la cancha
      m_drivetrainSim = new DifferentialDrivetrainSim(DriveConstants.kDrivetrainPlant,
          DriveConstants.kDriveGearbox,
          DriveConstants.kDriveGearing, DriveConstants.kTrackwidthMeters,
          DriveConstants.kWheelDiameterMeters / 2,
          VecBuilder.fill(0, 0, 0.0001, 0.1, 0.1, 0.005, 0.005));
      // Simular los encoder des nuestro robot
      m_leftEncoderSim = new EncoderSim(m_leftEncoder);
      m_rightEncoderSim = new EncoderSim(m_rightEncoder);
      // Clase field2d para la poder ver la simulacion de la
      m_fieldSim = new Field2d();
      SmartDashboard.putData("Field", m_fieldSim);
    } else {
      m_leftEncoderSim = null;
      m_rightEncoderSim = null;
      m_fieldSim = null;
    }

    AutoBuilder.configureRamsete(
        this::getPose, // consiguir la posicion actual del robot
        this::resetOdometry, // reiniciar la odometria existente
        this::getChassisSpeeds, // conseguir la velocidad actual del chasis
        this::drive, // manejar la velocidad del chasis
        AutoConstants.kRamseteB,
        AutoConstants.kRamseteZeta,
        new ReplanningConfig(), // Configuraciòn predeterminada del remapeo. Ve la documentacion de API para mas
                                // opciones
        () -> {
          // Enviador de valor booleano que hace un efecto espejo para la se es de la
          // alianza roja
          // gira la ruta para adaptarse a la alianza roja
          // EL ORIGEN SE MANTENDRA EN LA ALINZA AZUL
          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        }, this // Hace referencia al subsystema y los sus requerimientos
    );
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_Odometry.update(
        Rotation2d.fromDegrees(getHeading()),
        m_leftEncoder.getDistance(),
        m_rightEncoder.getDistance());
    m_fieldSim.setRobotPose(getPose());
  }

  @Override
  public void simulationPeriodic() {
    // To update our simulation, we set motor voltage inputs, update the simulation,
    // and write the simulated positions and velocities to our simulated encoder and
    // gyro.
    // We negate the right side so that positive voltages make the right side
    // move forward.
    m_drivetrainSim.setInputs(
        m_leftLeader.get() * RobotController.getBatteryVoltage(),
        m_rightLeader.get() * RobotController.getBatteryVoltage());
    m_drivetrainSim.update(0.020);

    m_leftEncoderSim.setDistance(m_drivetrainSim.getLeftPositionMeters());
    m_leftEncoderSim.setRate(m_drivetrainSim.getLeftVelocityMetersPerSecond());
    m_rightEncoderSim.setDistance(m_drivetrainSim.getRightPositionMeters());
    m_rightEncoderSim.setRate(m_drivetrainSim.getRightVelocityMetersPerSecond());
    angle.set(-m_drivetrainSim.getHeading().getDegrees());
  }

  /**
   * Obtiene la velocidad del chasis a partir de la velocidad de las ruedas
   * 
   * @return Un obejto con la velocidad del chasis
   */
  public ChassisSpeeds getChassisSpeeds() {
    return m_Kinematics.toChassisSpeeds(getWheelSpeeds());
  }

  /**
   * Retorna la velocidad actual de la ruedas del robot
   *
   * @return La velocidad actual del las ruedas del robot
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(), m_rightEncoder.getRate());
  }

  public void drive(ChassisSpeeds speeds) {
    m_drive.feed();
    xPidController.setSetpoint(speeds.vxMetersPerSecond);
    yawPidController.setSetpoint(speeds.omegaRadiansPerSecond);

    m_drive.arcadeDrive(xPidController.calculate(getChassisSpeeds().vxMetersPerSecond),
        yawPidController.calculate(getChassisSpeeds().omegaRadiansPerSecond));
  }

  /**
   * Conduccion de tanque
   * Controlando cada lado individualmente con voltaje
   * 
   * @param leftVolts  voltaje del lado izquierdo
   * @param rightVolts voltaje del lado derecho
   */

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftLeader.setVoltage(leftVolts);
    m_rightLeader.setVoltage(rightVolts);
    m_drive.feed();
  }

  /**
   * Regresa la posición estimada actual del robot
   * 
   * @return La posición
   */
  public Pose2d getPose() {
    return m_Odometry.getPoseMeters();
  }

  /**
   * Conduccion del robot usando los controles arcade
   *
   * @param fwd El valor para el movimiento hacia adelante
   * @param rot El valor para la rotaciòn
   */

  SlewRateLimiter outputYLimiter = new SlewRateLimiter(1);
  double outputY;
  double outputX;

  public void arcadeDrive(double fwd, double rot) {
    outputY = fwd * 0.9;
    outputX = rot * 0.55;
    m_drive.arcadeDrive(outputY, outputX);
  }

  /**
   * Obtiene la distancia promedio de los dos encoders
   * 
   * @return la distancia promedio de los encoders
   */
  public double getAverageEncoderDistance() {
    return (m_leftEncoder.getDistance() - m_leftEncoder.getDistance()) / 2.0;
  }

  /**
   * Obtiene el encoder izquierdo
   * 
   * @return Un encoder (izquierdo)
   */
  public Encoder getLeftEncoder() {
    return m_leftEncoder;
  }

  /**
   * Obtiene el encoder derecho
   * 
   * @return Un encoder (derecho)
   */
  public Encoder getRightEncoder() {
    return m_rightEncoder;
  }

  /**
   * Da una maxima velocidad al chasis
   * util en casos donde se necesite mas presición
   *
   * @param maxOutput la salida maxima que chasis puede usar
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  /**
   * Reinicia la odometria en una pose especificada
   * 
   * @param pose Posicion en la que se reiniciara la odometria
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_drivetrainSim.setPose(pose);
    m_Odometry.resetPosition(Rotation2d.fromDegrees(getHeading()),
        m_leftEncoder.getDistance(),
        m_rightEncoder.getDistance(),
        pose);
  }

  /** Reincia los encoders del sistema de conducción a un valor de 0 */
  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  /** Reincia el angulo del robot a 0 */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Obtiene el angulo faltante en grados para apuntar hacia la alianza azul
   * 
   * @return Los grados faltantes para apuntar hacia la alianza azul
   */
  public double getAngleCalculationForBlue() {
    return Math.abs(m_gyro.getAngle() % 360);
  }

  /**
   * Regresa el angulo frontal del robot
   * 
   * @return El angulo del roboten grados, de -180 a 180
   */
  public double getHeading() {
    return Math.IEEEremainder(m_gyro.getAngle(), 360) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  /**
   * Hace que el modo neutral del chasis sea de modo Break
   */
  public void setNeutralModeBreak() {
    m_leftLeader.setNeutralMode(NeutralMode.Brake);
    m_rightLeader.setNeutralMode(NeutralMode.Brake);
  }

  /**
   * Hace que el modo neutral del chasis sea de modo Coast
   */
  public void setNeutralModeCoast() {
    m_leftLeader.setNeutralMode(NeutralMode.Coast);
    m_rightLeader.setNeutralMode(NeutralMode.Coast);
  }
}
