// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleConsumer;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
// import com.revrobotics.CANSparkMax;
// import com.revrobotics.CANSparkLowLevel.MotorType;


/**
 * This is a demo program showing the use of the DifferentialDrive class, specifically it contains
 * the code necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {
  private final DifferentialDrive m_robotDrive;
  private final Joystick m_leftStick;
  // private final Joystick m_rightStick;
  // private final XboxController m_controller = new XboxController(0);

  // private final PWMSparkMax m_leftMotor = new PWMSparkMax(3);
  // private final PWMSparkMax m_rightMotor = new CANSparkMax(1);
  private final SparkMax m_leftLeaderMotor = new SparkMax(3, MotorType.kBrushless);
  private final SparkMax m_rightFollowerMotor = new SparkMax(1, MotorType.kBrushless);
  private final SparkMax m_rightLeaderMotor = new SparkMax(2, MotorType.kBrushless);
  private final SparkMax m_leftFollowerMotor = new SparkMax(4, MotorType.kBrushless);
  private final Limelight m_camera = new Limelight(0);
  private Autonomous autonomous;

  /** Called once at the beginning of the robot program. */
  public Robot() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    // m_rightMotor.setInverted(true);

    // Set up configurations for SparkMax motor controllers:
    SparkMaxConfig globalConfig = new SparkMaxConfig();
    SparkMaxConfig rightLeaderConfig = new SparkMaxConfig();
    SparkMaxConfig leftLeaderConfig = new SparkMaxConfig();
    SparkMaxConfig rightFollowerConfig = new SparkMaxConfig();
    SparkMaxConfig leftFollowerConfig = new SparkMaxConfig();

    globalConfig
        .smartCurrentLimit(50)
        .idleMode(IdleMode.kBrake);

    // Apply the global config and invert since it is on the opposite side
    rightLeaderConfig
        .apply(globalConfig)
        .inverted(true);

    leftLeaderConfig
        .apply(globalConfig);

    // Apply the global config and set the leader SPARK for follower mode
    leftFollowerConfig
        .apply(globalConfig)
        .follow(m_leftLeaderMotor);

    // Apply the global config and set the leader SPARK for follower mode
    rightFollowerConfig
        .apply(globalConfig)
        .follow(m_rightLeaderMotor);

    m_leftLeaderMotor.configure(leftLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_leftFollowerMotor.configure(leftFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_rightLeaderMotor.configure(rightLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_rightFollowerMotor.configure(rightFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


    m_robotDrive = new DifferentialDrive(m_leftLeaderMotor::set, m_rightLeaderMotor::set);
    m_leftStick = new Joystick(0);
    // m_rightStick = new Joystick(1);

    SendableRegistry.addChild(m_robotDrive, m_leftLeaderMotor);
    SendableRegistry.addChild(m_robotDrive, m_rightLeaderMotor);
  }

  @Override
  public void teleopInit() {m_camera.turnOffLEDS();}

  @Override
  public void disabledInit() {m_camera.turnOffLEDS();}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void teleopPeriodic() {
    m_robotDrive.arcadeDrive(-m_leftStick.getRawAxis(1), -m_leftStick.getRawAxis(0));
  }

  @Override
  public void autonomousInit() {
    // Initialize limelight properties here:
    System.out.println("Turning on camera LEDs");
    m_camera.turnOnLEDS();
    autonomous = new Autonomous(m_camera, m_robotDrive, 6);
  }

  @Override
  public void autonomousPeriodic() {
    autonomous.tick();
  }
}
