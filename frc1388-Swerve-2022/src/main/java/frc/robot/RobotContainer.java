// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.SwerveCommand;
// import frc.robot.commands.SwerveTest;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...


  private final XboxController m_driveController = new XboxController(0);

  private final DriveTrain m_driveTrainSubsystem  = new DriveTrain(
    new SwerveModule(
      new WPI_TalonSRX(Constants.DriveTrainConstants.FRONT_RIGHT_ROTATION_MOTOR_ID), 
      new AnalogInput(Constants.DriveTrainConstants.FRONT_RIGHT_ROTATION_ENCODER_ID), 
      new CANSparkMax(Constants.DriveTrainConstants.FRONT_RIGHT_DRIVE_MOTOR_ID, MotorType.kBrushless)
    ),
    new SwerveModule(
      new WPI_TalonSRX(Constants.DriveTrainConstants.FRONT_LEFT_ROTATION_MOTOR_ID), 
      new AnalogInput(Constants.DriveTrainConstants.FRONT_LEFT_ROTATION_ENCODER_ID), 
      new CANSparkMax(Constants.DriveTrainConstants.FRONT_LEFT_DRIVE_ENCODER_ID, MotorType.kBrushless)
    ),
    new SwerveModule(
      new WPI_TalonSRX(Constants.DriveTrainConstants.BACK_LEFT_ROTATION_MOTOR_ID), 
      new AnalogInput(Constants.DriveTrainConstants.BACK_LEFT_ROTATION_ENCODER_ID), 
      new CANSparkMax(Constants.DriveTrainConstants.BACK_LEFT_DRIVE_MOTOR_ID, MotorType.kBrushless)
    ),
    new SwerveModule(
      new WPI_TalonSRX(Constants.DriveTrainConstants.BACK_RIGHT_ROTATION_MOTOR_ID), 
      new AnalogInput(Constants.DriveTrainConstants.BACK_RIGHT_ROTATION_ENCODER_ID), 
      new CANSparkMax(Constants.DriveTrainConstants.BACK_RIGHT_DRIVE_MOTOR_ID, MotorType.kBrushless)
    ),
    new ADIS16470_IMU()
  );

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // uncomment
    m_driveTrainSubsystem.setDefaultCommand(new SwerveCommand(
      m_driveTrainSubsystem,
      () -> m_driveController.getLeftX(),
      () -> m_driveController.getLeftY(),
      () -> m_driveController.getRightX(),
      () -> m_driveController.getRightTriggerAxis()
    ));

    // m_driveTrainSubsystem.setDefaultCommand(
    //   new SwerveTest(m_driveTrainSubsystem,
    //   () -> m_driveController.getRightX(),
    //   () -> m_driveController.getLeftY()
    //   ));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(m_driveController, XboxController.Button.kA.value)
    .whenPressed(()-> m_driveTrainSubsystem.resetGyro());
  }

  /**
   * RETURNS NULL!!!  
   * 
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    // return m_autoCommand;
    return null;
  }
}
