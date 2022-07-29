// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SwerveModule;

public class DriveTrain extends SubsystemBase {
  
  private final double TRACK  =  1.26; // width from center of rotation of swerve module in meters
  private final double WHEEL_BASE = 1; // length from center of rotation of swerve module in meters
  
  private final SwerveModule m_frontRight;
  private final SwerveModule m_frontLeft;
  private final SwerveModule m_backLeft;
  private final SwerveModule m_backRight;

  private final double CENTER_TO_TRACK = TRACK / 2;
  private final double CENTER_TO_WHEEL_BASE = WHEEL_BASE / 2;
  private final Translation2d m_frontRighTranslation2d = new Translation2d( CENTER_TO_WHEEL_BASE, -CENTER_TO_TRACK);
  private final Translation2d m_frontLefTranslation2d  = new Translation2d( CENTER_TO_WHEEL_BASE,  CENTER_TO_TRACK);
  private final Translation2d m_backLefTranslation2d   = new Translation2d(-CENTER_TO_WHEEL_BASE,  CENTER_TO_TRACK);
  private final Translation2d m_backRighTranslation2d  = new Translation2d(-CENTER_TO_WHEEL_BASE, -CENTER_TO_TRACK);
  
  // TODO use numerical index to map these guys
  private final Translation2d[] m_swerveTranslation2ds = {
    m_frontRighTranslation2d,
    m_frontLefTranslation2d,
    m_backLefTranslation2d,
    m_backRighTranslation2d 
  };
  
  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(m_swerveTranslation2ds);

  // for testing
  public enum Locations {frontRight, frontLeft, backLeft, backRight};
  
  /** Creates a new DriveTrain. */
  public DriveTrain(SwerveModule frontRight, SwerveModule frontLeft, SwerveModule backLeft, SwerveModule backRight) {
    m_frontRight = frontRight;
    m_frontLeft =  frontLeft;
    m_backLeft =   backLeft;
    m_backRight =  backRight;

    m_frontRight.setOffset(90);
    m_frontLeft.setOffset(0);
    m_backLeft.setOffset(0);
    m_backRight.setOffset(0);

    System.out.println("**************************\ntest print\n\n\n\n");
  }

  public void testPrint(double i) {
    System.out.println("**************************\ntest print\n\n\n\n");
  }

  public void move(Vector2d velocity, double omega) {
    setStates(m_kinematics.toSwerveModuleStates(new ChassisSpeeds(velocity.x, velocity.y, omega)));
  }

  public void setStates(SwerveModuleState[] states) {
    SwerveDriveKinematics.desaturateWheelSpeeds(states, 3); // 3 m/s max speed

    m_frontRight.setSwerveModuleState(states[0]);
    m_frontLeft.setSwerveModuleState(states[1]);
    m_backLeft.setSwerveModuleState(states[2]);
    m_backRight.setSwerveModuleState(states[3]);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  // for testing
  public void testModule(Locations module, double rotation, double speed) {
    switch (module){
      case frontRight: m_frontRight.setDriveSpeed(speed);
      m_frontRight.setRotationPosition(rotation);
      break;

      case frontLeft:  m_frontLeft.setDriveSpeed(speed);
      m_frontLeft.setRotationPosition(rotation);
      break;

      case backLeft: m_backLeft.setDriveSpeed(speed);
      m_backLeft.setRotationPosition(rotation);
      break;

      case backRight:  m_backRight.setDriveSpeed(speed);
      m_backRight.setRotationPosition(rotation);
      break;

      default: System.out.println("you broke it :(  ");
    }
  } //  end testing method
  
}
