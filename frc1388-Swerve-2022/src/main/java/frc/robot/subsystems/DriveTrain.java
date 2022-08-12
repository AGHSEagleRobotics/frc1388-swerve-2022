// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SwerveModule;

public class DriveTrain extends SubsystemBase {
  
  private final double TRACK  =  0.5588; // width from center of rotation of swerve module in meters
  private final double WHEEL_BASE = 0.5588; // length from center of rotation of swerve module in meters
  
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

  private final ADIS16470_IMU m_gyro;
  
  /** Creates a new DriveTrain. */
  public DriveTrain(SwerveModule frontRight, SwerveModule frontLeft, SwerveModule backLeft, SwerveModule backRight, ADIS16470_IMU gyro) {
    m_frontRight = frontRight;
    m_frontLeft =  frontLeft;
    m_backLeft =   backLeft;
    m_backRight =  backRight;

    m_frontRight.setOffset(2);
    m_frontLeft.setOffset(173);
    m_backLeft.setOffset(270);
    m_backRight.setOffset(185);

    m_gyro = gyro;
    m_gyro.calibrate();
    m_gyro.reset();
    m_gyro.setYawAxis(IMUAxis.kX);

  }

  public void testPrint(double i) {
  }

  public void move(Vector2d velocity, double omega) {
    // ChassisSpeeds speeds = new ChassisSpeeds(velocity.x, velocity.y, omega);
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(velocity.x, velocity.y, omega, new Rotation2d(getGyroAngle()));
    SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, 0.5); // 3 m/s max speed

    System.out.println(states[0].speedMetersPerSecond + "\t" + states[0].angle);
    System.out.println(m_frontRight.getRotationAngle());

    m_frontRight.setSwerveModuleState(states[0]);
    m_frontLeft.setSwerveModuleState(states[1]);
    m_backLeft.setSwerveModuleState(states[2]);
    m_backRight.setSwerveModuleState(states[3]);
    // setStates(m_kinematics.toSwerveModuleStates(new ChassisSpeeds(velocity.x, velocity.y, omega)));
  }

  // public void setStates(SwerveModuleState[] states) {
  //   SwerveDriveKinematics.desaturateWheelSpeeds(states, 0.5); // 3 m/s max speed

  //   System.out.println(states[0].speedMetersPerSecond + "\t" + states[0].angle);
  //   System.out.println(m_frontRight.getRotationAngle());

  //   m_frontRight.setSwerveModuleState(states[0]);
  //   m_frontLeft.setSwerveModuleState(states[1]);
  //   m_backLeft.setSwerveModuleState(states[2]);
  //   m_backRight.setSwerveModuleState(states[3]);
  // }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // System.out.println("front right " + m_frontRight.getRotationAngle());
    // System.out.println("front left " + m_frontLeft.getRotationAngle());
    // System.out.println("back left " + m_backLeft.getRotationAngle());
    // System.out.println("back right " + m_backRight.getRotationAngle());

    System.out.println("gyro in degrees: " + getGyroAngle());
   
  }

  // for testing
  public void testModule(Locations module, double rotation, double speed) {
    // switch (module){
      // case frontRight: 
      m_frontRight.setDriveSpeed(speed);
      m_frontRight.setRotationPosition(rotation);
      // break;

      // case frontLeft:  
      m_frontLeft.setDriveSpeed(speed);
      m_frontLeft.setRotationPosition(rotation);
      // break;

      // case backLeft: 
      m_backLeft.setDriveSpeed(speed);
      m_backLeft.setRotationPosition(rotation);
      // break;

      // case backRight: 
     m_backRight.setDriveSpeed(speed);
      m_backRight.setRotationPosition(rotation);
      // break;

      // default: System.out.println("you broke it :(  ");
    // }
  } //  end testing method

  /**
   * Gets the gyro angle
   * @return returns the angle in radians
   */
  private double getGyroAngle() {
    return Math.toRadians(m_gyro.getAngle() + 180.0);
  }
  
}
