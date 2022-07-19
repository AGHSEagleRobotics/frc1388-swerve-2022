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
  
  private final SwerveModule m_frontRight;
  private final SwerveModule m_frontLeft;
  private final SwerveModule m_backLeft;
  private final SwerveModule m_backRight;
  
  private final Translation2d[] m_swerveTranslation2ds = {
    new Translation2d(5, 2),
    new Translation2d(5, 2),
    new Translation2d(5, 2),
    new Translation2d(5, 2)
  };
  
  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(m_swerveTranslation2ds);
  
  /** Creates a new DriveTrain. */
  public DriveTrain(SwerveModule frontRight, SwerveModule frontLeft, SwerveModule backLeft, SwerveModule backRight) {
    m_frontRight = frontRight;
    m_frontLeft =  frontLeft;
    m_backLeft =   backLeft;
    m_backRight =  backRight;

    m_frontRight.setOffset(0);
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
  // public void foo(ChassisSpeeds robotSpeeds) {
  //   // returns SwerveModuleStates[]
  //   var dsdfsadas = m_kinematics.toSwerveModuleStates(robotSpeeds );
  // }
  public void setStates(SwerveModuleState... state) {
    SwerveDriveKinematics.desaturateWheelSpeeds(state, 2);

    // TODO this method also needs to do other things I think

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // for testing pid
    // TODO uncoment this for testing
    // for (int i = 0; i < 100; i++) {
    //   for (int j = 0; j < 150; j++) {
    //     m_frontRight.setRotationPosition(i * 45);
    //   }
    // }

    /*
     * FIXME make m_speed and m_angle private
     * TODO maybe have no params for setDriveSpeed and setRotationPosition, insted use the fields in the class
     */

    //XXX if testing comment this out
    m_frontRight.setDriveSpeed(m_frontRight.m_speed);
    m_frontLeft.setDriveSpeed(m_frontLeft.m_speed);
    m_backLeft.setDriveSpeed(m_backLeft.m_speed);
    m_backRight.setDriveSpeed(m_backRight .m_speed);

    m_frontRight.setRotationPosition(m_frontRight.m_angle);
    m_frontLeft.setRotationPosition(m_frontLeft.m_angle);
    m_backLeft.setRotationPosition(m_backLeft.m_angle);
    m_backRight.setRotationPosition(m_backRight.m_angle);

  }
  
}
