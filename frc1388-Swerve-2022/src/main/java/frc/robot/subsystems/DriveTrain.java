// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SwerveModule;

public class DriveTrain extends SubsystemBase {

  private final SwerveModule m_frontRight;
  private final SwerveModule m_frontLeft;
  private final SwerveModule m_backLeft;
  private final SwerveModule m_backRight;

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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // for testing pid
    for (int i = 0; i < 100; i++) {
      for (int j = 0; j < 150; j++) {
        m_frontRight.setRotationPosition(i * 45);
      }
    }

    // System.out.println("**************************\ntest print \n \n \n \n ");
    

  }
  
}
