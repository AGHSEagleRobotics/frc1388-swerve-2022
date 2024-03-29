// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class SwerveCommand extends CommandBase {

  private final DriveTrain m_driveTrain;

  // Joysticks
  private Supplier<Double> m_leftX;
  private Supplier<Double> m_leftY;
  private Supplier<Double> m_rightX;
  private Supplier<Double> m_rightTrigger;

  /** Creates a new SwerveCommand. */
  public SwerveCommand(DriveTrain driveTrain, Supplier<Double> leftX, Supplier<Double> leftY, Supplier<Double> rightX, Supplier<Double> rightTrigger) {

    m_driveTrain = driveTrain;

    m_leftX =  leftX;
    m_leftY =  leftY;
    m_rightX = rightX;
    m_rightTrigger = rightTrigger;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
  } // end constructer

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double leftx = m_leftX.get();
    double lefty = m_leftY.get();
    double rightx = m_rightX.get();

    // remove for regular use
    double rightTrigger = m_rightTrigger.get();
    rightTrigger = Math.pow(rightTrigger, 1.5) + 0.2;
    double theta = Math.atan2(lefty, leftx);

    // cubic scaling
    // leftx = leftx * leftx * leftx;
    // lefty = lefty * lefty * lefty;
    rightx = rightx * rightx * rightx;

    // gas pedal (remove for regular use)
    if ( MathUtil.applyDeadband(leftx , .1) == 0 && MathUtil.applyDeadband(lefty , .1) == 0) {
      leftx = 0;
      lefty = 0;
    } else {
      leftx = rightTrigger * Math.cos(theta);
      lefty = rightTrigger * Math.sin(theta);
    }

    System.out.println(rightTrigger);

    // max speed 3mps
    double ymps =     -0.5 * MathUtil.applyDeadband(leftx , .1);
    double xmps =     -0.5 * MathUtil.applyDeadband(lefty , .1);
    double rotation = -1 * MathUtil.applyDeadband(rightx,   .1);

    m_driveTrain.move(new Vector2d(xmps, ymps), rotation);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveTrain.move(new Vector2d(0, 0), 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
