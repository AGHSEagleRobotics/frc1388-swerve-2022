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

  /** Creates a new SwerveCommand. */
  public SwerveCommand(DriveTrain driveTrain, Supplier<Double> leftX, Supplier<Double> leftY, Supplier<Double> rightX) {

    m_driveTrain = driveTrain;

    m_leftX =  leftX;
    m_leftY =  leftY;
    m_rightX = rightX;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
  } // end constructer

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double ymps =     MathUtil.applyDeadband(m_leftX.get()  * -0.5, .1);
    double xmps =     MathUtil.applyDeadband(m_leftY.get()  * -0.5, .1);
    double rotation = MathUtil.applyDeadband(m_rightX.get() * -1,   .1);

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
