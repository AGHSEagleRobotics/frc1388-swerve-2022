// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogInput;

/** Add your docs here. */
public class SwerveModule {

    public double m_angle;
    public double m_speed;

    private final double secPerMin = 60;
    private final double motorRotationsPerWheelRotation = 6.67; // from AndyMark
    private final double inchesPerWheelRotation = 4 * Math.PI;
    private final double metersPerInch = 0.0254;
    private final double metersPerMotorRotation = (inchesPerWheelRotation * metersPerInch) / motorRotationsPerWheelRotation;
    private final double metersPerSecondPerRPM = metersPerMotorRotation * secPerMin;

    private final WPI_TalonSRX m_rotationMotor;
    private final AnalogInput m_rotationEncoder;
    private final PIDController m_rotationPidController;

    private final CANSparkMax m_driveMotor;
    private final RelativeEncoder m_driveEncoder;
    private final SparkMaxPIDController m_driveMotorPID;

    private double m_encoderOffset = 0;

    public SwerveModule(WPI_TalonSRX rotationMotor, AnalogInput rotationEncoder, CANSparkMax driveMotor) {

        m_rotationMotor = rotationMotor;
        m_rotationEncoder = rotationEncoder;

        m_driveMotor = driveMotor;
        m_driveMotor.setIdleMode(IdleMode.kBrake);
        m_driveEncoder = m_driveMotor.getEncoder();
        m_driveEncoder.setVelocityConversionFactor(metersPerSecondPerRPM);
        m_driveMotorPID = m_driveMotor.getPIDController();
        m_driveMotorPID.setP(Constants.SwerveModuleConstants.DRIVE_P);
        m_driveMotorPID.setI(Constants.SwerveModuleConstants.DRIVE_I);
        m_driveMotorPID.setD(Constants.SwerveModuleConstants.DRIVE_D);

        // add values
        m_rotationPidController = new PIDController(Constants.SwerveModuleConstants.ROTATION_P, Constants.SwerveModuleConstants.ROTATION_I, Constants.SwerveModuleConstants.ROTATION_D);

        m_rotationPidController.setTolerance(3);
        m_rotationPidController.enableContinuousInput(0, 360);
        m_rotationPidController.setIntegratorRange(-1, 1);

    }

    // offset
    public void setOffset(int offset) {
        m_encoderOffset = offset;
    }
    public double getOffset() {
        return m_encoderOffset;
    }

    // motors
    public void setSwerveModuleState(SwerveModuleState possition) {
        SwerveModuleState swerveModuleState = SwerveModuleState.optimize(possition, new Rotation2d(m_angle));
        m_angle = swerveModuleState.angle.getDegrees();
        m_speed = swerveModuleState.speedMetersPerSecond;

        setDriveSpeed(m_speed);
        setRotationPosition(m_angle);
    }
        /** Setting motion speed
     * @param speed is in meter / second
     */
    public void setDriveSpeed(double speed) {
        m_driveMotorPID.setReference(speed / metersPerSecondPerRPM, CANSparkMax.ControlType.kVelocity);
        // m_driveEncoder.getVelocity();
    }
        /** sets rotation
     * @param rotation is in degrees
     */
    public void setRotationPosition(double rotaiton) {
        m_rotationMotor.set(m_rotationPidController.calculate(getRotationAngle() , rotaiton));
    }

    /** turns voltage from encoder to degrees */
    private double getRotationAngle() {
        return m_encoderOffset + MathUtil.clamp((360 / (4.987 - 0.015)) * (m_rotationEncoder.getVoltage() - 0.015), 0, 360);
    }
}
