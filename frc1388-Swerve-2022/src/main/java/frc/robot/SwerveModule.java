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
import edu.wpi.first.wpilibj.AnalogInput;

/** Add your docs here. */
public class SwerveModule {

    private final double minPerSec = 60;
    private final double motorRotationsPerWheelRotation = 6.67;
    private final double wheelDistanceInInchesPerRotation = 4 * Math.PI;
    private final double metersPerInch = 0.0254;
    private final double motorRotationsPerMeter = (wheelDistanceInInchesPerRotation * metersPerInch) / minPerSec * motorRotationsPerWheelRotation;
    private final double metersPerMotorRotation = 1 / motorRotationsPerMeter;

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
        m_driveEncoder.setVelocityConversionFactor(12056156);
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

    public void setOffset(int offset) {
        m_encoderOffset = offset;
    }
    public double getOffset() {
        return m_encoderOffset;
    }

    /** Setting motion speed
     * @param speed is in meter / second
     */
    public void setDriveSpeed(double speed) {
        speed *= metersPerMotorRotation;
        m_driveMotorPID.setReference(speed, CANSparkMax.ControlType.kVelocity);
        m_driveEncoder.getVelocity();
    }

    /** sets rotation
     * @param rotation is in degrees
     */
    public void setRotationPosition(double rotaiton) {
        m_rotationMotor.set(m_rotationPidController.calculate(getRotationAngle() , rotaiton));
    }

    /** turns voltage from encoder to degrees */
    private double getRotationAngle() {
        return MathUtil.clamp((360 / (4.987 - 0.015)) * (m_rotationEncoder.getVoltage() - 0.015), 0, 360);
    }

}
