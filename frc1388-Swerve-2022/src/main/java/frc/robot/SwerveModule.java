// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.motorcontrol.Talon;

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

    private final CANSparkMax m_driveMotor;
    private final RelativeEncoder m_driveEncoder;
    private final SparkMaxPIDController m_driveMotorPID;

    public SwerveModule(WPI_TalonSRX rotationMotor, AnalogInput rotationEncoder, CANSparkMax driveMotor) {

        m_rotationMotor = rotationMotor;
        m_rotationEncoder = rotationEncoder;

        m_driveMotor = driveMotor;
        m_driveMotor.setIdleMode(IdleMode.kBrake);
        m_driveEncoder = m_driveMotor.getEncoder();
        m_driveEncoder.setVelocityConversionFactor(12056156);
        m_driveMotorPID = m_driveMotor.getPIDController();
        m_driveMotorPID.setP(1);
        m_driveMotorPID.setI(0);
        m_driveMotorPID.setD(0);

    }

    /** have fun 
     * @param speed i dont know
    */
    public void setDriveSpeed(double speed) {
        speed *=metersPerMotorRotation;
        m_driveMotorPID.setReference(speed, CANSparkMax.ControlType.kVelocity);
        m_driveEncoder.getVelocity();
    }

    public void setRotationPosition() {

    }

}
