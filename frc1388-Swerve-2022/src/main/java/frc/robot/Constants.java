// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public final class SwerveModuleConstants {
        public static final double ROTATION_P = 0.01;
        public static final double ROTATION_I = 0;
        public static final double ROTATION_D = 0;

        public static final double DRIVE_P = 0.005;
        public static final double DRIVE_I = 0;
        public static final double DRIVE_D = 0;
    }

    public final class DriveTrainConstants {
        // probaly won't use these:
        // public double FRONT_RIGHT_ENCODER_OFSET = 0;
        // public double FRONT_LEFT_ENCODER_OFSET =  0;
        // public double BACK_LEFT_ENCODER_OFSET =   0;
        // public double Back_RIGHT_ENCODER_OFSET =  0;

        public static final int FRONT_RIGHT_ROTATION_MOTOR_ID = 1;
        public static final int FRONT_RIGHT_ROTATION_ENCODER_ID = 0;
        public static final int FRONT_RIGHT_DRIVE_MOTOR_ID = 5;
        
        public static final int FRONT_LEFT_ROTATION_MOTOR_ID = 2;
        public static final int FRONT_LEFT_ROTATION_ENCODER_ID = 1;
        public static final int FRONT_LEFT_DRIVE_ENCODER_ID = 6;
        
        public static final int BACK_LEFT_ROTATION_MOTOR_ID = 3;
        public static final int BACK_LEFT_ROTATION_ENCODER_ID = 2;
        public static final int BACK_LEFT_DRIVE_MOTOR_ID = 7;
        
        public static final int BACK_RIGHT_ROTATION_MOTOR_ID = 4;
        public static final int BACK_RIGHT_ROTATION_ENCODER_ID = 3;
        public static final int BACK_RIGHT_DRIVE_MOTOR_ID = 8;
    }
}
