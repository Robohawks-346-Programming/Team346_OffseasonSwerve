// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final double DRIVETRAIN_TRACKWIDTH_METERS             = 0.7366; //29 in
    public static final double DRIVETRAIN_WHEELBASE_METERS              = 0.7874; //31 in
    public static final double DRIVETRAIN_GEAR_RATIO                    = 6.75; //For L2 Module
    public static final double WHEEL_DIAMETER                           = 0.09398; // 3.7 in
    public static final double DRIVE_CONVERSION                         = (WHEEL_DIAMETER * Math.PI) / DRIVETRAIN_GEAR_RATIO;
    public static final double TURN_CONVERSION                          = 12.8;
    public static final SwerveDriveKinematics DRIVE_KINEMATICS          = 
        new SwerveDriveKinematics(
            new Translation2d(DRIVETRAIN_WHEELBASE_METERS/ 2.0, DRIVETRAIN_TRACKWIDTH_METERS / 2.0), // front left
            new Translation2d(DRIVETRAIN_WHEELBASE_METERS/ 2.0, -DRIVETRAIN_TRACKWIDTH_METERS / 2.0), // front right
            new Translation2d(-DRIVETRAIN_WHEELBASE_METERS/ 2.0, DRIVETRAIN_TRACKWIDTH_METERS / 2.0), // back left
            new Translation2d(-DRIVETRAIN_WHEELBASE_METERS/ 2.0, -DRIVETRAIN_TRACKWIDTH_METERS / 2.0) // back right
        );

    public static final double MAX_VELOCITY                             = 1; // just for testing
    public static final int MAX_VOLTAGE                                 = 12;
    public static final int DRIVE_CURRENT_LIMIT                         = 60;
    public static final int TURN_CURRENT_LIMIT                          = 25;
    public static final double MAX_MOVE_VELOCITY                        = 0.5; // for testing
    public static final double MAX_TURN_VELOCITY                        = 0.5; // for testing
    public static final boolean IS_FIELD_RELATIVE                       = true;
    
    public static final int FRONT_LEFT_DRIVE_ID                         = 22;
    public static final int FRONT_LEFT_TURN_ID                          = 23;
    public static final int FRONT_LEFT_ENCODER_ID                       = 43;
    public static final double FRONT_LEFT_TURN_OFFSET                   = 180.0;

    public static final int FRONT_RIGHT_DRIVE_ID                        = 12;
    public static final int FRONT_RIGHT_TURN_ID                         = 13;
    public static final int FRONT_RIGHT_ENCODER_ID                      = 33;
    public static final double FRONT_RIGHT_TURN_OFFSET                  = 180.0;

    public static final int BACK_LEFT_DRIVE_ID                          = 24;
    public static final int BACK_LEFT_TURN_ID                           = 25;
    public static final int BACK_LEFT_ENCODER_ID                        = 45;
    public static final double BACK_LEFT_TURN_OFFSET                    = 180.0;

    public static final int BACK_RIGHT_DRIVE_ID                         = 10;
    public static final int BACK_RIGHT_TURN_ID                          = 11;
    public static final int BACK_RIGHT_ENCODER_ID                       = 31;
    public static final double BACK_RIGHT_TURN_OFFSET                   = 180.0;

    public static final double DRIVE_P                                  = 0.1;
    public static final double DRIVE_I                                  = 0;
    public static final double DRIVE_D                                  = 0;
    public static final double DRIVE_FF                                 = 2.96;

    public static final double TURN_P                                   = 0.01;
    public static final double TURN_I                                   = 0;
    public static final double TURN_D                                   = 0.005;
    public static final double TURN_FF                                  = 0;

    public static final int DRIVER_CONTROLLER_PORT                      = 0;
}
