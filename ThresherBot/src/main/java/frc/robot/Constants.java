// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

public final class Constants 
{
   public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
   public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
   public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms sprk max velocity lag

   // CAN IDs HERE
   // BackLeftDrive/Angle:    4,  5
   // BackRightDrive/Angle:   6,  7
   // FrontLeftDrive/Angle:   8,  9
   // FrontRightDrive/Angle: 10, 11
   // Pidgeon: 20

   public static final int ELEVATOR_MASTER_CAN_ID = 12;
   public static final int ELEVATOR_SLAVE_CAN_ID  = 13;
   public static final int ARM_WRIST_CAN_ID       = 14;
   public static final int ARM_ENCODER_CAN_ID     = 15;
   public static final int LASER_CAN_A_ID         = 16;
   public static final int LASER_CAN_B_ID         = 17;

   public static class DrivebaseConstants
   {
      public static final double MAX_SPEED = Units.feetToMeters(14.5);
      public static final double WHEEL_LOCK_TIME = 10; // seconds

   }


   public static class OperatorConstants 
   {
      public static final int XBOX_DRIVER_PORT   = 0;
      public static final int XBOX_OPERATOR_PORT = 1;

      public static final double DEADBAND = 0.1;
      public static final double LEFT_Y_DEADBAND = 0.1;
      public static final double RIGHT_X_DEADBAND = 0.1;
      public static final double TURN_CONSTANT = 6;
   }

   public static class ArmConstants
   {
      public static final double ELEVATOR_TRANSMISSION_RATIO = 15.0;
      public static final int ELEVATOR_LOW_LIMIT_SWITCH_PORT = 0;
      public static final int ELEVATOR_HIGH_LIMIT_SWITCH_PORT = 1;
      public static final int WRIST_LOW_LIMIT_SWITCH_PORT = 2;
      // TODO: TUNE
      public static final double WRIST_PID_P = 2.4;
      public static final double WRIST_PID_I = 0.0;
      public static final double WRIST_PID_D = 0.1;
      public static final double WRIST_PID_FF = 0.0;

      public static final double WRIST_ERROR_TOLERANCE = 2.5; // degrees
      public static final double WRIST_MAX_OUTPUT_UP = 0.4;
      public static final double WRIST_MAX_OUTPUT_DOWN = -0.3;
      public static final double WRIST_MAX_OUTPUT_DOWN_VERY_SLOW = -0.1;

      public static final double ELEVATOR_PID_P = 0.3;
      public static final double ELEVATOR_PID_I = 0.0;
      public static final double ELEVATOR_PID_D = 0.01;
      public static final double ELEVATOR_PID_FF = 0.0;

      public static final double ELEVATOR_ERROR_TOLERANCE = 0.5; // inches
      public static final double ELEVATOR_MAX_OUTPUT_UP = 0.65;
      public static final double ELEVATOR_MAX_OUTPUT_DOWN = -0.25;
      public static final double ELEVATOR_ENC_CONV_FACTOR = ((8.2686718/ELEVATOR_TRANSMISSION_RATIO));

      public static final double ELEVATOR_POS_DOWN     = 0.35;
      public static final double ELEVATOR_POS_MAX_HEIGHT = 26.35;
      public static final double ELEVATOR_POS_NEUTRAL  = 6.0;
      public static final double ELEVATOR_POS_SCORE_L2 = 10.0;
      public static final double ELEVATOR_POS_SCORE_L3 = 20.0;
      public static final double ELEVATOR_POS_SCORE_L4 = ELEVATOR_POS_MAX_HEIGHT;

      public static final double WRIST_POS_DOWN     = 0.0;
      public static final double WRIST_POS_ELV_SAFE = 5.0; // Minimum arm angle before elevator can move down
      public static final double WRIST_POS_SCORE_PREP_L2 = 20.0;
      public static final double WRIST_POS_SCORE_END_L2 = 70.0;
      public static final double WRIST_POS_SCORE_L3 = 90.0;
      public static final double WRIST_POS_SCORE_PREP_L4 = 130.0;
      public static final double WRIST_POS_SCORE_END_L4 = 70.0;
      public static final double WRIST_POS_UP = 175.0;
      public static final double WRIST_POS_VERY_SLOW = 10.0;

   }
}
