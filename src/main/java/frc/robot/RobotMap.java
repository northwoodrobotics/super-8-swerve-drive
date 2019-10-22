/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
  // For example to map the left and right motors, you could define the
  // following variables to use with your drivetrain subsystem.
  // public static int leftMotor = 1;
  // public static int rightMotor = 2;

  //Motor Map
  // I = inverse, R = rotate
  public static int rightBack = 0;
  public static int rightBackR = 1;
  public static boolean rightBackI = true;
  public static boolean rightBackRI = false;

  public static int leftBack = 2;
  public static int leftBackR = 3;
  public static boolean leftBackI = false;
  public static boolean leftBackRI = false;

  public static int leftFront = 4;
  public static int leftFrontR = 5;
  public static boolean leftFrontI = false;
  public static boolean leftFrontRI = false;

  public static int rightFront = 6;
  public static int rightFrontR = 7;
  public static boolean rightFrontI = true;
  public static boolean rightFrontRI = false;

  // If you are using multiple modules, make sure to define both the port
  // number and the module. For example you with a rangefinder:
  // public static int rangefinderPort = 1;
  // public static int rangefinderModule = 1;




}
