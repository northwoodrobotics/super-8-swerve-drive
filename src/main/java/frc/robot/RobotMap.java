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
  // I = inverse, S = steer
  
  
  //Motor 0
  public static int leftFront = 1;
  public static int leftFrontS = 2;
  public static boolean leftFrontI = false;
  public static boolean leftFrontSI = false;
  
  //Motor 1
  public static int rightFront = 3;
  public static int rightFrontS = 4;
  public static boolean rightFrontI = true;
  public static boolean rightFrontSI = false;
  
  //Motor 2
  public static int rightBack = 5;
  public static int rightBackS = 6;
  public static boolean rightBackI = true;
  public static boolean rightBackSI = false;
  
  //Motor 3
  public static int leftBack = 7;
  public static int leftBackS = 8;
  public static boolean leftBackI = false;
  public static boolean leftBackSI = false;
  

  // If you are using multiple modules, make sure to define both the port
  // number and the module. For example you with a rangefinder:
  // public static int rangefinderPort = 1;
  // public static int rangefinderModule = 1;




}
