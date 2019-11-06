/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.interfaces.Gyro;

import org.usfirst.frc4048.swerve.drive.SwerveDrive;
import org.usfirst.frc4048.swerve.math.CentricMode;
import frc.robot.RobotMap;
import frc.robot.commands.teleop.TeleDrive;
import org.usfirst.frc4048.swerve.drive.CanTalonSwerveEnclosure;


import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;


public class Drivetrain extends Subsystem {
	private CanTalonSwerveEnclosure swerveEnclosure1;
	private CanTalonSwerveEnclosure swerveEnclosure0;
	private CanTalonSwerveEnclosure swerveEnclosure3;
	private CanTalonSwerveEnclosure swerveEnclosure2;
	private SwerveDrive swerveDrive;

	public static final double GEAR_RATIO = (1024d);
	private static final double L = 21.0;
	private static final double W = 23.5;

	private static final double P = 10.0;
	private static final double I = 0.0;
	private static final double D = 0.0;
	private static final double F = 0.0;

	private WPI_TalonSRX driveMotor0;
	private WPI_TalonSRX driveMotor1;
	private WPI_TalonSRX driveMotor2;
	private WPI_TalonSRX driveMotor3;

	private WPI_TalonSRX steerMotor0;
	private WPI_TalonSRX steerMotor1;
	private WPI_TalonSRX steerMotor2;
	private WPI_TalonSRX steerMotor3;

	double[] wheelAngles = new double[4];

	private Gyro gyro = new ADXRS450_Gyro();
	private CentricMode centricMode = CentricMode.ROBOT;
	private boolean beakIsFront = false;

	public Drivetrain() {

	}

	public void init() {
		
		driveMotor0 = new WPI_TalonSRX(RobotMap.leftFront);
		driveMotor1 = new WPI_TalonSRX(RobotMap.rightFront);
		driveMotor2 = new WPI_TalonSRX(RobotMap.rightBack);
		driveMotor3 = new WPI_TalonSRX(RobotMap.leftBack);

		driveMotor0.setInverted(RobotMap.leftFrontI);
		driveMotor1.setInverted(RobotMap.rightFrontI);
		driveMotor2.setInverted(RobotMap.rightBackI);
		driveMotor3.setInverted(RobotMap.leftBackI);

		driveMotor0.setNeutralMode(NeutralMode.Brake);
		driveMotor1.setNeutralMode(NeutralMode.Brake);
		driveMotor2.setNeutralMode(NeutralMode.Brake);
		driveMotor3.setNeutralMode(NeutralMode.Brake);

		steerMotor0 = new WPI_TalonSRX(RobotMap.leftFrontS);
		steerMotor1 = new WPI_TalonSRX(RobotMap.rightFrontS);
		steerMotor2 = new WPI_TalonSRX(RobotMap.rightBackS);
		steerMotor3 = new WPI_TalonSRX(RobotMap.leftBackS);

		steerMotor0.setInverted(RobotMap.leftFrontSI);
		steerMotor1.setInverted(RobotMap.rightFrontSI);
		steerMotor2.setInverted(RobotMap.rightFrontSI);
		steerMotor3.setInverted(RobotMap.leftBackSI);

		steerMotor0.configSelectedFeedbackSensor(FeedbackDevice.Analog);
		steerMotor1.configSelectedFeedbackSensor(FeedbackDevice.Analog);
		steerMotor2.configSelectedFeedbackSensor(FeedbackDevice.Analog);
		steerMotor3.configSelectedFeedbackSensor(FeedbackDevice.Analog);

		steerMotor0.selectProfileSlot(0, 0);
		steerMotor1.selectProfileSlot(0, 0);
		steerMotor2.selectProfileSlot(0, 0);
		steerMotor3.selectProfileSlot(0, 0);

		steerMotor0.config_kP(0, P);
		steerMotor1.config_kP(0, P);
		steerMotor2.config_kP(0, P);
		steerMotor3.config_kP(0, P);

		steerMotor0.config_kI(0, I);
		steerMotor1.config_kI(0, I);
		steerMotor2.config_kI(0, I);
		steerMotor3.config_kI(0, I);

		steerMotor0.config_kD(0, D);		
		steerMotor1.config_kD(0, D);
		steerMotor2.config_kD(0, D);
		steerMotor3.config_kD(0, D);

		steerMotor0.config_kF(0, F);
		steerMotor1.config_kF(0, F);
		steerMotor2.config_kF(0, F);
		steerMotor3.config_kF(0, F);

		swerveEnclosure0 = new CanTalonSwerveEnclosure("enc 0", driveMotor0, steerMotor0, GEAR_RATIO);
		swerveEnclosure1 = new CanTalonSwerveEnclosure("enc 1", driveMotor1, steerMotor1, GEAR_RATIO);
		swerveEnclosure2 = new CanTalonSwerveEnclosure("enc 2", driveMotor2, steerMotor2, GEAR_RATIO);
		swerveEnclosure3 = new CanTalonSwerveEnclosure("enc 3", driveMotor3, steerMotor3, GEAR_RATIO);

		swerveEnclosure0.setReverseSteerMotor(true);
		swerveEnclosure1.setReverseSteerMotor(true);
		swerveEnclosure2.setReverseSteerMotor(true);
		swerveEnclosure3.setReverseSteerMotor(true);

		swerveEnclosure0.setReverseEncoder(true);
		swerveEnclosure1.setReverseEncoder(true);
		swerveEnclosure2.setReverseEncoder(true);
		swerveEnclosure3.setReverseEncoder(true);

		swerveDrive = new SwerveDrive(swerveEnclosure0, swerveEnclosure1, swerveEnclosure2, swerveEnclosure3, W, L);
		swerveDrive.setCentricMode(centricMode);
		resetEncoders();
		calibrateGyro();
	}

	public void drive(double fwd, double strafe, double rotateCW) {
		if (centricMode == CentricMode.ROBOT) {
			if (beakIsFront) {
				swerveDrive.move(fwd, strafe, rotateCW, getHeading());
			} else {
				swerveDrive.move(-fwd, -strafe, rotateCW, getHeading());
			}
		} else {
			swerveDrive.move(-fwd, -strafe, rotateCW, getHeading());
		}
	}

	public double[] getWheelAngles() {
		wheelAngles[0] = steerMotor0.getSelectedSensorPosition();
		wheelAngles[1] = steerMotor1.getSelectedSensorPosition();
		wheelAngles[2] = steerMotor2.getSelectedSensorPosition();
		wheelAngles[3] = steerMotor3.getSelectedSensorPosition();
		return wheelAngles;
	}

	public double getHeading() {
		return gyro.getAngle() % 360;
	}

	public void calibrateGyro() {
		System.out.println("Calibrating the gyro...");
		gyro.calibrate();
		System.out.println("Done calibrating the gyro.");
	}

	public void initDefaultCommand() {
		setDefaultCommand(new TeleDrive(this));
	}

	public void resetEncoders() {
		swerveEnclosure1.setEncPosition(0);
		swerveEnclosure0.setEncPosition(0);
		swerveEnclosure3.setEncPosition(0);
		swerveEnclosure2.setEncPosition(0);
		System.out.println("Drivetrain encoders have been reset.");
	}

	public void setCentricMode(CentricMode mode) {
		swerveDrive.setCentricMode(mode);
		centricMode = mode;
	}

	public void setBeakAsFront() {
		beakIsFront = true;
	}

	public void setClawAsFront() {
		beakIsFront = false;
	}

	public boolean beakIsFront() {
		return beakIsFront;
	}

	public CentricMode getCentricMode() {
		return centricMode;
	}

}