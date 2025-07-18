/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import robotCore.Device;
import robotCore.Gyro;
import robotCore.Logger;
import robotCore.apriltags.ApriltagLocation;
import robotCore.apriltags.ApriltagLocations;
import robotCore.apriltags.ApriltagsCamera;
import robotCore.apriltags.ApriltagsCamera.ApriltagsCameraType;
import robotCore.apriltags.PositionServer;
import robotCore.Swerve;

public class DriveSubsystem extends SubsystemBase {

    // SwerveBot07 (Yellow Robot new design / new motors)
    // private final boolean k_invertGyro = false;
    // private final double k_turnDeadZone = 3;
    // private final double k_turnP = 10.0 / 180.0;
    // private final double k_cameraAngle = 0;
    // private final double k_wheelSeparation = (7.406 * 2.54) / 100;
    // private final double k_driveBaseRadius = Math.sqrt(k_wheelSeparation * k_wheelSeparation);
    // static final double k_ticksPerMeter = 1064 / 1.8;

    // private final Translation2d m_frontLeftLocation = new Translation2d(k_wheelSeparation / 2, k_wheelSeparation / 2);
    // private final Translation2d m_backLeftLocation = new Translation2d(-k_wheelSeparation / 2, k_wheelSeparation / 2);
    // private final Translation2d m_backRightLocation = new Translation2d(-k_wheelSeparation / 2, -k_wheelSeparation / 2);
    // private final Translation2d m_frontRightLocation = new Translation2d(k_wheelSeparation / 2, -k_wheelSeparation / 2);

    // private static final double k_frontLeftMinSteeringPower = 0.32;
    // private static final double k_backLeftMinSteeringPower = 0.32;
    // private static final double k_backRightMinSteeringPower = 0.32;
    // private static final double k_frontRightMinsteeringPower = 0.34;

    // private static final int k_frontLeftSteeringZero = 1350; //1381;
    // private static final int k_backLeftSteeringZero = 190; //297;
    // private static final int k_backRightSteeringZero = 1305; //1273;
    // private static final int k_frontRightSteeringZero = -2131; //-2122;

    // public static final double k_maxDriveSpeed = 630;
    // private static final double k_frontLeftMinDrivePower = 0.32;
    // private static final double k_backLeftMinDrivePower = 0.31;
    // private static final double k_backRightMinDrivePower = 0.32;
    // private static final double k_frontRightMinDrivePower = 0.23;

    // private static final double k_frontLeftDriveF = 1.0 / k_maxDriveSpeed;
    // private static final double k_backLeftDriveF = 1.0 / k_maxDriveSpeed;
    // private static final double k_backRightDriveF = 0.95 / k_maxDriveSpeed;
    // private static final double k_frontRightDriveF = 0.95 / k_maxDriveSpeed;

    // public static final double k_drivePTerm = 0.001;
    // public static final double k_driveITerm = 0; //.00005; //.00005;
    // public static final double k_driveIZone = 200;

    // private static final double k_steeringP = 0.9;
    // private static final double k_frontLeftSteeringP = k_steeringP / 360;
    // private static final double k_backLeftSteeringP = k_steeringP / 360;
    // private static final double k_backRightSteeringP = k_steeringP / 360;
    // private static final double k_frontRightSteeringP = k_steeringP / 360;

    // private static final double k_steerintI = 0.02;
    // private static final double k_frontLeftSteeringD = k_steerintI;
    // private static final double k_backLeftSteeringD = k_steerintI;
    // private static final double k_backRightSteeringD = k_steerintI;
    // private static final double k_frontRightSteeringD = k_steerintI;

    // SwerveBot07 (Grey Robot new design)
    // private final boolean k_invertGyro = false;
    // private final double k_turnDeadZone = 3;
    // private final double k_turnP = 20.0 / 180.0;
    // private final double k_cameraAngle = 0;
    // private final double k_wheelSeparation = (7.406 * 2.54) / 100;
    // private final double k_driveBaseRadius = Math.sqrt(k_wheelSeparation * k_wheelSeparation);
    // static final double k_ticksPerMeter = 3579 / 1.345;

    // private final Translation2d m_frontLeftLocation = new Translation2d(k_wheelSeparation / 2, k_wheelSeparation / 2);
    // private final Translation2d m_backLeftLocation = new Translation2d(-k_wheelSeparation / 2, k_wheelSeparation / 2);
    // private final Translation2d m_backRightLocation = new Translation2d(-k_wheelSeparation / 2, -k_wheelSeparation / 2);
    // private final Translation2d m_frontRightLocation = new Translation2d(k_wheelSeparation / 2, -k_wheelSeparation / 2);

    // private static final double k_frontLeftMinSteeringPower = 0.34;
    // private static final double k_backLeftMinSteeringPower = 0.3;
    // private static final double k_backRightMinSteeringPower = 0.35;
    // private static final double k_frontRightMinsteeringPower = 0.28;

    // private static final int k_frontLeftSteeringZero = 1373;
    // private static final int k_backLeftSteeringZero = -2021;
    // private static final int k_backRightSteeringZero = -849;
    // private static final int k_frontRightSteeringZero = 219;

    // public static final double k_maxDriveSpeed = 2400;
    // private static final double k_frontLeftMinDrivePower = 0.31;
    // private static final double k_backLeftMinDrivePower = 0.31;
    // private static final double k_backRightMinDrivePower = 0.31;
    // private static final double k_frontRightMinDrivePower = 0.31;

    // private static final double k_frontLeftDriveF = 1.0 / k_maxDriveSpeed;
    // private static final double k_backLeftDriveF = 1.0 / k_maxDriveSpeed;
    // private static final double k_backRightDriveF = 1.0 / k_maxDriveSpeed;
    // private static final double k_frontRightDriveF = 1.0 / k_maxDriveSpeed;

    // public static final double k_drivePTerm = 0.0002;
    // public static final double k_driveITerm = 0.00003;
    // public static final double k_driveIZone = 200;

    // private static final double k_frontLeftSteeringP = 0.90 / 360;
    // private static final double k_backLeftSteeringP = 0.90 / 360;
    // private static final double k_backRightSteeringP = 0.80 / 360;
    // private static final double k_frontRightSteeringP = 0.90 / 360;

    // private static final double k_frontLeftSteeringD = 0.01;
    // private static final double k_backLeftSteeringD = 0.005;
    // private static final double k_backRightSteeringD = 0.01;
    // private static final double k_frontRightSteeringD = 0.01;

  // SwerveBot05 (Black Robot new design)
  private static final boolean k_invertGyro = false;
  private static final double k_turnDeadZone = 3;
  private static final double k_turnP = 20.0 / 180.0;
  private static final double k_cameraAngle = 0;
  private static final double k_wheelSeparation = (7.406 * 2.54) / 100;
  private static final double k_driveBaseRadius = Math.sqrt(k_wheelSeparation * k_wheelSeparation);
  public static final double k_ticksPerMeter = 3579 / 1.345;

  private final Translation2d m_frontLeftLocation = new Translation2d(k_wheelSeparation/2, k_wheelSeparation/2);
  private final Translation2d m_backLeftLocation = new Translation2d(-k_wheelSeparation/2, k_wheelSeparation/2);
  private final Translation2d m_backRightLocation = new Translation2d(-k_wheelSeparation/2, -k_wheelSeparation/2);
  private final Translation2d m_frontRightLocation = new Translation2d(k_wheelSeparation/2, -k_wheelSeparation/2);

  private static final double k_frontLeftMinSteeringPower = 0.30;
  private static final double k_backLeftMinSteeringPower = 0.28;
  private static final double k_backRightMinSteeringPower = 0.32;
  private static final double k_frontRightMinsteeringPower = 0.33;

  private static final int k_frontLeftSteeringZero = 1409;
  private static final int k_backLeftSteeringZero = 262;
  private static final int k_backRightSteeringZero = 1379;
  private static final int k_frontRightSteeringZero = -2001;

  public static final double k_maxDriveSpeed = 2200;
  private static final double k_frontLeftMinDrivePower = 0.31;
  private static final double k_backLeftMinDrivePower = 0.31;
  private static final double k_backRightMinDrivePower = 0.30;
  private static final double k_frontRightMinDrivePower = 0.31;

  private static final double k_frontLeftDriveF = 0.911 / k_maxDriveSpeed;
  private static final double k_backLeftDriveF = 0.865 / k_maxDriveSpeed;
  private static final double k_backRightDriveF = 0.882 / k_maxDriveSpeed;
  private static final double k_frontRightDriveF = 0.987 / k_maxDriveSpeed;

  public static final double k_drivePTerm = 0.0004;
  public static final double k_driveITerm = 0.00005;
  public static final double k_driveIZone = 200;

  private static final double k_frontLeftSteeringP = 0.90 / 360;
  private static final double k_backLeftSteeringP = 0.90 / 360;
  private static final double k_backRightSteeringP = 0.80 / 360;
  private static final double k_frontRightSteeringP = 0.90 / 360;

  private static final double k_frontLeftSteeringD = 0.01;
  private static final double k_backLeftSteeringD = 0.005;
  private static final double k_backRightSteeringD = 0.01;
  private static final double k_frontRightSteeringD = 0.01;

  // SwerveBot05 (White Robot new design)
  // private final double k_cameraAngle = 3.5;
  // private final boolean k_invertGyro = false;
  // private final double k_turnDeadZone = 3;
  // private final double k_turnP = 20.0 / 180.0;
  // private final double k_wheelSeparation = (7.406 * 2.54) / 100;
  // private final double k_driveBaseRadius = Math.sqrt(k_wheelSeparation * k_wheelSeparation);
  // static final double k_ticksPerMeter = 3579 / 1.345;


  // private final Translation2d m_frontLeftLocation = new Translation2d(k_wheelSeparation/2, k_wheelSeparation/2);
  // private final Translation2d m_backLeftLocation = new Translation2d(-k_wheelSeparation/2, k_wheelSeparation/2);
  // private final Translation2d m_backRightLocation = new Translation2d(-k_wheelSeparation/2, -k_wheelSeparation/2);
  // private final Translation2d m_frontRightLocation = new Translation2d(k_wheelSeparation/2, -k_wheelSeparation/2);

  // private static final double k_frontLeftMinSteeringPower = 0.33;
  // private static final double k_backLeftMinSteeringPower = 0.33;
  // private static final double k_backRightMinSteeringPower = 0.31;
  // private static final double k_frontRightMinsteeringPower = 0.31;

  // private static final int k_frontLeftSteeringZero = -928; //-853; // 1362; //1363;
  // private static final int k_backLeftSteeringZero = 234; //239; // -2007; //-2010;
  // private static final int k_backRightSteeringZero = 1351; //1404; // 1700; //1639;
  // private static final int k_frontRightSteeringZero = -1990; //-2027; // 224; //403;

  // public static final double k_maxDriveSpeed = 2400;
  // private static final double k_frontLeftMinDrivePower = 0.31;
  // private static final double k_backLeftMinDrivePower = 0.32;
  // private static final double k_backRightMinDrivePower = 0.31;
  // private static final double k_frontRightMinDrivePower = 0.27;

  // private static final double k_frontLeftDriveF = 0.9 / k_maxDriveSpeed;
  // private static final double k_backLeftDriveF = 0.9 / k_maxDriveSpeed;
  // private static final double k_backRightDriveF = 1.3 / k_maxDriveSpeed;
  // private static final double k_frontRightDriveF = 1.0 / k_maxDriveSpeed;

  // public static final double k_drivePTerm = 0.001;
  // public static final double k_driveITerm = 0.0001;
  // public static final double k_driveIZone = 200;

  // private static final double k_frontLeftSteeringP = 1.3 / 360;
  // private static final double k_backLeftSteeringP = 0.90 / 360;
  // private static final double k_backRightSteeringP = 0.90 / 360;
  // private static final double k_frontRightSteeringP = 0.90 / 360;

  // private static final double k_frontLeftSteeringD = 0.02;
  // private static final double k_backLeftSteeringD = 0.01;
  // private static final double k_backRightSteeringD = 0.02;
  // private static final double k_frontRightSteeringD = 0.01;

  // SwerveBot04 (Blue Robot new design)
  // private final boolean k_invertGyro = false;
  // private final double k_turnDeadZone = 3;
  // private final double k_turnP = 20.0 / 180.0;

  // private final Translation2d m_frontLeftLocation = new
  // Translation2d(0.0940562, 0.0940562);
  // private final Translation2d m_backLeftLocation = new
  // Translation2d(-0.0940562, 0.0940562);
  // private final Translation2d m_backRightLocation = new
  // Translation2d(-0.0940562, -0.0940562);
  // private final Translation2d m_frontRightLocation = new
  // Translation2d(0.0940562, -0.0940562);

  // private static final double k_frontLeftMinSteeringPower = 0.35;
  // private static final double k_backLeftMinSteeringPower = 0.35;
  // private static final double k_backRightMinSteeringPower = 0.35;
  // private static final double k_frontRightMinsteeringPower = 0.35;

  // private static final int k_frontLeftSteeringZero = -918; //1362; //1363;
  // private static final int k_backLeftSteeringZero = 214; //-2007; //-2010;
  // private static final int k_backRightSteeringZero = -826; //1700; //1639;
  // private static final int k_frontRightSteeringZero = -2020; //224; //403;

  // static final double k_maxDriveSpeed = 2500;
  // private static final double k_frontLeftMinDrivePower = 0.28;
  // private static final double k_backLeftMinDrivePower = 0.28;
  // private static final double k_backRightMinDrivePower = 0.28;
  // private static final double k_frontRightMinDrivePower = 0.28;

  // private static final double k_frontLeftDriveF = 1.0 / k_maxDriveSpeed;
  // private static final double k_backLeftDriveF = 1.0 / k_maxDriveSpeed;
  // private static final double k_backRightDriveF = 1.0 / k_maxDriveSpeed;
  // private static final double k_frontRightDriveF = 1.0 / k_maxDriveSpeed;

  // public static final double k_drivePTerm = 0.0002;
  // public static final double k_driveITerm = 0.00005;
  // public static final double k_driveIZone = 200;

  // private static final double k_frontLeftSteeringP = 0.90 / 360;
  // private static final double k_backLeftSteeringP = 0.90 / 360;
  // private static final double k_backRightSteeringP = 0.80 / 360;
  // private static final double k_frontRightSteeringP = 0.90 / 360;

  // private static final double k_frontLeftSteeringD = 0.02;
  // private static final double k_backLeftSteeringD = 0.02;
  // private static final double k_backRightSteeringD = 0.02;
  // private static final double k_frontRightSteeringD = 0.02;

  // SwerveBot05 (Black Robot New Motors)
  // private final double k_turnDeadZone = 3;
  // private final double k_turnP = 20.0 / 180.0;

  // private final Translation2d m_frontLeftLocation = new
  // Translation2d(0.0940562, 0.0940562);
  // private final Translation2d m_backLeftLocation = new
  // Translation2d(-0.0940562, 0.0940562);
  // private final Translation2d m_backRightLocation = new
  // Translation2d(-0.0940562, -0.0940562);
  // private final Translation2d m_frontRightLocation = new
  // Translation2d(0.0940562, -0.0940562);

  // private static final double k_frontLeftMinSteeringPower = 0.24;
  // private static final double k_backLeftMinSteeringPower = 0.25;
  // private static final double k_backRightMinSteeringPower = 0.27;
  // private static final double k_frontRightMinsteeringPower = 0.30;

  // private static final int k_frontLeftSteeringZero = 1327;
  // private static final int k_backLeftSteeringZero = 212;
  // private static final int k_backRightSteeringZero = 1358;
  // private static final int k_frontRightSteeringZero = -1900; //-2089;

  // static final double k_maxDriveSpeed = 1700;
  // private static final double k_frontLeftMinDrivePower = 0.34;
  // private static final double k_backLeftMinDrivePower = 0.34;
  // private static final double k_backRightMinDrivePower = 0.35;
  // private static final double k_frontRightMinDrivePower = 0.35;

  // private static final double k_frontLeftDriveF = 0.85 / k_maxDriveSpeed;
  // private static final double k_backLeftDriveF = 0.85 / k_maxDriveSpeed;
  // private static final double k_backRightDriveF = 0.97 / k_maxDriveSpeed;
  // private static final double k_frontRightDriveF = 0.97 / k_maxDriveSpeed;

  // public static final double k_drivePTerm = 0.0002;
  // public static final double k_driveITerm = 0.00005;
  // public static final double k_driveIZone = 300;

  // private static final double k_frontLeftSteeringP = 1.0 / 360;
  // private static final double k_backLeftSteeringP = 1.0 / 360;
  // private static final double k_backRightSteeringP = 1.0 / 360;
  // private static final double k_frontRightSteeringP = 1.0 / 360;

  // private static final double k_frontLeftSteeringD = 0.010;
  // private static final double k_backLeftSteeringD = 0.010;
  // private static final double k_backRightSteeringD = 0.010;
  // private static final double k_frontRightSteeringD = 0.010;

  // SwerveBot05 (Black Robot Original Motors)
  // private final Translation2d m_frontLeftLocation = new
  // Translation2d(0.0940562, 0.0940562);
  // private final Translation2d m_backLeftLocation = new
  // Translation2d(-0.0940562, 0.0940562);
  // private final Translation2d m_backRightLocation = new
  // Translation2d(-0.0940562, -0.0940562);
  // private final Translation2d m_frontRightLocation = new
  // Translation2d(0.0940562, -0.0940562);

  // private static final double k_frontLeftMinSteeringPower = 0.31;
  // private static final double k_backLeftMinSteeringPower = 0.33;
  // private static final double k_backRightMinSteeringPower = 0.32;
  // private static final double k_frontRightMinsteeringPower = 0.42;

  // private static final int k_frontLeftSteeringZero = 1467;
  // private static final int k_backLeftSteeringZero = 227;
  // private static final int k_backRightSteeringZero = 1360;
  // private static final int k_frontRightSteeringZero = -2020;

  // static final double k_maxDriveSpeed = 2400;
  // private static final double k_frontLeftMinDrivePower = 0.27;
  // private static final double k_backLeftMinDrivePower = 0.28;
  // private static final double k_backRightMinDrivePower = 0.28;
  // private static final double k_frontRightMinDrivePower = 0.28;

  // private static final double k_frontLeftDriveF = 1.03 / k_maxDriveSpeed;
  // private static final double k_backLeftDriveF = 1.03 / k_maxDriveSpeed;
  // private static final double k_backRightDriveF = 1.03 / k_maxDriveSpeed;
  // private static final double k_frontRightDriveF = 1.03 / k_maxDriveSpeed;

  // public static final double k_drivePTerm = 0.0002;
  // public static final double k_driveITerm = 0.00005;
  // public static final double k_driveIZone = 200;

  // private static final double k_frontLeftSteeringP = 1.0 / 360;
  // private static final double k_backLeftSteeringP = 1.0 / 360;
  // private static final double k_backRightSteeringP = 1.0 / 360;
  // private static final double k_frontRightSteeringP = 1.0 / 360;

  // private static final double k_frontLeftSteeringD = 0.010;
  // private static final double k_backLeftSteeringD = 0.010;
  // private static final double k_backRightSteeringD = 0.010;
  // private static final double k_frontRightSteeringD = 0.010;

  // SwerveBot01 (White Robot)
  // private static final double k_frontLeftMinSteeringPower = 0.33;
  // private static final double k_backLeftMinSteeringPower = 0.33;
  // private static final double k_backRightMinSteeringPower = 0.30;
  // private static final double k_frontRightMinsteeringPower = 0.30;

  // private static final int k_frontLeftSteeringZero = 255;
  // private static final int k_backLeftSteeringZero = 326;
  // private static final int k_backRightSteeringZero = -2019;
  // private static final int k_frontRightSteeringZero = -1965;

  // private static final double k_frontLeftSteeringP = 0.8 / 360;
  // private static final double k_backLeftSteeringP = 0.8 / 360;
  // private static final double k_backRightSteeringP = 0.9 / 360;
  // private static final double k_frontRightSteeringP = 0.8 / 360;

  // private static final double k_frontLeftSteeringD = 0.005;
  // private static final double k_backLeftSteeringD = 0.005;
  // private static final double k_backRightSteeringD = 0.005;
  // private static final double k_frontRightSteeringD = 0.005;

  // static final double k_maxDriveSpeed = 2500;
  // private static final double k_frontLeftMinDrivePower = 0.38;
  // private static final double k_backLeftMinDrivePower = 0.38;
  // private static final double k_backRightMinDrivePower = 0.38;
  // private static final double k_frontRightMinDrivePower = 0.38;

  // private static final double k_frontLeftDriveF = 1.05 / k_maxDriveSpeed;
  // private static final double k_backLeftDriveF = 1.05 / k_maxDriveSpeed;
  // private static final double k_backRightDriveF = 0.95 / k_maxDriveSpeed;
  // private static final double k_frontRightDriveF = 0.95 / k_maxDriveSpeed;

  // public static final double k_drivePTerm = 0.0010;
  // public static final double k_driveITerm = 0.0004;
  // public static final double k_driveIZone = 200;

  // SwerveBot02 (Yellow Robot)
  // private final boolean k_invertGyro = true;
  // private final double k_turnDeadZone = 3;
  // private final double k_turnP = 20.0 / 180.0;
  // private final double k_cameraAngle = 0;

  // private final Translation2d m_frontLeftLocation = new Translation2d(0.05842, 0.05842);
  // private final Translation2d m_backLeftLocation = new Translation2d(-0.05842, 0.05842);
  // private final Translation2d m_backRightLocation = new Translation2d(-0.05842, -0.05842);
  // private final Translation2d m_frontRightLocation = new Translation2d(0.05842, -0.05842);
  // private final double k_wheelSeparation = (7.406 * 2.54) / 100;
  // private final double k_driveBaseRadius = Math.sqrt(k_wheelSeparation * k_wheelSeparation);

  // private static final double k_frontLeftMinSteeringPower = 0.33;
  // private static final double k_backLeftMinSteeringPower = 0.35;
  // private static final double k_backRightMinSteeringPower = 0.28;
  // private static final double k_frontRightMinsteeringPower = 0.31;

  // private static final int k_frontLeftSteeringZero = -766;
  // private static final int k_backLeftSteeringZero = -682;
  // private static final int k_backRightSteeringZero = -1865;
  // private static final int k_frontRightSteeringZero = -1833;

  // static final double k_maxDriveSpeed = 2500;
  // private static final double k_frontLeftMinDrivePower = 0.30;
  // private static final double k_backLeftMinDrivePower = 0.29;
  // private static final double k_backRightMinDrivePower = 0.28;
  // private static final double k_frontRightMinDrivePower = 0.29;

  // public static final double k_drivePTerm = 0.0010;
  // public static final double k_driveITerm = 0.0001;
  // public static final double k_driveIZone = 200;

  // private static final double k_frontLeftDriveF = 0.9 / k_maxDriveSpeed;
  // private static final double k_backLeftDriveF = 1.0 / k_maxDriveSpeed;
  // private static final double k_backRightDriveF = 0.9 / k_maxDriveSpeed;
  // private static final double k_frontRightDriveF = 0.9 / k_maxDriveSpeed;

  // private static final double k_frontLeftSteeringP = 0.8 / 360;
  // private static final double k_backLeftSteeringP = 0.8 / 360;
  // private static final double k_backRightSteeringP = 0.9 / 360;
  // private static final double k_frontRightSteeringP = 0.8 / 360;

  // private static final double k_frontLeftSteeringD = 0.005;
  // private static final double k_backLeftSteeringD = 0.005;
  // private static final double k_backRightSteeringD = 0.005;
  // private static final double k_frontRightSteeringD = 0.005;

  // SwerveBot03 (Red Robot)
  // private final boolean k_invertGyro = false;
  // private final double k_turnDeadZone = 3;
  // private final double k_turnP = 20.0 / 180.0;

  // private final Translation2d m_frontLeftLocation = new Translation2d(0.05842,
  // 0.05842);
  // private final Translation2d m_backLeftLocation = new Translation2d(-0.05842,
  // 0.05842);
  // private final Translation2d m_backRightLocation = new Translation2d(-0.05842,
  // -0.05842);
  // private final Translation2d m_frontRightLocation = new Translation2d(0.05842,
  // -0.05842);

  // private static final double k_frontLeftMinSteeringPower = 0.27;
  // private static final double k_backLeftMinSteeringPower = 0.31;
  // private static final double k_backRightMinSteeringPower = 0.35;
  // private static final double k_frontRightMinsteeringPower = 0.36;

  // private static final int k_frontLeftSteeringZero = 373;
  // private static final int k_backLeftSteeringZero = 1560;
  // private static final int k_backRightSteeringZero = 444;
  // private static final int k_frontRightSteeringZero = -1861;

  // static final double k_maxDriveSpeed = 2400;
  // private static final double k_frontLeftMinDrivePower = 0.33;
  // private static final double k_backLeftMinDrivePower = 0.33;
  // private static final double k_backRightMinDrivePower = 0.34;
  // private static final double k_frontRightMinDrivePower = 0.34;

  // public static final double k_drivePTerm = 0.0010;
  // public static final double k_driveITerm = 0.0001;
  // public static final double k_driveIZone = 200;

  // private static final double k_frontLeftDriveF = 0.95 / k_maxDriveSpeed;
  // private static final double k_backLeftDriveF = 0.95 / k_maxDriveSpeed;
  // private static final double k_backRightDriveF = 1 / k_maxDriveSpeed;
  // private static final double k_frontRightDriveF = 1 / k_maxDriveSpeed;

  // private static final double k_frontLeftSteeringP = 0.8 / 360;
  // private static final double k_backLeftSteeringP = 0.8 / 360;
  // private static final double k_backRightSteeringP = 0.9 / 360;
  // private static final double k_frontRightSteeringP = 0.8 / 360;

  // private static final double k_frontLeftSteeringD = 0.005;
  // private static final double k_backLeftSteeringD = 0.005;
  // private static final double k_backRightSteeringD = 0.005;
  // private static final double k_frontRightSteeringD = 0.005;

  // // SwerveBot04 (Blue Robot)
  // private static final double k_frontLeftMinSteeringPower = 0.33;
  // private static final double k_backLeftMinSteeringPower = 0.33;
  // private static final double k_backRightMinSteeringPower = 0.30;
  // private static final double k_frontRightMinsteeringPower = 0.30;

  // private static final int k_frontLeftSteeringZero = -1950;
  // private static final int k_backLeftSteeringZero = -764;
  // private static final int k_backRightSteeringZero = -1940;
  // private static final int k_frontRightSteeringZero = 1470;

  // static final double k_maxDriveSpeed = 2500;
  // private static final double k_frontLeftMinDrivePower = 0.29;
  // private static final double k_backLeftMinDrivePower = 0.29;
  // private static final double k_backRightMinDrivePower = 0.30;
  // private static final double k_frontRightMinDrivePower = 0.30;

  // private static final double k_frontLeftDriveF = 1.05 / k_maxDriveSpeed;
  // private static final double k_backLeftDriveF = 1.05 / k_maxDriveSpeed;
  // private static final double k_backRightDriveF = 0.95 / k_maxDriveSpeed;
  // private static final double k_frontRightDriveF = 0.95 / k_maxDriveSpeed;

  // public static final double k_drivePTerm = 0.0010;
  // public static final double k_driveITerm = 0.0004;
  // public static final double k_driveIZone = 200;

  // private static final double k_frontLeftSteeringP = 0.8 / 360;
  // private static final double k_backLeftSteeringP = 0.8 / 360;
  // private static final double k_backRightSteeringP = 0.9 / 360;
  // private static final double k_frontRightSteeringP = 0.8 / 360;

  // private static final double k_frontLeftSteeringD = 0.005;
  // private static final double k_backLeftSteeringD = 0.005;
  // private static final double k_backRightSteeringD = 0.005;
  // private static final double k_frontRightSteeringD = 0.005;

  //////////////////////

  // Blue/White Robot
  // private static final double k_frontLeftMinSteeringPower = 0.33;
  // private static final double k_backLeftMinSteeringPower = 0.33;
  // private static final double k_backRightMinSteeringPower = 0.30;
  // private static final double k_frontRightMinsteeringPower = 0.30;

  // Blue Robot
  // private static final int k_frontLeftSteeringZero = -1950;
  // private static final int k_backLeftSteeringZero = -764;
  // private static final int k_backRightSteeringZero = -1940;
  // private static final int k_frontRightSteeringZero = 1470;

  // White Robot
  // private static final int k_frontLeftSteeringZero = 255;
  // private static final int k_backLeftSteeringZero = 326;
  // private static final int k_backRightSteeringZero = -2019;
  // private static final int k_frontRightSteeringZero = -1965;

  // private static final double k_frontLeftSteeringP = 0.8 / 360;
  // private static final double k_backLeftSteeringP = 0.8 / 360;
  // private static final double k_backRightSteeringP = 0.9 / 360;
  // private static final double k_frontRightSteeringP = 0.8 / 360;

  // private static final double k_frontLeftSteeringD = 0.005;
  // private static final double k_backLeftSteeringD = 0.005;
  // private static final double k_backRightSteeringD = 0.005;
  // private static final double k_frontRightSteeringD = 0.005;

  // Blue Robot
  // static final double k_maxDriveSpeed = 2500;
  // private static final double k_frontLeftMinDrivePower = 0.29;
  // private static final double k_backLeftMinDrivePower = 0.29;
  // private static final double k_backRightMinDrivePower = 0.30;
  // private static final double k_frontRightMinDrivePower = 0.30;

  // White Robot
  // static final double k_maxDriveSpeed = 2500;
  // private static final double k_frontLeftMinDrivePower = 0.38;
  // private static final double k_backLeftMinDrivePower = 0.38;
  // private static final double k_backRightMinDrivePower = 0.38;
  // private static final double k_frontRightMinDrivePower = 0.38;

  // static final double k_ticksPerMeter = 3579 / 1.345;
  public static final double k_maxDriveSpeedMetersPerSecond = k_maxDriveSpeed / k_ticksPerMeter; // = 0.9395 m/s
  public static final double k_maxAngularSpeed = 2.0; // radians / sec

  // White/Blue
  // private static final double k_frontLeftDriveF = 1.05 / k_maxDriveSpeed;
  // private static final double k_backLeftDriveF = 1.05 / k_maxDriveSpeed;
  // private static final double k_backRightDriveF = 0.95 / k_maxDriveSpeed;
  // private static final double k_frontRightDriveF = 0.95 / k_maxDriveSpeed;

  // White/Blue
  // public static final double k_drivePTerm = 0.0010;
  // public static final double k_driveITerm = 0.0004;
  // public static final double k_driveIZone = 200;

  private final Gyro m_gyro = new Gyro();
  private ApriltagsCamera m_camera = new ApriltagsCamera();
  private final PositionServer m_posServer = new PositionServer(m_camera);

  SwerveModule m_frontLeft = new SwerveModule(Swerve.FLDrivePWM, Swerve.FLDriveDir, Swerve.FLDriveEncInt,
      Swerve.FLDriveEncDir, Swerve.FLSteeringPWM, Swerve.FLSteeringDir, Swerve.FLSteeringEncA, Swerve.FLSteeringEncB, Swerve.FLI2CAddr,
      "FrontLeft");
  SwerveModule m_backLeft = new SwerveModule(Swerve.BLDrivePWM, Swerve.BLDriveDir, Swerve.BLDriveEncInt,
      Swerve.BLDriveEncDir, Swerve.BLSteeringPWM, Swerve.BLSteeringDir, Swerve.BLSteeringEncA, Swerve.BLSteeringEncB, Swerve.BLI2CAddr,
      "BackLeft");
  SwerveModule m_backRight = new SwerveModule(Swerve.BRDrivePWM, Swerve.BRDriveDir, Swerve.BRDriveEncInt,
      Swerve.BRDriveEncDir, Swerve.BRSteeringPWM, Swerve.BRSteeringDir, Swerve.BRSteeringEncA, Swerve.BRSteeringEncB, Swerve.BRI2CAddr,
      "BackRight");
  SwerveModule m_frontRight = new SwerveModule(Swerve.FRDrivePWM, Swerve.FRDriveDir, Swerve.FRDriveEncInt,
      Swerve.FRDriveEncDir, Swerve.FRSteeringPWM, Swerve.FRSteeringDir, Swerve.FRSteeringEncA, Swerve.FRSteeringEncB, Swerve.FRI2CAddr,
      "FrontRight");

  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
      m_frontLeftLocation, m_backLeftLocation, m_backRightLocation, m_frontRightLocation);

  private SwerveDrivePoseEstimator m_poseEstimator;

  // private final double k_turnDeadZone = 3;
  // private final double k_turnP = 20.0 / 180.0;

  private static final String k_cameraIP = "127.0.1.1";
  private static final int k_cameraPort = 5800;

  public static ApriltagLocation m_aprilTags[] = {
      new ApriltagLocation(1, 3, 2, 90),
      new ApriltagLocation(2, 4, 3, 180),
      new ApriltagLocation(3, 3, 4, -90),
      new ApriltagLocation(4, 3, 3, 0),
  };

  /**
   * Creates a new DriveSubsystem.
   */
  public DriveSubsystem() {
    Logger.log("DriveSubsystem", 3, "DriveSubsystem()");

    m_frontLeft.setSteeringMinPower(k_frontLeftMinSteeringPower);
    m_backLeft.setSteeringMinPower(k_backLeftMinSteeringPower);
    m_backRight.setSteeringMinPower(k_backRightMinSteeringPower);
    m_frontRight.setSteeringMinPower(k_frontRightMinsteeringPower);

    m_frontLeft.setSteeringZero(k_frontLeftSteeringZero);
    m_backLeft.setSteeringZero(k_backLeftSteeringZero);
    m_backRight.setSteeringZero(k_backRightSteeringZero);
    m_frontRight.setSteeringZero(k_frontRightSteeringZero);

    m_frontLeft.setSteeringPTerm(k_frontLeftSteeringP);
    m_frontLeft.setSteeringDTerm(k_frontLeftSteeringD);

    m_backLeft.setSteeringPTerm(k_backLeftSteeringP);
    m_backLeft.setSteeringDTerm(k_backLeftSteeringD);

    m_backRight.setSteeringPTerm(k_backRightSteeringP);
    m_backRight.setSteeringDTerm(k_backRightSteeringD);

    m_frontRight.setSteeringPTerm(k_frontRightSteeringP);
    m_frontRight.setSteeringDTerm(k_frontRightSteeringD);

    m_frontLeft.setDriveMinPower(k_frontLeftMinDrivePower);
    m_backLeft.setDriveMinPower(k_backLeftMinDrivePower);
    m_backRight.setDriveMinPower(k_backRightMinDrivePower);
    m_frontRight.setDriveMinPower(k_frontRightMinDrivePower);

    m_frontLeft.setDriveF(k_frontLeftDriveF);
    m_backLeft.setDriveF(k_backLeftDriveF);
    m_backRight.setDriveF(k_backRightDriveF);
    m_frontRight.setDriveF(k_frontRightDriveF);

    m_frontLeft.setDriveP(k_drivePTerm);
    m_backLeft.setDriveP(k_drivePTerm);
    m_backRight.setDriveP(k_drivePTerm);
    m_frontRight.setDriveP(k_drivePTerm);

    m_frontLeft.setDriveI(k_driveITerm);
    m_backLeft.setDriveI(k_driveITerm);
    m_backRight.setDriveI(k_driveITerm);
    m_frontRight.setDriveI(k_driveITerm);

    m_frontLeft.setDriveIZone(k_driveIZone);
    m_backLeft.setDriveIZone(k_driveIZone);
    m_backRight.setDriveIZone(k_driveIZone);
    m_frontRight.setDriveIZone(k_driveIZone);

    m_gyro.invert(k_invertGyro);
    m_gyro.reset(0);

    m_poseEstimator = new SwerveDrivePoseEstimator(m_kinematics, m_gyro.getRotation2d(), getModulePositions(),
        new Pose2d(1, 3, Rotation2d.fromDegrees(0)));

    ApriltagLocations.setLocations(m_aprilTags);
    m_camera.setCameraInfo(0, 5, k_cameraAngle, ApriltagsCameraType.PiCam_640x480);
    m_camera.connect(k_cameraIP, k_cameraPort);
    m_posServer.start();

    AutoBuilder.configureHolonomic(this::getPose2d,
        this::ResetPose,
        this::getChassisSpeeds,
        this::setChassisSpeeds,
        new HolonomicPathFollowerConfig(
            new PIDConstants(1.5, 0, 0),
            new PIDConstants(3, 0, 0),
            k_maxDriveSpeedMetersPerSecond,
            k_driveBaseRadius,
            new ReplanningConfig(false, false)),
        () -> false,
        this);

  }

  /*
   * Return the angle to the specified Apriltags target
   */
  public double getTargetAngle(int target) {
    ApriltagLocation location = ApriltagLocations.findTag(target);

    if (location == null) {
      throw new RuntimeException(String.format("Invalid target: %d", target));
    }

    Pose2d pose = getPose2d();
    double dx = location.m_xMeters - pose.getX();
    double dy = location.m_yMeters - pose.getY();

    return Math.toDegrees(Math.atan2(dy, dx));
  }

  /*
   * Compute the rotation rate needed to perform a turn to target maneuver
   */
  public double computeAutoAim(double targetAngleInDegrees) {
    Pose2d pos = getPose2d();

    double da = Gyro.normalizeAngle(pos.getRotation().getDegrees() - targetAngleInDegrees);

    // Logger.log("DriveSubsystem", 1, String.format("computeAutoAim: da=%f", da));

    if (Math.abs(da) > k_turnDeadZone) {
      return -k_turnP * da;
    }

    return 0; // On target
  }

  public Pose2d getPose2d() {
    return m_poseEstimator.getEstimatedPosition();
  }

  private void ResetPose(Pose2d pose) {
    m_poseEstimator.resetPosition(m_gyro.getRotation2d(), getModulePositions(), pose);
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = {
        m_frontLeft.getState(),
        m_backLeft.getState(),
        m_backRight.getState(),
        m_frontRight.getState()
    };
    return states;
  }

  public ChassisSpeeds getChassisSpeeds() {
    return m_kinematics.toChassisSpeeds(getModuleStates());
  }

  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    setModuleStates(m_kinematics.toSwerveModuleStates(chassisSpeeds));
  }

  private void setModuleStates(SwerveModuleState[] swerveModuleStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, k_maxDriveSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_backLeft.setDesiredState(swerveModuleStates[1]);
    m_backRight.setDesiredState(swerveModuleStates[2]);
    m_frontRight.setDesiredState(swerveModuleStates[3]);
  }

  private SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
        m_frontLeft.getPosition(),
        m_backLeft.getPosition(),
        m_backRight.getPosition(),
        m_frontRight.getPosition()
    };
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, double periodSeconds) {
    SwerveModuleState[] swerveModuleStates = m_kinematics.toSwerveModuleStates(
        ChassisSpeeds.discretize(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, ySpeed, rot, m_poseEstimator.getEstimatedPosition().getRotation())
                : new ChassisSpeeds(xSpeed, ySpeed, rot),
            periodSeconds));

    setModuleStates(swerveModuleStates);

    // Robot.sleep(1000);
  }

  public void stop() {
    m_frontLeft.stop();
    m_backLeft.stop();
    m_backRight.stop();
    m_frontRight.stop();
  }

  public SwerveModule getFrontLeftModule() {
    return m_frontLeft;
  }

  public SwerveModule getBackLeftModule() {
    return m_backLeft;
  }

  public SwerveModule getBackRightModule() {
    return m_backRight;
  }

  public SwerveModule getFrontRighModule() {
    return m_frontRight;
  }

  public void resetGyro() {
    m_gyro.reset(0);
  }

  int m_count = 0;

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Logger.log("DriveSubsystem", -1, "periodic()");

    m_poseEstimator.updateWithTime(ApriltagsCamera.getTime(), m_gyro.getRotation2d(), getModulePositions());
    m_posServer.setPosition(m_poseEstimator.getEstimatedPosition());
    m_camera.processRegions(m_poseEstimator);

    // if (++m_count == 30) {
    //   Logger.log("DriveSubsystem", 1, String.format("yaw=%f", m_gyro.getYaw()));
    //   m_count = 0;
    // }

    // Logger.log("DriveSubsystem", 1, String.format(",%f,%f,%f,%f",
    // m_frontLeft.getSteeringPosition(),
    // m_backLeft.getSteeringPosition(), m_backRight.getSteeringPosition(),
    // m_frontRight.getSteeringPosition()));
    // RobotBase.sleep(100);

    // Logger.log("DriveSubsystem", 1, String.format(",%f,%f,%f,%f",
    // m_frontLeft.getSteeringPositionInDegrees(),
    // m_backLeft.getSteeringPositionInDegrees(),
    // m_backRight.getSteeringPositionInDegrees(),
    // m_frontRight.getSteeringPositionInDegrees()));
  }
}
