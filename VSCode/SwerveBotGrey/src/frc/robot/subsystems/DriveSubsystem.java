/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import robotCore.Gyro;
import robotCore.Logger;
import robotCore.Swerve;

public class DriveSubsystem extends SubsystemBase {
  // SwerveBot06 (Grey Robot new design / new motors)
  private final boolean k_invertGyro = false;
  private final double k_wheelSeparation = (7.406 * 2.54) / 100;
  private final double k_driveBaseRadius = Math.sqrt(k_wheelSeparation * k_wheelSeparation);
  static final double k_ticksPerMeter = 1037.75 / 1.72;

  private final Translation2d m_frontLeftLocation = new Translation2d(k_wheelSeparation / 2, k_wheelSeparation / 2);
  private final Translation2d m_backLeftLocation = new Translation2d(-k_wheelSeparation / 2, k_wheelSeparation / 2);
  private final Translation2d m_backRightLocation = new Translation2d(-k_wheelSeparation / 2, -k_wheelSeparation / 2);
  private final Translation2d m_frontRightLocation = new Translation2d(k_wheelSeparation / 2, -k_wheelSeparation / 2);

  private static final double k_frontLeftMinSteeringPower = 0.34;
  private static final double k_backLeftMinSteeringPower = 0.30;
  private static final double k_backRightMinSteeringPower = 0.34;
  private static final double k_frontRightMinsteeringPower = 0.30;

  private static final int k_frontLeftSteeringZero = -888;
  private static final int k_backLeftSteeringZero = -2002;
  private static final int k_backRightSteeringZero = -888;
  private static final int k_frontRightSteeringZero = -1983;

  public static final double k_maxDriveSpeed = 710;
  private static final double k_frontLeftMinDrivePower = 0.28;
  private static final double k_backLeftMinDrivePower = 0.29;
  private static final double k_backRightMinDrivePower = 0.28;
  private static final double k_frontRightMinDrivePower = 0.28;

  private static final double k_frontLeftDriveF = 0.94 / k_maxDriveSpeed; //0.92 / k_maxDriveSpeed;
  private static final double k_backLeftDriveF = 0.90 / k_maxDriveSpeed; //0.90 / k_maxDriveSpeed;
  private static final double k_backRightDriveF = 0.96 / k_maxDriveSpeed; //0.92 / k_maxDriveSpeed;
  private static final double k_frontRightDriveF = 0.96 / k_maxDriveSpeed; //0.92 / k_maxDriveSpeed;

  public static final double k_drivePTerm = 0.002;
  public static final double k_driveITerm = 0.0001;
  public static final double k_driveIZone = 50;

  private static final double k_steeringP = 0.75;
  private static final double k_frontLeftSteeringP = k_steeringP/ 360;
  private static final double k_backLeftSteeringP = k_steeringP/ 360;
  private static final double k_backRightSteeringP = k_steeringP / 360;
  private static final double k_frontRightSteeringP = k_steeringP / 360;

  private static final double k_steeringD = 0.01;
  private static final double k_frontLeftSteeringD = k_steeringD;
  private static final double k_backLeftSteeringD = k_steeringD;
  private static final double k_backRightSteeringD = k_steeringD;
  private static final double k_frontRightSteeringD = k_steeringD;

  public static final double k_maxDriveSpeedMetersPerSecond = k_maxDriveSpeed / k_ticksPerMeter;
  public static final double k_maxAngularSpeed = 2.0; // radians / sec

  private final Gyro m_gyro = new Gyro();

  SwerveModule m_frontLeft = new SwerveModule(Swerve.FLDrivePWM, Swerve.FLDriveDir, Swerve.FLDriveEncInt,
      Swerve.FLDriveEncDir, Swerve.FLSteeringPWM, Swerve.FLSteeringDir, Swerve.FLSteeringEncA, Swerve.FLSteeringEncB,
      Swerve.FLI2CAddr,
      "FrontLeft");
  SwerveModule m_backLeft = new SwerveModule(Swerve.BLDrivePWM, Swerve.BLDriveDir, Swerve.BLDriveEncInt,
      Swerve.BLDriveEncDir, Swerve.BLSteeringPWM, Swerve.BLSteeringDir, Swerve.BLSteeringEncA, Swerve.BLSteeringEncB,
      Swerve.BLI2CAddr,
      "BackLeft");
  SwerveModule m_backRight = new SwerveModule(Swerve.BRDrivePWM, Swerve.BRDriveDir, Swerve.BRDriveEncInt,
      Swerve.BRDriveEncDir, Swerve.BRSteeringPWM, Swerve.BRSteeringDir, Swerve.BRSteeringEncA, Swerve.BRSteeringEncB,
      Swerve.BRI2CAddr,
      "BackRight");
  SwerveModule m_frontRight = new SwerveModule(Swerve.FRDrivePWM, Swerve.FRDriveDir, Swerve.FRDriveEncInt,
      Swerve.FRDriveEncDir, Swerve.FRSteeringPWM, Swerve.FRSteeringDir, Swerve.FRSteeringEncA, Swerve.FRSteeringEncB,
      Swerve.FRI2CAddr,
      "FrontRight");

  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
      m_frontLeftLocation, m_backLeftLocation, m_backRightLocation, m_frontRightLocation);

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
                    xSpeed, ySpeed, rot, m_gyro.getRotation2d())
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

  }
}
