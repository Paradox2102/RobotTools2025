/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import robotCore.Device;
import robotCore.Logger;
import robotCore.Swerve;

public class DriveSubsystem extends SubsystemBase {
  SwerveModule m_frontLeft = new SwerveModule(Swerve.FLDrivePWM, Swerve.FLDriveDir, Swerve.FLDriveEncInt, Swerve.FLDriveEncDir, Swerve.FLSteeringPWM, 
  Swerve.FLSteeringDir, Swerve.FLSteeringEncA, Swerve.FLSteeringEncB, Swerve.FLI2CAddr, "FrontLeft");
  SwerveModule m_backLeft = new SwerveModule(Swerve.BLDrivePWM, Swerve.BLDriveDir, Swerve.BLDriveEncInt, Swerve.BLDriveEncDir, Swerve.BLSteeringPWM, 
  Swerve.BLSteeringDir, Swerve.BLSteeringEncA, Swerve.BLSteeringEncB, Swerve.BLI2CAddr, "BackLeft");
  SwerveModule m_frontRight = new SwerveModule(Swerve.FRDrivePWM, Swerve.FRDriveDir, Swerve.FRDriveEncInt, Swerve.FRDriveEncDir, Swerve.FRSteeringPWM,
  Swerve.FRSteeringDir, Swerve.FRSteeringEncA, Swerve.FRSteeringEncB, Swerve.FRI2CAddr, "FrontRight");
  SwerveModule m_backRight = new SwerveModule(Swerve.BRDrivePWM, Swerve.BRDriveDir, Swerve.BRDriveEncInt, Swerve.BRDriveEncDir, Swerve.BRSteeringPWM,
  Swerve.BRSteeringDir, Swerve.BRSteeringEncA, Swerve.BRSteeringEncB, Swerve.FRI2CAddr, "BackRight");

  
  /**
   * Creates a new DriveSubsystem.
   */
  public DriveSubsystem() {
    Logger.log("DriveSubsystem", 3, "DriveSubsystem()");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Logger.log("DriveSubsystem", -1, "periodic()");
  }

  public SwerveModuleGetFrontLeftModule() {
    return m_frontLeft;
  }
  
  public SwerveModuleGetFrontRightModule() {
    return m_frontRight;
  }
  
  public SwerveModuleGetBackLeftModule() {
    return m_backLeft;
  }

  public SwerveModuleGetBackRightModule() {
    return m_backRight;
  }

  }

