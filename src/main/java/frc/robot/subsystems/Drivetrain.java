// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.SwerveModuleConstants;
import frc.robot.lib.g;

public class Drivetrain extends SubsystemBase {
  Pigeon2 m_pigeon2;
  SwerveDriveKinematics m_kinematics;

  /** Creates a new Drivetrain. */
  public Drivetrain() {

    initialize();
  }

  public void initialize() {
    // Define the IMU/Gyro called Pigeon2 that is on the CANIvore Bus network
    m_pigeon2 = new Pigeon2(g.CAN_IDS_CANIVORE.PIGEON2, g.CAN_IDS_CANIVORE.NAME);
    // Define the swerve module constants for each module.

    g.SWERVE.Modules[0] = new SwerveModule(new SwerveModuleConstants(
        "FL", 
        10, false,
        20, false,
        1, 0,
        0, 0));
    g.SWERVE.Modules[1] = new SwerveModule(new SwerveModuleConstants(
        "FR", 
        11, false,
        21, false,
        2, 0,
        0, 0));
    g.SWERVE.Modules[2] = new SwerveModule(new SwerveModuleConstants(
        "BR", 
        12, false,
        22, false,
        3, 0,
        0, 0));
    g.SWERVE.Modules[3] = new SwerveModule(new SwerveModuleConstants(
        "BL", 
        13, false,
        23, false,
        4, 0,
        0, 0));

    // Create a module locations for the Odometry
    Translation2d[] locations = new Translation2d[g.SWERVE.MODULE.COUNT];
    // Setup the module arrays with needed information
    for(int i = 0; i < g.SWERVE.MODULE.COUNT; i++){
      locations[i] = new Translation2d(g.SWERVE.Modules[i].k.LOCATION_X_METER,g.SWERVE.Modules[i].k.LOCATION_Y_METER);
    }
  }

  @Override
  public void periodic() {
    g.ROBOT.AngleActual_deg = m_pigeon2.getYaw().getValueAsDouble();
    // This method will be called once per scheduler run
  }
}
