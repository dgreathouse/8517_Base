// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.lib.SwerveModuleConstants;


/** Add your docs here. */
public class SwerveModule {
    SwerveModuleConstants k;
    SwerveModulePosition m_position;
    public SwerveModule(SwerveModuleConstants _k){
        k = _k;
        m_position = new SwerveModulePosition();
    }
}
