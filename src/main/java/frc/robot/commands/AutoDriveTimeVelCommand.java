// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.lib.DriveMode;
import frc.robot.lib.g;
import frc.robot.subsystems.Drivetrain;

public class AutoDriveTimeVelCommand extends Command {
  Drivetrain m_drive;
  Timer m_timer = new Timer();
  double m_rampTime = 1.0; // Seconds
  double m_timeOut_sec;
  double m_driveAngle;
  double m_robotAngle;
  double m_speed;
  double m_rampUpTime_sec;
  double m_rampDownTime_sec;
  double m_currentSpeed = 0;
  boolean m_goToNote;
  boolean m_goToApril;
  boolean m_enableStartSteering;
  double m_driveAngleAdjusted = m_driveAngle;
  /** Creates a new AutoDriveTimeVelCommand. */
  public AutoDriveTimeVelCommand(double _speed_mps, double _driveAngle, double _robotAngle, double _timeOut_sec, double _rampUpTime_sec, double _rampDownTime_sec,  boolean _enableStartSteering) {
    m_drive = RobotContainer.m_drivetrain;
    m_timeOut_sec = _timeOut_sec;
    m_driveAngle = _driveAngle;
    m_robotAngle = _robotAngle;
    m_speed = _speed_mps;
    m_rampUpTime_sec = _rampUpTime_sec;
    m_rampDownTime_sec = _rampDownTime_sec;
    m_currentSpeed = m_speed;
    m_enableStartSteering = _enableStartSteering;
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.start();
    m_driveAngleAdjusted = m_driveAngle;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_driveAngleAdjusted = m_driveAngle;

    double currentTime_sec = m_timer.get();
    if (currentTime_sec < m_timeOut_sec && currentTime_sec > m_timeOut_sec - m_rampDownTime_sec) { // In the ramp down time
      m_currentSpeed = m_speed * (m_timeOut_sec - currentTime_sec) / m_rampDownTime_sec;
    } else if (currentTime_sec < m_rampUpTime_sec) {// In the ramp up time
      m_currentSpeed = m_speed * currentTime_sec / m_rampUpTime_sec;
    } else { // past the ramp up time and not in ramp down time
      m_currentSpeed = m_speed;
    }
    g.ROBOT.AngleTarget_deg = m_robotAngle;
    m_drive.drivePolarFieldCentric(m_driveAngleAdjusted, m_currentSpeed,true,true);
        
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_timer.hasElapsed(m_timeOut_sec)) {
      g.DRIVETRAIN.driveMode = DriveMode.FAST_STOP;
      return true;
    }
    return false;
  }
}
