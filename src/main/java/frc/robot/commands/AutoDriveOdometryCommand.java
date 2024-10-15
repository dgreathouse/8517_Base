// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.lib.DriveMode;
import frc.robot.lib.g;
import frc.robot.subsystems.Drivetrain;
/**
 * Rotate Robo
 */
public class AutoDriveOdometryCommand extends Command {
  Drivetrain m_drive;
  Pose2d m_poseDesired;
  PIDController m_drivePID;
  double m_currentSpeed = 0;
  double m_driveSpeed;
  boolean m_isFinished = false;
  boolean m_isRobotRotateFinished = false;
  boolean m_isWheelRotateFinished = false;
  Timer m_timer = new Timer();
  double m_rampUpTime = 0.5;
  boolean m_isFastStop = false;

  /** Rotate robot to g.ROBOT.AngleTarget_deg, Align steer wheels for 0.5 seconds, 
   *  ramp up drive speed and drive to Pose. Finish by setting DriveMode to FastStop 

   * @param _drive The drive subsystem for requirements
   * @param _pose  the desired Pose2d to drive to
   * @param _speed The speed in MPS to drive at.
   */
  public AutoDriveOdometryCommand(Pose2d _pose, double _speed) {
    m_drive = RobotContainer.m_drivetrain;
    m_poseDesired = _pose;
    m_driveSpeed = _speed;
    addRequirements(m_drive);// here to declare subsystem dependencies.
  }
  public AutoDriveOdometryCommand(Pose2d _pose, double _speed, boolean _isFastStop) {
    this(_pose,_speed);
    m_isFastStop = _isFastStop;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // The drive PID based on distance to target and output of MPS.
    m_drivePID = new PIDController(3.0, 0.750, 0.0);
    m_drivePID.setTolerance(0.05, 0.2);// 0.05M or 2 inches
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    g.ROBOT.AngleTarget_deg = m_poseDesired.getRotation().getDegrees();;
    if(m_drive.isRotateAtTarget() && !m_isRobotRotateFinished){ 
      m_isRobotRotateFinished = true;  
      m_timer.start();
    }
    Pose2d trajectory = m_poseDesired.relativeTo(g.ROBOT.Pose); // Get a Trajectory to the desired Pose relative to the current pose.
    double targetDriveAngle = trajectory.getTranslation().getAngle().getDegrees(); // The drive angle to the new pose.
    if(m_isRobotRotateFinished){ // Rotate is done
      if(!m_isWheelRotateFinished){  // Rotating wheels
        m_drive.drivePolarFieldCentric(targetDriveAngle, 1, true, false);
        if(m_timer.hasElapsed(0.5)){
          m_isWheelRotateFinished = true;
          m_timer.restart();
        }
      }else { // Driving
        double targetDistance = m_poseDesired.getTranslation().getDistance(g.ROBOT.Pose.getTranslation()); 
        double speed = m_drivePID.calculate(0, targetDistance); 
        speed = rampUpValue(speed, m_rampUpTime); // Ramp up the speed so a sudden step in voltage does not happen
        speed = MathUtil.clamp(speed, -m_driveSpeed, m_driveSpeed); // Clamp the speed to the maximum requested
        m_drive.drivePolarFieldCentric(targetDriveAngle, speed, true, true);
      }
    }else {  // Rotating Robot
      m_drive.drivePolarFieldCentric(0, 0, true, true);
    }


  }

  private double rampUpValue(double _val, double rampTime_sec) {
    double currentTime_sec = m_timer.get();
    if (currentTime_sec < rampTime_sec) {
      _val = _val * currentTime_sec / rampTime_sec;
    }
    return _val;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_drivePID.atSetpoint()) {
      if(m_isFastStop){
         g.DRIVETRAIN.driveMode = DriveMode.FAST_STOP;
      }{
        // TODO: Don't want to stop, The next command should keep moving and nothing is needed here.
      }
      m_isFinished = true;
    }
    return m_isFinished;
  }
}
