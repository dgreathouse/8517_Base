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

public class AutoDriveOdometryCommand extends Command {
  Drivetrain m_drive;
  Pose2d m_poseDesired;
  PIDController m_drivePID;
  double m_currentSpeed = 0;
  double m_driveSpeed;
  boolean m_isFinished = false;
  Timer m_timer = new Timer();
  double m_rampUpTime = 0.5;

  /**
   * 
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

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // The drive PID based on distance to target and output of MPS.
    m_drivePID = new PIDController(3.0, 0.750, 0.0);
    m_drivePID.setTolerance(0.05, 0.2);// 0.05M or 2 inches

    m_timer.start();
    // TODO: rotate the wheels to new location for a second before enabling the drive, This may not be possible because the robot angle also needs to change
    // FIXME: How to get it to drive straight on start. Option, Rotate robot then align wheels and then drive. That takes to much time.  Or leave rotation until end of path Needs thought and testing
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d trajectory = m_poseDesired.relativeTo(g.ROBOT.Pose); // Get a Trajectory to the desired Pose relative to the
                                                                // current pose.

    double robotAngle = m_poseDesired.getRotation().getDegrees(); // The angle of the robot from the desired pose angle
    double targetDriveAngle = trajectory.getTranslation().getAngle().getDegrees(); // The drive angle to the new pose.
    double targetDistance = m_poseDesired.getTranslation().getDistance(g.ROBOT.Pose.getTranslation()); // The drive
                                                                                                       // distance to
                                                                                                       // the new pose.
    double speed = m_drivePID.calculate(0, targetDistance); // Speed from PID based on 0 target and a changing distance
                                                                        // as the robot moves at a target angle towards the
                                                                        // destination. Output is speed MPS targetDistance is in Meters
                                                                        // targetDistance is the new setpoint since we are moving to
                                                                        // 0 for the target distance. This seams a little reversed
                                                                        // but should work.

    speed = rampUpValue(speed, m_rampUpTime); // Ramp up the speed so a sudden step in voltage does not happen
    speed = MathUtil.clamp(speed, -m_driveSpeed, m_driveSpeed); // Clamp the speed to the maximum requested
    g.ROBOT.AngleTarget_deg = robotAngle;
    m_drive.drivePolarFieldCentric(targetDriveAngle, speed, true, true); // Drive at a angle and speed and let the
                                                                                                   // SwerveDrive move to the a robot angle.



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
      g.DRIVETRAIN.driveMode = DriveMode.FAST_STOP;
      m_isFinished = true;
    }
    return m_isFinished;
  }
}
