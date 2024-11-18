// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.g;
import frc.robot.subsystems.Drivetrain;

public class DrivetrainDefaultCommand extends Command {
  private Drivetrain m_drive;
  SlewRateLimiter m_stickLimiterLX = new SlewRateLimiter(3);
  SlewRateLimiter m_stickLimiterLY = new SlewRateLimiter(3);
  SlewRateLimiter m_stickLimiterRX = new SlewRateLimiter(3);
  ChassisSpeeds m_speeds = new ChassisSpeeds();

  /** Creates a new DrivetrainDefaultCommand. */
  public DrivetrainDefaultCommand(Drivetrain _drivetrain) {
    m_drive = _drivetrain;
    addRequirements(m_drive);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // get thumbstick values
    double leftYRaw = -g.OI.driveController.getLeftX();
    double leftXRaw = -g.OI.driveController.getLeftY();
    double rightXRaw = -g.OI.driveController.getRightX();

    // Limit the inputs for a deadband related to the joystick
    double leftYFiltered = MathUtil.applyDeadband(leftYRaw, 0.08, 1.0);
    double leftXFiltered = MathUtil.applyDeadband(leftXRaw, 0.08, 1.0);
    double rightXFiltered = MathUtil.applyDeadband(rightXRaw, 0.15, 1.0);

    // Limit the speed of change to reduce the acceleration
    leftXFiltered = m_stickLimiterLX.calculate(leftXFiltered);
    leftYFiltered = m_stickLimiterLY.calculate(leftYFiltered);
    rightXFiltered = m_stickLimiterRX.calculate(rightXFiltered);

    // Set the speed variables for the chassis speeds
    m_speeds.vxMetersPerSecond = leftXFiltered * g.SWERVE.MODULE.DRIVE.MAX_VELOCITY_MeterPerSec;
    m_speeds.vyMetersPerSecond = leftYFiltered * g.SWERVE.MODULE.DRIVE.MAX_VELOCITY_MeterPerSec;
    m_speeds.omegaRadiansPerSecond = rightXFiltered * g.SWERVE.MODULE.DRIVE.MAX_ANGULAR_VELOCITY_RadianPerSec;

    switch (g.DRIVETRAIN.driveMode) {
      case FIELD_CENTRIC:
        m_drive.driveFieldCentric(m_speeds);
        break;
      case ANGLE_FIELD_CENTRIC:
        m_drive.driveAngleFieldCentric(m_speeds.vxMetersPerSecond, m_speeds.vyMetersPerSecond, true, true);
        break;
      case POLAR_CENTRIC:
        // Do nothing in teleop since this is used in autonomous
        break;
      case ROBOT_CENTRIC:
        m_drive.driveRobotCentric(m_speeds);
        break;
      case ROTATE_FIELD_CENTRIC:
        m_speeds.omegaRadiansPerSecond = g.DRIVETRAIN.ROTATE_FIELDCENTRIC_SPEED_RadPSec;
        m_drive.driveFieldCentric(m_speeds);
        break;
      case FAST_STOP:
        m_drive.fastStop();
        break;
      default:
        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
