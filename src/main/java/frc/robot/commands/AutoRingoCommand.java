// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.Interpolate;
import frc.robot.lib.NoteData;
import frc.robot.lib.OrangePi5Vision;
import frc.robot.lib.g;
import frc.robot.subsystems.Drivetrain;

public class AutoRingoCommand extends Command {
  OrangePi5Vision m_vision = new OrangePi5Vision();
  Drivetrain m_drive;
  double[] x = {4.0,27.5};
  double[] y = {80,32};
  /** Creates a new AutoRingoCommand. */
  public AutoRingoCommand(double _inches) {
    addRequirements(g.ROBOT.Drive);
    m_drive = g.ROBOT.Drive;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    NoteData data = m_vision.getNoteData();
    SmartDashboard.putNumber("Vision/Note Yaw", data.yaw);
    SmartDashboard.putNumber("Vision/Note Distance", data.distance);
    SmartDashboard.putBoolean("Vision/Note Visible", data.isNote);

    SmartDashboard.putNumber("Vision/Note Inches", Interpolate.getY(x, y, data.distance));
    // Area 27.5=32in is the closest, Area 4=80in is the farthest
    // 

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
