// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.g;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeDefaultCommand extends Command {
  /** Creates a new IntakeDefaultCommand. */
  IntakeSubsystem m_inatke;
  public IntakeDefaultCommand(IntakeSubsystem _intake) {
    m_inatke = _intake;
    addRequirements(m_inatke);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double leftTrigger = 1 + g.OI.driveController.getL2Axis();
    double rightTrigger = (1 + g.OI.driveController.getR2Axis()) * -1;

    if(leftTrigger > .2){
      m_inatke.spin(leftTrigger);
    }else if( rightTrigger < -.2){
      m_inatke.spin(rightTrigger);
    }else {
      m_inatke.spin(0.0);
    }


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
