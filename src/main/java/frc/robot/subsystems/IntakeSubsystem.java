// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.g;

public class IntakeSubsystem extends SubsystemBase {
  TalonFX m_leftMotor;
  VoltageOut m_leftVoltageOut = new VoltageOut(0).withEnableFOC(true);

  /** Creates a new Intake. */
  public IntakeSubsystem() {
    m_leftMotor = new TalonFX(g.CAN_IDS_ROBORIO.INTAKE, g.CAN_IDS_ROBORIO.NAME);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void spin(double _volts) {
    double volts = _volts * 5;
    m_leftMotor.setControl(m_leftVoltageOut.withOutput(volts));
  }
}
