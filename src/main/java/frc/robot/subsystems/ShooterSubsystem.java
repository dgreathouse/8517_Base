// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.FlipperStates;
import frc.robot.lib.ShooterState;
import frc.robot.lib.g;

public class ShooterSubsystem extends SubsystemBase {
  TalonFX m_leftMotor; // Declare a TalonFX motor controller class and call it m_leftMotor;
  TalonFX m_rightMotor; // Declare a TalonFX motor controller class and call it m_rightMotor;

  Servo m_leftServo;
  Servo m_rightServo;

  VoltageOut m_spinVoltageOut = new VoltageOut(0);

  public ShooterSubsystem() {
    initialize();
  }

  private void initialize() {
    m_leftMotor = new TalonFX(g.CAN_IDS_CANIVORE.SHOOTER_LEFT, g.CAN_IDS_CANIVORE.NAME);
    m_rightMotor = new TalonFX(g.CAN_IDS_CANIVORE.SHOOTER_RIGHT, g.CAN_IDS_CANIVORE.NAME);
    m_leftMotor.setNeutralMode(NeutralModeValue.Brake);
    m_rightMotor.setNeutralMode(NeutralModeValue.Brake);
    m_rightMotor.setInverted(true);

    m_leftServo = new Servo(0);
    m_rightServo = new Servo(1);
  }

  public void spin(double _speedTop, double _speedBot) {
    // Set the left and right motor voltage.
    m_leftMotor.setControl(m_spinVoltageOut.withEnableFOC(true).withOutput(_speedTop * g.ROBOT.BATTERY_MAX_VOLTS));
    m_rightMotor.setControl(m_spinVoltageOut.withEnableFOC(true).withOutput(-_speedBot * g.ROBOT.BATTERY_MAX_VOLTS));
  }

  public void setFlippersRetracted() {
    g.SHOOTER.flipperState = FlipperStates.BACK;
  }

  public void setFlipperExtended() {
    g.SHOOTER.flipperState = FlipperStates.SHOOT;
  }

  public void setFlipperPreload() {
    g.SHOOTER.flipperState = FlipperStates.PRELOAD;
  }

  private void retractFlippers() {
    m_leftServo.set(.6);
    m_rightServo.set(.3);
  }

  private void extendFlippers() {
    m_leftServo.set(0.2);
    m_rightServo.set(0.69);
  }

  private void preloadFlippers() {
    m_leftServo.set(0.45);
    m_rightServo.set(0.44);
  }

  public void setShooterFeedShort() {
    g.SHOOTER.shooterState = ShooterState.FEEDSHORT;
  }

  public void setShooterFeedLong() {
    g.SHOOTER.shooterState = ShooterState.FEEDLONG;
  }

  public void setShooterOff() {
    g.SHOOTER.shooterState = ShooterState.OFF;
  }

  @Override
  public void periodic() {
    switch (g.SHOOTER.shooterState) {
      case FEEDLONG:
        spin(1.0, 1.0);
        break;
      case FEEDSHORT:
        spin(0.5, 0.5);
        break;
      case OFF:
        spin(0.0, 0.0);
        break;
      default:
        spin(0.0, 0.0);
        break;

    }
    switch (g.SHOOTER.flipperState) {
      case BACK:
        retractFlippers();
        break;
      case PRELOAD:
        preloadFlippers();
        break;
      case SHOOT:
        extendFlippers();
        break;
      default:
        retractFlippers();
        break;

    }
  }
}
