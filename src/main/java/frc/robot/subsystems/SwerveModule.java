// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.lib.IUpdateDashboard;
import frc.robot.lib.SwerveModuleConstants;
import frc.robot.lib.g;

/** Add your docs here. */
public class SwerveModule implements IUpdateDashboard{
    SwerveModuleConstants m_k;
    public Translation2d m_location;
    private TalonFX m_driveMotor;
    private TalonFX m_steerMotor;
    private CANcoder m_canCoder;
    private SwerveModulePosition m_position = new SwerveModulePosition();
    private PIDController m_steerPID = new PIDController(g.SWERVE.MODULE.STEER.PID_kp, g.SWERVE.MODULE.STEER.PID_ki, 0);
    private PIDController m_drivePID = new PIDController(g.SWERVE.MODULE.DRIVE.PID_kp, g.SWERVE.MODULE.DRIVE.PID_ki, 0);
    private SimpleMotorFeedforward m_driveFF = new SimpleMotorFeedforward(g.SWERVE.MODULE.DRIVE.PID_ks, g.SWERVE.MODULE.DRIVE.PID_kv);
    private VoltageOut m_steerVoltageOut = new VoltageOut(0.0);
    private VoltageOut m_driveVoltageOut = new VoltageOut(0.0);
    private StatusSignal<Double> m_drivePosition;
    private StatusSignal<Double> m_driveVelocity;
    private StatusSignal<Double> m_steerPosition;
    private StatusSignal<Double> m_steerVelocity;
    public SwerveModule(SwerveModuleConstants _k) {
        StatusCode status;
        m_k = _k;
        m_location = new Translation2d(m_k.LOCATION_X_METER, m_k.LOCATION_Y_METER);
        m_driveMotor = new TalonFX(m_k.DRIVE_CAN_ID, g.CAN_IDS_CANIVORE.NAME);
        m_steerMotor = new TalonFX(m_k.STEER_CAN_ID, g.CAN_IDS_CANIVORE.NAME);
        m_canCoder = new CANcoder(m_k.CANCODER_ID, g.CAN_IDS_CANIVORE.NAME);

        // Configure Drive Motor
        TalonFXConfiguration driveConfigs = new TalonFXConfiguration();
        driveConfigs.MotorOutput.Inverted = m_k.DRIVE_IS_REVERSED ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
                
        driveConfigs.withOpenLoopRamps(new OpenLoopRampsConfigs().withVoltageOpenLoopRampPeriod(1));
        m_driveMotor.setNeutralMode(NeutralModeValue.Brake);


        status = m_driveMotor.getConfigurator().apply(driveConfigs);
        System.out.println(m_k.NAME + " Drive Motor TalonFX Config Status =" + status.toString());

        CurrentLimitsConfigs driveCurrentConfig = new CurrentLimitsConfigs()
                .withStatorCurrentLimit(50)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(50)
                .withSupplyCurrentLimitEnable(true);

        status = m_driveMotor.getConfigurator().apply(driveCurrentConfig);
        System.out.println(m_k.NAME + " Drive Motor Current Config Status =" + status.toString());

        m_drivePosition = m_driveMotor.getPosition();
        m_drivePosition.setUpdateFrequency(g.CAN_IDS_CANIVORE.UPDATE_FREQ_HZ);
        m_driveVelocity = m_driveMotor.getVelocity();
        m_driveVelocity.setUpdateFrequency(g.CAN_IDS_CANIVORE.UPDATE_FREQ_HZ);
        // Configure Steer Motor
        m_steerPID.enableContinuousInput(-180.0, 180.0);
        TalonFXConfiguration steerConfigs = new TalonFXConfiguration();
        steerConfigs.MotorOutput.Inverted = m_k.STEER_IS_REVERSED ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;

        steerConfigs.withOpenLoopRamps(new OpenLoopRampsConfigs().withVoltageOpenLoopRampPeriod(1));
        status = m_steerMotor.getConfigurator().apply(steerConfigs);
        System.out.println(m_k.NAME + " Steer Motor TalonFX Config Status =" + status.toString());
        m_steerMotor.setNeutralMode(NeutralModeValue.Brake);
        
        // Configure CANCoder
        CANcoderConfiguration cancoderConfigs = new CANcoderConfiguration();
        cancoderConfigs.MagnetSensor.MagnetOffset = m_k.CANCODER_OFFSET_ROT;
        
        status = m_canCoder.getConfigurator().apply(cancoderConfigs);
        System.out.println(m_k.NAME + " m_canCoder Config Status =" + status.toString());
        // Set the offset position of the steer motor based on the CANCoder
        m_steerMotor.setPosition(m_canCoder.getPosition().getValueAsDouble() * g.SWERVE.MODULE.STEER.GEAR_RATIO);

        m_steerPosition = m_steerMotor.getPosition();
        m_steerPosition.setUpdateFrequency(g.CAN_IDS_CANIVORE.UPDATE_FREQ_HZ);
        m_steerVelocity = m_steerMotor.getVelocity();
        m_steerVelocity.setUpdateFrequency(g.CAN_IDS_CANIVORE.UPDATE_FREQ_HZ);

    }

    public SwerveModulePosition updatePosition() {
        double drive_rot = m_driveMotor.getPosition().getValueAsDouble();
        double angle_rot = m_steerMotor.getPosition().getValueAsDouble();
        // anagle_rot is the Motor rotations. Apply the gear ratio to get wheel
        // rotations for steer
        angle_rot = angle_rot / g.SWERVE.MODULE.STEER.GEAR_RATIO;
        /* And push them into a SwerveModuleState object to return */
        m_position.distanceMeters = drive_rot / g.SWERVE.MODULE.DRIVE.WHEEL_MotRotPerMeter;
        /* Angle is already in terms of steer rotations */
        m_position.angle = Rotation2d.fromRotations(angle_rot);

        return m_position;
    }

    public double getSteerActualAngle() {
        return m_position.angle.getDegrees() * 360 / g.SWERVE.MODULE.STEER.GEAR_RATIO;
    }

    public void setDesiredState(SwerveModuleState _state, boolean _enableSteer, boolean _enableDrive) {
        SwerveModuleState optimized = SwerveModuleState.optimize(_state, m_position.angle);
        /* ------------------------------------- Steer --------------------------------------------------- */
        if (_enableSteer) {

            double steerVolts = m_steerPID.calculate(getSteerActualAngle(), optimized.angle.getDegrees());
            m_steerMotor.setControl(m_steerVoltageOut.withOutput(steerVolts).withEnableFOC(true));
            // m_steerMotor.setControl(null);
        } else {
            m_steerMotor.setControl(m_steerVoltageOut.withOutput(0).withEnableFOC(true));
        }
        /* ------------------------------------- Drive --------------------------------------------------- */
        if (_enableDrive) {
            double driveSetVelocity_mps = optimized.speedMetersPerSecond * g.DRIVETRAIN.driveSpeedMultiplier;
            double driveVolts = m_drivePID.calculate(m_driveMotor.getVelocity().getValueAsDouble() / g.SWERVE.MODULE.DRIVE.WHEEL_MotRotPerMeter, driveSetVelocity_mps);
            driveVolts = MathUtil.clamp(driveVolts, -6, 6);
            driveVolts = driveVolts + m_driveFF.calculate(driveSetVelocity_mps);

            m_driveMotor.setControl(m_driveVoltageOut.withOutput(driveVolts).withEnableFOC(true));
        } else {
            m_driveMotor.setControl(m_driveVoltageOut.withOutput(0).withEnableFOC(true));
        }
    }
    public double getDriveCurrent(){
        return m_driveMotor.getTorqueCurrent().getValueAsDouble();
    }
  /**
   * Called by separate thread to put stuff to the dashboard at a slower rate than the main periodic
   */
  public void updateDashboard() {
    
  }
}
