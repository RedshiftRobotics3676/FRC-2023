// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import static frc.robot.Constants.*;

//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CubeIntake extends SubsystemBase {
  
  /** Creates a new CubeIntake. */
  WPI_TalonFX motor;
  TalonFXConfiguration config = new TalonFXConfiguration();

  public CubeIntake() {
    motor = new WPI_TalonFX(cubeIntakeCanID);
    motor.setNeutralMode(NeutralMode.Brake);
    
    config.slot1.kP = cubeIntakeKp;
    config.slot1.kI = cubeIntakeKi;
    config.slot1.kD = cubeIntakeKd;
    config.slot1.kF = cubeIntakeKf;
    motor.configAllSettings(config);
    motor.selectProfileSlot(0, 0);
  }

  public void in() {
    motor.set(ControlMode.PercentOutput, -1);
  }

  public void out() {
    motor.set(ControlMode.PercentOutput, 1);
  }

  public void stop() {
    motor.set(ControlMode.PercentOutput, 0);
  }

  public void shoot(double speed) {
    motor.set(ControlMode.PercentOutput, speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run    
  }
}
