/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/**
 * Add your docs here.
 */
public class DriveTrain extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private CANSparkMax leftFront;
  private CANSparkMax rightFront;
  private CANSparkMax leftBack;
  private CANSparkMax rightBack;

  private SpeedControllerGroup leftMotors;
  private SpeedControllerGroup rightMotors;

  private DifferentialDrive differentialDrive;

  public DriveTrain() {
    leftFront = new CANSparkMax(1, MotorType.kBrushless);
    rightFront = new CANSparkMax(3, MotorType.kBrushless);
    leftBack = new CANSparkMax(2, MotorType.kBrushless);
    rightBack = new CANSparkMax(4, MotorType.kBrushless);

    leftMotors = new SpeedControllerGroup(leftFront, leftBack);
    rightMotors = new SpeedControllerGroup(rightFront, rightBack);

    differentialDrive = new DifferentialDrive(leftMotors, rightMotors);
  }

  public void arcadeDrive(double drive, double steer) {
    differentialDrive.arcadeDrive(drive, steer);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new TeleopDrive());
  }
}
