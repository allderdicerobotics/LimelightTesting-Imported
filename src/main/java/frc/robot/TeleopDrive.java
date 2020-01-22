/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.networktables.*;

import java.lang.Math;
import edu.wpi.first.wpilibj.controller.PIDController;

public class TeleopDrive extends Command {

  private boolean m_LimelightHasValidTarget = false; //Does the limelight have a valid target
  private double m_LimelightDriveCommand = 0; //How hard to drive
  private double m_LimelightSteerCommand = 0; //How hard to steer

  private PIDController m_llsteer_pid = new PIDController(0.03, 0.000003, 0.003); //Create a PID controller for steering

  public TeleopDrive() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.driveTrain);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Update_Limelight_Tracking(); //Update data from the limelight

    //Get joystick data
    double steer = Robot.drive.getRawAxis(4);
    double drive = -Robot.drive.getRawAxis(1);
    boolean auto = Robot.drive.getRawButton(1);

    steer *= 0.4; //Make sure steering isnt to hard
    drive *= 0.5; //Slow down teleop driving

    if (auto)
    {
      if (m_LimelightHasValidTarget)
      {
        Robot.driveTrain.arcadeDrive(m_LimelightDriveCommand,m_LimelightSteerCommand); //If has target in semiauto, drive with limelight PID
      }
      else
      {
        Robot.driveTrain.arcadeDrive(0.0, 0.0); //If no target in semiauto, dont do anything
      }
    }
    else
    {
      Robot.driveTrain.arcadeDrive(drive,steer); //If not in semiauto
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }

  public void Update_Limelight_Tracking()
  {
    // Get data from the LL2 over networktables

    double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    double ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
    
    if (tv < 1.0) //If no valid target
    {
      m_LimelightHasValidTarget = false; //We have no valid targets
      m_LimelightDriveCommand = 0.0; //Dont move
      m_LimelightSteerCommand = 0.0; //Dont steer
      return; //End the function
    }

    m_LimelightHasValidTarget = true; //Otherwise, LL2 has valid target

    // Start with pid steering
    
    double steer_cmd = m_llsteer_pid.calculate(0-tx); //PID, but negative so steering goes the right way

    //convert ta (area) and ty (y angle)

    //to tune, if nessecary,
    //1. align limelight so it is facing a target directly straight
    //2. get ta from limelight webpage (10.1.17.1:5801) under image
    //3. measure distance with a tape measure
    //4. update variables
    //     a1 <-> ta from ll webpage (originally 1.203)
    //     d1 <-> actual distance measured (originally 64)

    double a1 = 1.203;
    double d1 = 64;

    //convert area and y degree to distance in inches

    //Inverse square law:
    //    a1 = area from testing
    //    d1 = distance from testing
    //    a2 = current measured area
    //    d2 = distraw = actual distance from target
    double distraw = Math.sqrt((a1*Math.pow(d1,2))/ta); 

    //Right triangle: 
    //                              ^  <- TARGET
    //             distraw      /    |
    //                     /         |
    //                 /            _| not used
    //             /  ) <-ty       | |
    // ROBOT -> /--------------------|
    //                  tdist
    //EQN v
    //cos(ty) = tdist / distraw
    double tdist = Math.cos(ty)*distraw;

    //result is in inches, output over smart dashboard
    SmartDashboard.putNumber("Target Distance (in): ", tdist);

    m_LimelightSteerCommand = steer_cmd; //update steering values
  }
}
