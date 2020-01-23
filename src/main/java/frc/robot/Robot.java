/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.DriveTrain;
import edu.wpi.first.wpilibj.Joystick;

import edu.wpi.first.networktables.*;

import java.lang.Math;
import edu.wpi.first.wpilibj.controller.PIDController;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  public static DriveTrain driveTrain;
  public static Joystick drive;

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    driveTrain = new DriveTrain();

    drive = new Joystick(0);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {

  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        
        //GO FORWARD 22 ROTATIONS
        //LIMELIGHT ALIGN
        //SHOOT
        //GET BALLS
        //LIMELIGHT ALIGN
        //SHOOT
        break;
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }

  public void Update_Limelight_Tracking_auto()
  {
    // Get data from the LL2 over networktables

    double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    double ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
    
    if (tv < 1.0) //If no valid target
    {
      ma_LimelightHasValidTarget = false; //We have no valid targets
      ma_LimelightDriveCommand = 0.0; //Dont move
      ma_LimelightSteerCommand = 0.0; //Dont steer
      return; //End the function
    }

    ma_LimelightHasValidTarget = true; //Otherwise, LL2 has valid target

    // Start with pid steering
    
    double steer_cmd = ma_llsteer_pid.calculate(0-tx); //PID, but negative so steering goes the right way

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

    ma_LimelightSteerCommand = steer_cmd; //update steering values
  }
}
}
