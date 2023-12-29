// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

public class RobotContainer {
  public static double driveSpeed;
  public static double turnSpeed;

  // The robot's subsystems and commands are defined here...
  private Drivetrain drivetrain = new Drivetrain();

  private static final XboxController driveControl = new Joystick(Constants.kDriverControllerPort);
  private static final XboxController turnControl = new Joystick(Constants.kDriverControllerPort_2);  

  private final PWMSparkMax m_leftMotor = new PWMSparkMax(Constants.LEFT_FRONT_SPARK);
  private final PWMSparkMax m_rightMotor = new PWMSparkMax(Constants.RIGHT_FRONT_SPARK);
  private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftMotor, m_rightMotor);


  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    //drivetrain.setDefaultCommand(new DriveCommand(drivetrain, driveJoystick));
  }

  private void configureBindings() {
    
  }
  
  public static void triggerBindings() {
    driveSpeed = driveControl.getLeftY(); // Adjust axis index if needed
    turnSpeed = turnControl.getLeftX();
    Drivetrain.drive(driveSpeed, turnSpeed);
    //see drift and make no drift 
  }
}
