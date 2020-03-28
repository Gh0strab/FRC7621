/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class Robot extends TimedRobot {
RobotContainer container;
public static SendableChooser <Command> autoChooser;
//private static Command defaultAutoCommand;
  @Override
  public void robotInit() {
    
    container = new RobotContainer();
    //autoChooser = new SendableChooser<>();
    //autoChooser.addOption("MoveAndShoot", new MoveAndShoot());
    
    //SmartDashboard.putData("AutoChooser", autoChooser);
    
  }

  @Override
  public void robotPeriodic() {
  }

  @Override
  public void autonomousInit() {
   /* 
    if (autoChooser != null) {
			Command autonomousCommand = autoChooser.getSelected();
			if (autonomousCommand != null) {
          autonomousCommand.schedule();
          CommandScheduler.getInstance().run();
					} else {
			  System.out.println("Auto Null Warning #1");
        defaultAutoCommand.schedule();
        CommandScheduler.getInstance().run();
				}
				  } else {
				System.out.println("Auto Null Warning #2");
        defaultAutoCommand.schedule();
        CommandScheduler.getInstance().run();
            }
            */
    container.getAutonomousCommand().schedule();
    CommandScheduler.getInstance().run();
      
    
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

}
