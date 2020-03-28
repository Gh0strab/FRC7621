package frc.robot;

import frc.robot.subsystems.Drivetrain;


import java.util.Arrays;

import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.AutonomousCommands.DriveToColorWheelBalls;

public class RobotContainer{

    private Drivetrain drive = new Drivetrain();

    public Command getAutonomousCommand(){
        TrajectoryConfig config = new TrajectoryConfig(Units.feetToMeters(2), Units.feetToMeters(2));
        config.setKinematics(drive.getKinematics());

        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            Arrays.asList(
                new Pose2d(),//sets/assumes position is 0,0
                new Pose2d(12.843, -5.918, new Rotation2d()),
                new Pose2d(7.521, -7.482, new Rotation2d())),//use these line to set x and y cords. use this in auto commands. has a max of 3 poses
                config);
            
            
    RamseteCommand command = new RamseteCommand(
        trajectory,
        drive::getPose,
        new RamseteController(2.0, 0.7),
        drive.getFeedforward(),
        drive.getKinematics(),
        drive::getSpeeds,
        drive.getLeftPIDController(),
        drive.getRightPIDController(),
        drive::setOutputVolts,    
        drive
    );
    return command.andThen(() -> drive.setOutputVolts(0, 0));
    }
    public void reset() {
        drive.reset();
      }
}
