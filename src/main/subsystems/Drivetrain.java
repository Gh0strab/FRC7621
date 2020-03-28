package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

public class Drivetrain extends SubsystemBase{

WPI_VictorSPX LeftMaster = new WPI_VictorSPX(2);
WPI_VictorSPX RightMaster = new WPI_VictorSPX(0);
WPI_VictorSPX LeftSlave = new WPI_VictorSPX(3);
WPI_VictorSPX RightSlave = new WPI_VictorSPX(1);

Encoder LeftEncoder = new Encoder(1,2);
Encoder RightEncoder = new Encoder(3,4);

AHRS gyro =new AHRS(SPI.Port.kMXP);

Pose2d pose;

PIDController leftPIDController = new PIDController(9.95, 0, 0);
PIDController rightPIDController = new PIDController(9.95, 0, 0);

DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(24));
DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(getHeading(), pose);

SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.268, 1.89, 0.243);//all values need to be adjusted with Robot Characterization tool

public Drivetrain(){
    LeftSlave.follow(LeftMaster);
    RightSlave.follow(RightMaster);

    RightMaster.setInverted(true);
    LeftMaster.setInverted(true);

    gyro.reset();
}

public Rotation2d getHeading(){
    return Rotation2d.fromDegrees(-gyro.getAngle());
}
public SimpleMotorFeedforward getFeedforward(){
    return feedforward;
}
public DifferentialDriveKinematics getKinematics(){
    return kinematics;
}
public PIDController getLeftPIDController(){
    return leftPIDController;
}
public PIDController getRightPIDController(){
    return rightPIDController;
}
public Pose2d getPose(){
    return pose;
}
public DifferentialDriveWheelSpeeds getSpeeds(){
    return new DifferentialDriveWheelSpeeds(
        LeftEncoder.getRate() / 10.75 * 2 * Math.PI * Units.inchesToMeters(3.0) / 60,
        RightEncoder.getRate() / 10.75* 2 * Math.PI * Units.inchesToMeters(3.0) / 60
    );
}
public void setOutputVolts(double leftVolts, double rightVolts){
    LeftMaster.set(leftVolts / 12);
    RightMaster.set(rightVolts / 12);
}
public void reset() {
    odometry.resetPosition(new Pose2d(), getHeading());
}


@Override
public void periodic() {
  pose = odometry.update(getHeading(), LeftEncoder.getRate() / 10.75 * 2 * Math.PI * Units.inchesToMeters(3.0) / 60, RightEncoder.getRate() / 10.75* 2 * Math.PI * Units.inchesToMeters(3.0) / 60 );
}
//we divid by 10.75 because encoders return RPM values and not distance, 10.75 is the gear ratio for the KoP Drivetrain. then we multiply by 2PIr where r = our wheel radius. This gives us meters per minute. we then divide by 60 to get meters per second

}