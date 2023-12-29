package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {

   CANSparkMax rightFrontSpark;
   CANSparkMax rightBackSpark;
   CANSparkMax leftFrontSpark;
   CANSparkMax leftBackSpark;

   RelativeEncoder rightFrontEncoder;
   RelativeEncoder rightBackEncoder;
   RelativeEncoder leftFrontEncoder;
   RelativeEncoder leftBackEncoder;

   static MotorControllerGroup rightMotors;
   static MotorControllerGroup leftMotors;
   DifferentialDrive dDrive;

   static DifferentialDrive differentialDrive = new DifferentialDrive(rightMotors, leftMotors);

   AHRS gyro = new AHRS(SPI.Port.kMXP);

   public Drivetrain() {
    rightFrontSpark = new CANSparkMax(Constants.RIGHT_FRONT_SPARK, MotorType.kBrushless);
    rightBackSpark = new CANSparkMax(Constants.RIGHT_BACK_SPARK, MotorType.kBrushless);
    leftFrontSpark = new CANSparkMax(Constants.LEFT_FRONT_SPARK, MotorType.kBrushless);
    leftBackSpark = new CANSparkMax(Constants.LEFT_BACK_SPARK, MotorType.kBrushless);

    rightFrontEncoder = rightFrontSpark.getEncoder();
    rightBackEncoder = rightBackSpark.getEncoder();
    leftFrontEncoder = leftFrontSpark.getEncoder();
    leftBackEncoder = leftBackSpark.getEncoder();
    // gets encoder values

    rightFrontSpark.restoreFactoryDefaults();
    rightBackSpark.restoreFactoryDefaults();
    leftFrontSpark.restoreFactoryDefaults();
    leftBackSpark.restoreFactoryDefaults();
    // resets all of the random stuff

    rightFrontSpark.enableVoltageCompensation(12);
    rightBackSpark.enableVoltageCompensation(12);
    leftFrontSpark.enableVoltageCompensation(12);
    leftBackSpark.enableVoltageCompensation(12);
    // sets voltage for motors

    rightMotors = new MotorControllerGroup(rightFrontSpark, rightBackSpark);
    leftMotors = new MotorControllerGroup(leftFrontSpark, leftBackSpark);
    dDrive = new DifferentialDrive(leftMotors, rightMotors);

   }

   public static void drive(double speed, double rotation) {
    differentialDrive.arcadeDrive(speed, rotation);
    // takes speed and rotation thing and makes it drive
   }

   public void zeroEncoders() {
    rightFrontEncoder.setPosition(0);
    rightBackEncoder.setPosition(0);
    leftFrontEncoder.setPosition(0);
    leftBackEncoder.setPosition(0);
   }

   public void stopMotors() {
    rightFrontSpark.set(0);
    rightBackSpark.set(0);
    leftFrontSpark.set(0);
    leftBackSpark.set(0);
   }

   public double getRightFrontPosition() {
    return rightFrontEncoder.getPosition();
   }

   public double geRightBackPosition() {
    return rightBackEncoder.getPosition();
   }

   public double getLeftFrontPosition() {
    return leftFrontEncoder.getPosition();
   }

   public double getLeftBackPosition() {
    return leftBackEncoder.getPosition();
   }

}