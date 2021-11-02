// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.SPI;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public CANSparkMax leftLead, leftFollowA, leftFollowB, rightLead, rightFollowA, rightFollowB;
  public Joystick driver;
  public Joystick operator;
  public static AHRS navx;
  public double originalAngle;
  public boolean PreviousPOVActive;
  public double target_value = 0;
  public CANEncoder leftDriveEncoder, rightDriveEncoder;
  public double DriveEncoderCONV = 21.0 / 101.4;
  public boolean PreviousPressed = false;
  public double TargetTicks = 0;
  public double speed = 0;
  public double turn = 0;

  public enum states {
    turn1, turn2, forward, back, done, initback
  };

  public states status = states.done;
  public states PrevStatus = states.done;

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    leftLead = new CANSparkMax(4, MotorType.kBrushless);
    leftFollowA = new CANSparkMax(5, MotorType.kBrushless);
    leftFollowB = new CANSparkMax(6, MotorType.kBrushless);
    rightLead = new CANSparkMax(1, MotorType.kBrushless);
    rightFollowA = new CANSparkMax(2, MotorType.kBrushless);
    rightFollowB = new CANSparkMax(3, MotorType.kBrushless);

    leftLead.restoreFactoryDefaults();
    leftFollowA.restoreFactoryDefaults();
    leftFollowB.restoreFactoryDefaults();
    rightLead.restoreFactoryDefaults();
    rightFollowA.restoreFactoryDefaults();
    rightFollowB.restoreFactoryDefaults();

    rightLead.setInverted(true);
    leftLead.setIdleMode(IdleMode.kBrake);
    rightLead.setIdleMode(IdleMode.kBrake);

    leftFollowA.follow(leftLead);
    leftFollowB.follow(leftLead);
    rightFollowA.follow(rightLead);
    rightFollowB.follow(rightLead);

    driver = new Joystick(5);
    operator = new Joystick(4);
    navx = new AHRS(SPI.Port.kMXP);
    navx.reset();

    leftDriveEncoder = leftLead.getEncoder();
    rightDriveEncoder = rightLead.getEncoder();

    leftDriveEncoder.setVelocityConversionFactor(-1);
    rightDriveEncoder.setVelocityConversionFactor(-1);

  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable chooser
   * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
   * remove all of the chooser code and uncomment the getString line to get the
   * auto name from the text box below the Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional comparisons to the
   * switch structure below with additional strings. If using the SendableChooser
   * make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    originalAngle = navx.getAngle();
    PreviousPOVActive = false;
    leftDriveEncoder.setPosition(0);
    rightDriveEncoder.setPosition(0);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    double heading = BaseAngle(navx.getAngle() - originalAngle);
    double error = 0.0;
    boolean povActive = false;
    boolean Pressed = false;
    povActive = driver.getPOV() != -1;
    Pressed = driver.getRawButton(4);
    double pov = BaseAngle(driver.getPOV());
    speed = 0.0;
    turn = 0.0;

    if (operator.getRawButton(1)) {
      leftDriveEncoder.setPosition(0);
      rightDriveEncoder.setPosition(0);
      status = states.forward;
    }
    if (Pressed && status == states.done) {
      status = states.forward;
    }
    switch (status) {
      case forward:
        if (driveForward(PrevStatus != states.forward, 3.0)) {
          status = states.turn1;
          Pressed = false;
        }

        break;
      case turn1:
        if (turnToAngle(180)) {
          status = states.initback;
          Pressed = false;
        }
        break;
      case initback:
        driveForward(true, 3.0);
        status = states.back;
        break;
      case back:
        if (driveForward(false, 3.0)) {
          status = states.turn2;
          Pressed = false;
        }
        break;
      case turn2:
        if (turnToAngle(0)) {
          status = states.done;
          Pressed = false;
        }
        break;
    }

    if (povActive) {

      if (!PreviousPOVActive) {
        target_value = BaseAngle(heading + pov);
      }
      error = ShortAngle(heading, target_value);
      if (Math.abs(error) > 2) {
        turn = error / 25.0;
      } else {
        turn = 0;
      }
      if (turn > .4) {
        turn = .4;
      }
    } else {
      // speed = driver.getRawAxis(1);
      // turn = driver.getRawAxis(4);
    }

    speed = DeadZone(speed, .2);
    turn = DeadZone(turn, .1);

    leftLead.set(speed - turn);
    rightLead.set(speed + turn);

    // speed *= .4;
    // turn *= .3;

    SmartDashboard.putNumber("Heading", heading);
    SmartDashboard.putNumber("Speed", speed);
    SmartDashboard.putNumber("Turn", turn);
    SmartDashboard.putBoolean("POVActive", povActive);
    SmartDashboard.putNumber("POV", pov);
    SmartDashboard.putNumber("TargetValue", target_value);

    SmartDashboard.putNumber("Left Encoder", -leftDriveEncoder.getPosition());
    SmartDashboard.putNumber("Right Encoder", -rightDriveEncoder.getPosition());
    SmartDashboard.putNumber("Left Distance", -leftDriveEncoder.getPosition() * DriveEncoderCONV);
    SmartDashboard.putNumber("Right Distance", -rightDriveEncoder.getPosition() * DriveEncoderCONV);
    SmartDashboard.putString("Status", status.toString());
    PreviousPOVActive = povActive;
    PreviousPressed = Pressed;
    PrevStatus = status;
  }

  public double GetDistance() {

    double average = -(leftDriveEncoder.getPosition() + rightDriveEncoder.getPosition()) * DriveEncoderCONV / 2.0;

    return average;
  }

  public boolean turnToAngle(double angle) {
    boolean done = false;
    double heading = BaseAngle(navx.getAngle() - originalAngle);
    double error = ShortAngle(heading, angle);
    if (Math.abs(error) > 2) {
      turn = error / 25.0;
    } else {
      turn = 0;
      done = true;
    }
    if (Math.abs(turn) > .25) {
      turn = .15 * Math.signum(turn);
    }
    return done;
  }

  public boolean driveForward(boolean firstCall, double distance) {

    boolean done = false;
    if (firstCall) {
      TargetTicks = GetDistance() + distance;
    }
    if (GetDistance() < TargetTicks) {
      speed = -0.3;
      turn = 0;
    } else {
      speed = 0;
      turn = 0;
      done = true;
    }
    return done;
  }

  public double BaseAngle(double angle) {
    angle %= 360.0;

    if (angle > 180.0) {
      angle -= 360.0;
    }
    if (angle <= -180.0) {
      angle += 360.0;
    }

    return angle;
  }

  public double ShortAngle(double from, double to) {
    double angle = BaseAngle(to - from);
    return angle;
  }

  private double DeadZone(double value, double size) {

    double sign = Math.signum(value);

    if (Math.abs(value) < size) {
      value = 0;
    } else {
      value = sign * (Math.abs(value) - size);
    }

    return value;

  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
    String message = ""; 
    if (driver.isConnected() && operator.isConnected()) {
      message = "Joysticks are Connected";
    } else {
      message = "Joysticks Not Connected:";
      if (!driver.isConnected()){
        message += " Driver ";
      }
      if (!operator.isConnected()){
      message += " Operator ";
      }
    }
    SmartDashboard.putString("Joystick Check", message);
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }
}
