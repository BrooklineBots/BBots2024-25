package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;

public class Intake {
  private final Servo leftWheel;
  private final Servo rightWheel;
  private final Servo leftFlipServo;
  private final Servo rightFlipServo;

  private final Telemetry telemetry;

  public Intake(final HardwareMap hwMap, final Telemetry telemetry) {
    this.telemetry = telemetry;
    leftWheel = hwMap.get(Servo.class, Constants.IntakeConstants.LEFT_WHEEL_ID);
    rightWheel = hwMap.get(Servo.class, Constants.IntakeConstants.RIGHT_WHEEL_ID);
    leftFlipServo = hwMap.get(Servo.class, Constants.IntakeConstants.LEFT_FLIP_SERVO_ID);
    rightFlipServo = hwMap.get(Servo.class, Constants.IntakeConstants.RIGHT_FLIP_SERVO_ID);
    //        leftFlipServo.setDirection(Servo.Direction.REVERSE);

    leftWheel.setDirection(Servo.Direction.FORWARD);
    rightWheel.setDirection(Servo.Direction.REVERSE);
  }

  public void collect() {
    wheelSetPos(Constants.IntakeConstants.SERVO_POWER);
  }

  public double[] getWheelPowers() {
    return new double[] {leftWheel.getPosition(), rightWheel.getPosition()};
  }

  public double[] getFlipperPos() {
    return new double[] {leftFlipServo.getPosition(), rightFlipServo.getPosition()};
  }

  public void release() {
    wheelSetPos(1); // Constants.IntakeConstants.SERVO_POWER
  }

  public void rotateUp() {
    setFlipperPos(
        Constants.IntakeConstants.LEFT_UP_POSITION, Constants.IntakeConstants.RIGHT_UP_POSITION);
  }

  public void rotateDown() {
    setFlipperPos(
        Constants.IntakeConstants.LEFT_DOWN_POSITION,
        Constants.IntakeConstants.RIGHT_DOWN_POSITION);
  }

  public void wheelSetPos(final double position) {
    final Thread thread =
        new Thread(
            () -> {
              leftWheel.setPosition(position);
              rightWheel.setPosition(position);
            });
    thread.start();
  }

  public void setIntakePowers(final double leftPower, final double rightPower) {
    final Thread thread =
        new Thread(
            () -> {
              leftWheel.setPosition(leftPower);
              rightWheel.setPosition(rightPower);
            });
    thread.start();
  }

  public void setFlipperPos(final double position1, final double position2) {
    final Thread thread =
        new Thread(
            () -> {
              leftFlipServo.setPosition(position1);
              rightFlipServo.setPosition(position2);
            });
    thread.start();
  }

  public void stopWheels() {
    final Thread thread =
        new Thread(
            () -> {
              leftWheel.setPosition(0);
              rightWheel.setPosition(0);
            });
    thread.start();
  }
}
