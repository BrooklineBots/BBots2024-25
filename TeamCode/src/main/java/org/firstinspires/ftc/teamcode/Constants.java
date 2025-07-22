package org.firstinspires.ftc.teamcode;

public class Constants {
  public static class DriveConstants {
    public static final String FRONT_LEFT_MOTOR_ID = "frontLeftMotor";
    public static final String FRONT_RIGHT_MOTOR_ID = "frontRightMotor";
    public static final String BACK_LEFT_MOTOR_ID = "backLeftMotor";
    public static final String BACK_RIGHT_MOTOR_ID = "backRightMotor";
    public static final String IMU_ID = "imu";
  }

  public static class ArmConstants {
    // Vertical Arm Constants
    public static final String LEFT_ARM_ID = "leftArm";
    public static final String RIGHT_ARM_ID = "rightArm";
    public static final int VERTICAL_MIN_POSITION = 0; // TODO: change
    public static final int VERTICAL_MAX_POSITION = 3700;
    public static final double VERTICAL_MOVE_POWER = 0.4;
    public static final int MAX_ALLOWED_DIFFERENCE = 50;

    public static final double ARM_BELT_LENGTH = 54; // in
  }

  public enum ArmPosition {
    STOWED(20),
    GO_TO_HIGH_BAR(1100),
    SCORE_HIGH_BUCKET(3200),
    // GO_TO_HIGH_BAR(2200),
    SCORE_HIGH_BAR(200); // TODO: check value (could be not-updated)

    public final int encoderTicks;

    ArmPosition(final int encoderTicks) {
      this.encoderTicks = encoderTicks;
    }
  }

  public static class OuttakeConstants {
    public static final String CLAW_SERVO_ID = "outtakeClaw";
  }

  public enum OuttakePosition {
    CLOSE_POSITION(0.26),
    OPEN_POSITION(0.0);

    public final double position;

    OuttakePosition(final double position) {
      this.position = position;
    }
  }

  public static class OuttakeArmConstants {
    public static final String CLAW_ARM_SERVO_ID = "clawArmServo";
    public static final double OUTTAKE_DELAY_TRANSFER = 0.5;
  }

  public enum OuttakeArmPosition {
    TRANSFER_POSITION(0.345),
    AUTONOMOUS_POSITION(0.4),
    GO_TO_HIGH_BAR_POSITION(0.665),
    SCORE_HIGH_BUCKET_POSITION(0.64),
    SCORE_HIGH_BAR_POSITION(0.68), //TODO: don't use this
    PICKUP_POSITION(0.68);

    public final double position;

    OuttakeArmPosition(final double position) {
      this.position = position;
    }
  }

  public static class IntakeConstants {
    public static final String INTAKE_FLIP_SERVO_ID = "intakeFlipServo";
    public static final String CLAW_INTAKE_SERVO_ID = "clawIntakeServo";
    public static final String CLAW_ROTATION_SERVO_ID = "clawRotationServo";
  }

  public enum IntakePosition {
    FLIP_TRANSFER_POSITION(0.4),
    FLIP_PICKUP_POSITION(0.968),
    CLAW_OPEN_POSITION(0.0),
    CLAW_CLOSE_POSITION(0.5);

    public final double position;

    IntakePosition(final double position) {
      this.position = position;
    }
  }


  public static class LimelightConstants {
    /** Pipelines configured on the Limelight UI. */
    public enum Pipeline {
      /** Detect red alliance elements. UI pipeline 0. */
      RED(0),
      /** Detect blue alliance elements. UI pipeline 1. */
      BLUE(1),
      /** Detect universal yellow elements (centre game piece). UI pipeline 2. */
      YELLOW(2);

      public final int index;

      Pipeline(final int index) {
        this.index = index;
      }
    }

    public static final double kP =
        0.015; // proportional gain from tx degrees → motor power TODO: Tune me!
    public static final double kMinCmd =
        0.07; // minimum power so the robot actually moves TODO: Tune me!
  }

  public static class HorizontalConstants {
    public static final String RIGHT_EXTENSION_ID = "right_extension_servo";
    public static final String LEFT_EXTENSION_ID = "left_extension_servo";
    public static final double EXTENSION_POWER = 0.5;
  }

  public static class HomingSensorConstants{
    public static final String SENSOR_ID = "homingSensor";
  }
}
