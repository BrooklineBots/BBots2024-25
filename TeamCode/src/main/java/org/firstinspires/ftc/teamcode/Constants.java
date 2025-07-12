package org.firstinspires.ftc.teamcode;

public class Constants {
  public static class DriveConstants {
    public static final String FRONT_LEFT_MOTOR_ID = "frontLeftMotor";
    public static final String FRONT_RIGHT_MOTOR_ID = "frontRightMotor";
    public static final String BACK_LEFT_MOTOR_ID = "backLeftMotor";
    public static final String BACK_RIGHT_MOTOR_ID = "backRightMotor";
  }

  public static class ArmConstants {
    // Vertical Arm Constants
    public static final String LEFT_ARM_ID = "leftArm";
    public static final String RIGHT_ARM_ID = "rightArm";
    public static final int VERTICAL_MIN_POSITION = 0; // TODO: change
    public static final int VERTICAL_MAX_POSITION = 4100;
    public static final double VERTICAL_MOVE_POWER = 1;
    public static final int MAX_ALLOWED_DIFFERENCE = 10;

    public static final double ARM_BELT_LENGTH = 54; // in
  }

  public enum ArmPosition {
    STOWED(15),
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
    CLOSE_POSITION(0.28),
    OPEN_POSITION(0.0);

    public final double position;

    OuttakePosition(final double position) {
      this.position = position;
    }
  }

  public static class ClawArmConstants {
    public static final String CLAW_ARM_SERVO_ID = "clawArmServo";
    public static final double CLAW_ARM_DELAY_BUCKET = 0.2;
    public static final double CLAW_ARM_DELAY_TRANSFER = 0.3;
  }

  public enum ClawArmPosition {
    TRANSFER_POSITION(0.34),
    GO_TO_HIGH_BAR_POSITION(0.6),
    SCORE_HIGH_BUCKET_POSITION(0.6),
    SCORE_HIGH_BAR_POSITION(0.67),
    PICKUP_POSITION(0.67);

    public final double position;

    ClawArmPosition(final double position) {
      this.position = position;
    }
  }

  public static class ClawIntakeConstants {
    public static final String INTAKE_FLIP_SERVO_ID = "intakeFlipServo";
    public static final String CLAW_INTAKE_SERVO_ID = "clawIntakeServo";
    public static final String CLAW_ROTATION_SERVO_ID = "clawRotationServo";
  }

  public enum ClawIntakePosition {
    FLIP_TRANSFER_POSITION(0.4),
    FLIP_PICKUP_POSITION(1.0),
    CLAW_OPEN_POSITION(0.0),
    CLAW_CLOSE_POSITION(0.0);

    public final double position;

    ClawIntakePosition(final double position) {
      this.position = position;
    }
  }

  public static class IntakeConstants {
    public static final String LEFT_WHEEL_ID = "leftWheel";
    public static final String RIGHT_WHEEL_ID = "rightWheel";
    public static final String LEFT_FLIP_SERVO_ID = "leftFlipServo";
    public static final String RIGHT_FLIP_SERVO_ID = "rightFlipServo";
    public static final double LEFT_UP_POSITION = 0.82; // TODO: change
    public static final double LEFT_DOWN_POSITION = 0.37; // TODO: change
    public static final double RIGHT_UP_POSITION = 0.95;
    public static final double RIGHT_DOWN_POSITION = 0.45;
    public static final double MAX_VOLTAGE = 5; // TODO: change
    public static final double SERVO_POWER = 0.5;
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
    public static final double EXTENSION_POWER = 0.7;
    public static final double RIGHT_EXTEND_POWER = 0.6;
    public static final double LEFT_EXTEND_POWER = 0.75;
    public static final double RIGHT_RETRACT_POWER = -0.6;
    public static final double LEFT_RETRACT_POWER = -0.75;
    public static final double RETRACT_DELAY_SECONDS = 0.1;
    public static final double EXTEND_DELAY_SECONDS = 0.005;
  }
}
