package org.firstinspires.ftc.teamcode;

public class Constants {
    public static class DriveConstants {
        public static final String FRONT_LEFT_MOTOR_ID = "front_left_motor";
        public static final String FRONT_RIGHT_MOTOR_ID = "front_right_motor";
        public static final String BACK_LEFT_MOTOR_ID = "back_left_motor";
        public static final String BACK_RIGHT_MOTOR_ID = "back_right_motor";
    }

    public static class ArmConstants {
        public static final String LEFT_ARM_MOTOR_ID = "left_arm_motor";
        public static final String RIGHT_ARM_MOTOR_ID = "right_arm_motor";
        public static final int MIN_POSITION = 0;
        public static final int MAX_POSITION = 4409;
        public static final double MOVE_POWER = 0.25;
        public static final int MAX_ALLOWED_DIFFERENCE = 50;
    }

    public enum ArmPosition {
        INTAKE(0),
        SCORE_LOW(500),
        SCORE_MID(1000),
        SCORE_HIGH(1500);

        public final int encoderTicks;

        ArmPosition(int encoderTicks) {
            this.encoderTicks = encoderTicks;
        }
    }

    public static class ClawConstants {
        public static final String CLAW_SERVO_ID = "servo1";
        public static final double OPEN_POSITION = 0.27;
        public static final double CLOSE_POSITION = 0.0;
    }
}
