package org.firstinspires.ftc.teamcode;

public class Constants {
    public static class DriveConstants {
        public static final String FRONT_LEFT_MOTOR_ID = "front_left_motor";
        public static final String FRONT_RIGHT_MOTOR_ID = "front_right_motor";
        public static final String BACK_LEFT_MOTOR_ID = "back_left_motor";
        public static final String BACK_RIGHT_MOTOR_ID = "back_right_motor";
    }

    public static class ArmConstants {
        //Vertical Arm Constants
        public static final String LEFT_ARM_ID = "leftArm";
        public static final String RIGHT_ARM_ID = "rightArm";
        public static final int VERTICAL_MIN_POSITION = 0; //TODO: change
        public static final int VERTICAL_MAX_POSITION = 4409;
        public static final double VERTICAL_MOVE_POWER = 0.25;
        public static final int MAX_ALLOWED_DIFFERENCE = 50;

        //Horizontal Arm Constants
        public static final String HORIZONTAL_ARM_ID = "horizontalArm";
        public static final int HORIZONTAL_MIN_POSITION = 0; //TODO: change
        public static final int HORIZONTAL_MAX_POSITION = 200; //TODO: change
        public static final double HORIZONTAL_MOVE_POWER = 0.25;



    }

    public enum ArmPosition{
        INTAKE(0),
        SCORE_LOW(500),
        SCORE_MID(1000),
        SCORE_HIGH(1500);

        public final int encoderTicks;

        ArmPosition(int encoderTicks){
            this.encoderTicks = encoderTicks;
        }
    }

    public static class ClawConstants {
        public static final String CLAW_SERVO_ID = "servo1";
        public static final double OPEN_POSITION = 0.6;
        public static final double CLOSE_POSITION = 0.7;
    }

    public static class IntakeConstants{
        public static final String LEFT_WHEEL_ID = "leftWheel";
        public static final String RIGHT_WHEEL_ID = "rightWheel";
        public static final String FLIP_SERVO_ID = "flipServo";
        public static final double UP_POSITION = 0.6;
        public static final double DOWN_POSITION = 0.2;
        public static final double MAX_VOLTAGE = 5; //TODO: change
        public static final double SERVO_POWER = 0.5;

    }

}
