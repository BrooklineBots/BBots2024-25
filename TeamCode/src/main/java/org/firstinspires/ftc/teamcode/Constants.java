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
        public static final int VERTICAL_MAX_POSITION = 4000;
        public static final double VERTICAL_MOVE_POWER = 0.25;
        public static final int MAX_ALLOWED_DIFFERENCE = 10;

        //Horizontal Arm Constants
        public static final String HORIZONTAL_ARM_ID = "horizontalArm";
        public static final int HORIZONTAL_MIN_POSITION = 0; //TODO: change
        public static final int HORIZONTAL_MAX_POSITION = 200; //TODO: change
        public static final double HORIZONTAL_MOVE_POWER = 0.25;



    }

    public enum ArmPosition{
        INTAKE(0),
        STOWED(100),
        SCORE_HIGH_BUCKET(2000),
        GO_TO_HIGH_BAR(1500),
        SCORE_HIGH_BAR(1300); //TODO: change

        public final int encoderTicks;

        ArmPosition(int encoderTicks){
            this.encoderTicks = encoderTicks;
        }
    }

    public static class ClawConstants {
        public static final String CLAW_SERVO_ID = "servo1";
    }

    public enum ClawPosition{
        CLOSE_POSITION(0.7),
        OPEN_POSITION(0.6);


        public final double position;

        ClawPosition(double position){
            this.position = position;
        }
    }

    public static class IntakeConstants{
        public static final String LEFT_WHEEL_ID = "leftWheel";
        public static final String RIGHT_WHEEL_ID = "rightWheel";
        public static final String LEFT_FLIP_SERVO_ID = "leftFlipServo";
        public static final String RIGHT_FLIP_SERVO_ID = "rightFlipServo";
        public static final double LEFT_UP_POSITION = 0.6; //TODO: change
        public static final double RIGHT_UP_POSITION = 0.6;
        public static final double LEFT_DOWN_POSITION = 0.2; //TODO: change
        public static final double RIGHT_DOWN_POSITION = 0.2;
        public static final double MAX_VOLTAGE = 5; //TODO: change
        public static final double SERVO_POWER = 0.5;

    }

}
