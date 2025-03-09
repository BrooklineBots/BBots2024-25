//package org.firstinspires.ftc.teamcode.Subsystems;
//
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.Servo;
//
//
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.teamcode.Constants;
//
//public class Intake {
//    private Servo leftWheel;
//    private Servo rightWheel;
//    private Servo leftFlipServo;
//    private Servo rightFlipServo;
//
//
//    private Telemetry telemetry;
//
//    public Intake(HardwareMap hwMap, Telemetry telemetry){
//        this.telemetry = telemetry;
//        leftWheel = hwMap.get(Servo.class, Constants.IntakeConstants.LEFT_WHEEL_ID);
//        rightWheel = hwMap.get(Servo.class, Constants.IntakeConstants.RIGHT_WHEEL_ID);
//        leftFlipServo = hwMap.get(Servo.class, Constants.IntakeConstants.LEFT_FLIP_SERVO_ID);
//        rightFlipServo = hwMap.get(Servo.class, Constants.IntakeConstants.RIGHT_FLIP_SERVO_ID);
////        leftFlipServo.setDirection(Servo.Direction.REVERSE);
//
//        leftWheel.setDirection(Servo.Direction.FORWARD);
//        rightWheel.setDirection(Servo.Direction.REVERSE);
//    }
//
//    public void collect(){
//        wheelSetPos(Constants.IntakeConstants.SERVO_POWER);
//    }
//
//    public double[] getWheelPowers(){
//        return new double[]{leftWheel.getPosition(), rightWheel.getPosition()};
//    }
//
//    public double[] getFlipperPos(){
//        return new double[]{leftFlipServo.getPosition(), rightFlipServo.getPosition()};
//    }
//
//    public void release(){
//        wheelSetPos(-Constants.IntakeConstants.SERVO_POWER);
//    }
//
//    public void rotateUp(){
//        setFlipperPos(Constants.IntakeConstants.LEFT_UP_POSITION,
//                      Constants.IntakeConstants.RIGHT_UP_POSITION);
//    }
//
//    public void rotateDown(){
//        setFlipperPos(Constants.IntakeConstants.LEFT_DOWN_POSITION,
//                      Constants.IntakeConstants.RIGHT_DOWN_POSITION);
//    }
//
//
//    public void wheelSetPos(double position){
//        Thread thread = new Thread(() -> {
//            leftWheel.setPosition(position);
//            rightWheel.setPosition(position);
//        });
//        thread.start();
//    }
//
//    public void setIntakePowers(double leftPower, double rightPower){
//        Thread thread = new Thread(() -> {
//            leftWheel.setPosition(leftPower);
//            rightWheel.setPosition(rightPower);
//        });
//        thread.start();
//    }
//
//    public void setFlipperPos(double position1, double position2){
//        Thread thread = new Thread(() -> {
//            leftFlipServo.setPosition(position1);
//            rightFlipServo.setPosition(position2);
//        });
//        thread.start();
//
//    }
//
//    public void passSample(){ //TODO: SPIT COMMAND
//        rotateUp();
//        release();
//    }
//
//
//    public void stopWheels(){
//        Thread thread = new Thread(() -> {
//            leftWheel.setPosition(0);
//            rightWheel.setPosition(0);
//        });
//        thread.start();
//    }
//
//
//}
