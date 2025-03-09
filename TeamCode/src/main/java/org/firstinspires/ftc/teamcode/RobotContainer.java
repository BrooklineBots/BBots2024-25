package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.VerticalArm;
import org.firstinspires.ftc.teamcode.Subsystems.Claw;
import org.firstinspires.ftc.teamcode.Subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.autonomous.AutonomousRecorder;

@TeleOp(name = "mainDrive")
public class RobotContainer extends OpMode {

    private VerticalArm verticalArm;
    private Claw claw;
    private MecanumDrive drive;
    private Intake intake;

    private AutonomousRecorder recorder;

    private long recordingTimer;
    private final long startTimer = System.currentTimeMillis();
    private boolean isRecording = false;

    private boolean isAPressed = false;
    private boolean isBPressed = false;
    private boolean isA1Pressed = false;

    @Override
    public void init() {

       recorder = new AutonomousRecorder(hardwareMap.appContext, telemetry);

       verticalArm = new VerticalArm(hardwareMap, telemetry);
       claw = new Claw(hardwareMap, telemetry);
       drive = new MecanumDrive(hardwareMap, telemetry);
//       intake = new Intake(hardwareMap, telemetry);

       recordingTimer = System.currentTimeMillis() - startTimer;


    }

    public boolean isWithinTolerance(double targetValue, double currentValue, double tolerance ){
        return Math.abs(targetValue-currentValue) <= tolerance;
    }

    @Override
    public void loop() {

        if (gamepad1.x) {
            if (recorder.startRecording()) {
                isRecording = true;
                recordingTimer = System.currentTimeMillis();
                gamepad1.rumble(250);
            }
        }

        if (isRecording) {
            double[] drivePowers = drive.getMotorPowers();
            double[] armPower = verticalArm.getArmPowers();
            double[] wheelPower = {0, 0}; // intake.getWheelPowers();
            double clawPosition = claw.getClawPosition();
            double flipperPosition = 0.0; //intake.getFlipperPos();
            recordingTimer = System.currentTimeMillis() - startTimer;

            recorder.recordData(
                    recordingTimer,
                    drivePowers[0],  // Front Left
                    drivePowers[1],  // Front Right
                    drivePowers[2],  // Back Left
                    drivePowers[3],  // Back Right
                    armPower[0], //Left Arm
                    armPower[1], //Right Arm
                    clawPosition, //Claw
                    0, //wheelPower[0], //Intake Left
                    0, //wheelPower[1], //Intake Right
                    0 //flipperPosition //Flipper
            );
            telemetry.update();
        } else {
//            telemetry.addData("Container, isRecording: ", isRecording);
//            telemetry.update();
        }

        if (isRecording && ( (double) recordingTimer / 1000) >= 15.0) {
            recorder.stopRecording();
            isRecording = false;
            gamepad1.rumble(250);
        }

        if(!isWithinTolerance(0, gamepad1.left_stick_y, 0.05) ||
                !isWithinTolerance(0, gamepad1.left_stick_x, 0.05) ||
                !isWithinTolerance(0, gamepad1.right_stick_x, 0.05)){
            drive.drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        } else{
            drive.stop();
        }

        if(gamepad2.dpad_up){
            verticalArm.goToPosition(Constants.ArmPosition.GO_TO_HIGH_BAR);
        } else if(gamepad2.dpad_right){
            verticalArm.goToPosition(Constants.ArmPosition.SCORE_HIGH_BUCKET);
        } else if(gamepad2.dpad_down){
            verticalArm.goToPosition(Constants.ArmPosition.STOWED);
        } else if(gamepad2.dpad_left){
            if(verticalArm.getGoalPosition() == Constants.ArmPosition.GO_TO_HIGH_BAR
                    && claw.getGoalPosition() == Constants.ClawPosition.CLOSE_POSITION){
                verticalArm.goToPosition(Constants.ArmPosition.SCORE_HIGH_BAR);
                claw.setPosition(Constants.ClawPosition.OPEN_POSITION.position);
            } else if(verticalArm.getGoalPosition() == Constants.ArmPosition.SCORE_HIGH_BUCKET
                    && claw.getGoalPosition() == Constants.ClawPosition.CLOSE_POSITION){
                claw.setPosition(Constants.ClawPosition.OPEN_POSITION.position);
            } else {
                claw.setPosition(Constants.ClawPosition.OPEN_POSITION.position);
                verticalArm.goToPosition(Constants.ArmPosition.STOWED);

            }

        }

        if(gamepad1.a){
            isA1Pressed = !isA1Pressed;
            if (isA1Pressed) {
                claw.openClaw();
            } else{
                claw.closeClaw();
            }
        }


        //DRIVERS PRACTICE STOPPING AT THE RIGHT TIME
        if(gamepad2.a){
            isAPressed = !isAPressed;
            if(isAPressed){
                intake.collect();
            } else if(!isAPressed){
                intake.stop();
            }
        }

        if(gamepad2.b){
            isBPressed = !isBPressed;
            if(isBPressed){
                intake.rotateUp();
            } else if(!isBPressed){
                intake.rotateDown();
            }
        }

        if(gamepad2.x){
            intake.passSample();
            claw.closeClaw();
        }

        //make autonomous commands

//        telemetry.addData("Left Arm Position: ", verticalArm.getCurrentPosition()[0]);
//        telemetry.addData("Right Arm Position: ", verticalArm.getCurrentPosition()[1]);
        telemetry.update();


    }


}
