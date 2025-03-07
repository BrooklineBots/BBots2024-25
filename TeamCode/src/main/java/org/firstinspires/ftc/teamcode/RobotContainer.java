package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

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

    private ElapsedTime recordingTimer;
    private boolean isRecording = false;

    private boolean isAPressed = false;
    private boolean isBPressed = false;

    @Override
    public void init() {

       recorder = new AutonomousRecorder(hardwareMap.appContext);

       verticalArm = new VerticalArm(hardwareMap, telemetry);
       claw = new Claw(hardwareMap, telemetry);
       drive = new MecanumDrive(hardwareMap, telemetry);
//       intake = new Intake(hardwareMap, telemetry);

       recordingTimer = new ElapsedTime();


    }

    public boolean isWithinTolerance(double targetValue, double currentValue, double tolerance ){
        return Math.abs(targetValue-currentValue) <= tolerance;
    }

    @Override
    public void loop() {
        telemetry.addData("Is Recording:", isRecording);
        this.telemetry.update();

        if (isRecording && recordingTimer.seconds() >= 15.0) {
            recorder.stopRecording();
            isRecording = false;
            gamepad1.rumble(250);
        }

        if(!isWithinTolerance(0, gamepad1.left_stick_y, 0.1) || !isWithinTolerance(0, gamepad1.left_stick_x, 0.1) || !isWithinTolerance(0, gamepad1.right_stick_x, 0.1)){
            drive.driveFieldRelative(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);

            recorder.giveCommand("drive.driveFieldRelative(" + gamepad1.left_stick_x + ", " + gamepad1.left_stick_y + ", " + gamepad1.right_stick_x + ");");
        } else{
            drive.stop();
            recorder.giveCommand("drive.stop();");
        }

        if(gamepad1.y){
            drive.testMotors();
        }

        if (!isWithinTolerance(0, gamepad2.left_stick_y, 0.1)) {
            verticalArm.moveUp(gamepad2.left_stick_y);
            recorder.giveCommand("verticalArm.moveUp(" + gamepad2.left_stick_y + ");");
        } else {
            verticalArm.stop();
            recorder.giveCommand("verticalArm.stop();");
        }

        if (gamepad1.a) {
            claw.openClaw();
            recorder.giveCommand("claw.closeClaw();");
        } else if (gamepad1.b) {
            claw.closeClaw();
            recorder.giveCommand("claw.openClaw();");
        }

//        if(gamepad2.a){
//            isAPressed = !isAPressed;
//        }

//        if(isAPressed){
//            intake.collect();
//        } else if(!isAPressed){
//            intake.stop();
//        }
//
//        if(gamepad2.b){
//            isBPressed = !isBPressed;
//        }
//
//        if(isBPressed){
//            intake.rotateUp();
//        } else if(!isBPressed){
//            intake.rotateDown();
//        }
//
//        if(gamepad2.x){
//            intake.passSample();
//            claw.closeClaw();
//        }

        if (gamepad1.x) {
            if (recorder.startRecording()) {
                isRecording = true;
                recordingTimer.reset();
                gamepad1.rumble(250);
            }
        }
        telemetry.update();


    }


}
