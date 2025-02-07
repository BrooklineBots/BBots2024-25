package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Claw;
import org.firstinspires.ftc.teamcode.Subsystems.MecanumDrive;

@TeleOp(name = "testDrive2")
public class DriveOpMode6 extends OpMode {

    private Arm arm;
    private Claw claw;
    private MecanumDrive drive;

    @Override
    public void init() {
       arm = new Arm(hardwareMap, telemetry);
       claw = new Claw(hardwareMap, telemetry);
       drive = new MecanumDrive();

    }

    public boolean isWithinTolerance(double targetValue, double currentValue, double tolerance ){
        return Math.abs(targetValue-currentValue) <= tolerance;
    }

    @Override
    public void loop() {

        if(!isWithinTolerance(0, gamepad1.left_stick_y, 0.1) || !isWithinTolerance(0, gamepad1.left_stick_x, 0.1) || !isWithinTolerance(0, gamepad1.right_stick_x, 0.1)){
            drive.drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        } else{
            drive.stopMotors();
        }

        if (!isWithinTolerance(0, gamepad2.left_stick_y, 0.1)) {
            arm.moveUp(gamepad2.left_stick_y);
        } else {
            arm.stop();
        }

        if (gamepad1.a) {
            claw.closeClaw();
        } else if (gamepad1.b) {
            claw.openClaw();
        }


    }


}
