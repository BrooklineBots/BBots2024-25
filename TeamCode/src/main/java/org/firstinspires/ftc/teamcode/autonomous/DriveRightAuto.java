package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.Subsystems.MecanumDrive;

@Autonomous(name = "Drive Right", group = "Autonomous")
public class DriveRightAuto extends LinearOpMode {
    private MecanumDrive drive;

    @Override
    public void runOpMode() {
        this.drive = new MecanumDrive(hardwareMap, telemetry);
        waitForStart();
        drive.driveFieldRelative(0, -1, 0);
        sleep(5000);
        drive.driveFieldRelative(0, 0, 0);

    }

}
