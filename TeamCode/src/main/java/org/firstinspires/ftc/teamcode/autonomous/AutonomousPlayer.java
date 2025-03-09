package org.firstinspires.ftc.teamcode.autonomous;

import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Subsystems.*;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "Autonomous Player", group = "Playback")
public class AutonomousPlayer extends LinearOpMode {
    // Subsystems
    private MecanumDrive drive;
    private VerticalArm verticalArm;
    private Claw claw;
//    private Intake intake; //TODO: UNcomment me

    // Data structure for recorded states
    private static class Record {
        long timestamp;
        double fl, fr, bl, br;
        double leftArm, rightArm;
        double claw, intakeLeft, intakeRight, leftFlipper, rightFlipper;
    }

    @Override
    public void runOpMode() {
        initializeSubsystems();
        List<Record> records = readCSV("autonomous_data.csv");

        waitForStart();
        ElapsedTime timer = new ElapsedTime();

        for (Record record : records) {
            // Wait until the target timestamp
            while (timer.milliseconds() < record.timestamp && opModeIsActive()) {
                sleep(1);
            }

            // Apply motor/servo values
            drive.setExactMotorPowers(record.fl, record.fr, record.bl, record.br);
            verticalArm.setArmPowers(record.leftArm, record.rightArm);
            claw.setPosition(record.claw);
//            intake.setIntakePowers(record.intakeLeft, record.intakeRight);//TODO: UNcomment me
//            intake.setFlipperPos(record.leftFlipper, record.rightFlipper);//TODO: UNcomment me
        }

        // Stop all motors at the end
        drive.stop();
        verticalArm.stop();
//        intake.stopWheels();//TODO: UNcomment me
    }

    private void initializeSubsystems() {
        drive = new MecanumDrive(hardwareMap, telemetry);
        verticalArm = new VerticalArm(hardwareMap, telemetry);
        claw = new Claw(hardwareMap, telemetry);
//        intake = new Intake(hardwareMap, telemetry);//TODO: UNcomment me
    }

    private List<Record> readCSV(String filename) {
        List<Record> records = new ArrayList<>();
        File directory = hardwareMap.appContext.getExternalFilesDir(Environment.DIRECTORY_DOCUMENTS);
        File file = new File(directory, filename);

        try (BufferedReader br = new BufferedReader(new FileReader(file))) {
            String line;
            br.readLine(); // Skip CSV header
            while ((line = br.readLine()) != null) {
                String[] values = line.split(",");
                Record record = new Record();
                record.timestamp = Long.parseLong(values[0]);
                record.fl = parseDoubleOrZero(values[1]);
                record.fr = parseDoubleOrZero(values[2]);
                record.bl = parseDoubleOrZero(values[3]);
                record.br = parseDoubleOrZero(values[4]);
                record.leftArm = Double.parseDouble(values[5]);
                record.rightArm = Double.parseDouble(values[6]);
                record.claw = Double.parseDouble(values[7]);
//                record.intakeLeft = Double.parseDouble(values[8]);
//                record.intakeRight = Double.parseDouble(values[9]);
//                record.leftFlipper = Double.parseDouble(values[10]);
//                record.rightFlipper = Double.parseDouble(values[11]);
                records.add(record);
            }
        } catch (IOException | NumberFormatException e) {
            telemetry.log().add("Failed to read CSV: " + e.getMessage());
        }
        return records;
    }

    // Helper method to handle NaN
    private double parseDoubleOrZero(String value) {
        try {
            double parsed = Double.parseDouble(value);
            return Double.isNaN(parsed) ? 0.0 : parsed;
        } catch (NumberFormatException e) {
            return 0.0;
        }
    }
}