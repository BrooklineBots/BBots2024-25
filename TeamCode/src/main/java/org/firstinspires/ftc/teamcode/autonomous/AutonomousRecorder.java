package org.firstinspires.ftc.teamcode.autonomous;

import android.os.Environment;

import java.io.FileWriter;
import java.io.IOException;

public class AutonomousRecorder {
    private FileWriter fileWriter;
    private boolean isRecording = false;
    private boolean hasRecorded = false;

    public AutonomousRecorder() {
        try {
            fileWriter = new FileWriter(Environment. getExternalStorageDirectory().getPath() + "/AutonomousRecord.java");
            writeHeader();
        } catch (IOException e) {
            // Handle exception, perhaps log it?
            isRecording = false;
        }
    }

    public void giveCommand(String command) {
        if (!isRecording) return;

        try {
            fileWriter.write(command + "\n");
        } catch (IOException e) {
            // Handle exception
        }
    }

    private void writeHeader() throws IOException {
        fileWriter.write("// Auto-generated by AutonomousRecorder\n");
        fileWriter.write("package robotcontrol;\n");
        fileWriter.write("\n");
        fileWriter.write("public class AutonomousPath {\n");
        fileWriter.write("    public static void main(String[] args) {\n");
        fileWriter.write("        // Motor control commands go here\n");
    }

    public boolean startRecording() {
        if (hasRecorded) {
            return false;
        }
        hasRecorded = true;
        isRecording = true;
        return true;
    }

    public void stopRecording() {
        isRecording = false;
        try {
            fileWriter.write("}\n");
            fileWriter.close();
        } catch (IOException e) {
            // Handle exception
        }
    }

    public boolean getIsRecording() {
        return isRecording;
    }
}
