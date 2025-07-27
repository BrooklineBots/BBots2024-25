package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;

public class HomingSensor {
  private final TouchSensor homingSensor;

  private final Telemetry telemetry;
  private final HardwareMap hwMap;

  public HomingSensor(HardwareMap hwMap, Telemetry telemetry) {
    this.hwMap = hwMap;
    this.telemetry = telemetry;

    homingSensor = hwMap.touchSensor.get(Constants.HomingSensorConstants.SENSOR_ID);
  }

  public boolean isPressed() {
    return homingSensor.isPressed();
  }
}
