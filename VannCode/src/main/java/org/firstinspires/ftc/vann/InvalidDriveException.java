package org.firstinspires.ftc.vann;

public class InvalidDriveException extends Exception {

    public InvalidDriveException(DriveController.WheelType wheelType) {
        super("Selected drive is not compatible with " + wheelType.name() + "!");
    }
}
