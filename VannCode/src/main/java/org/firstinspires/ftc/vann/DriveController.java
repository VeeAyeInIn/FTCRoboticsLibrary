package org.firstinspires.ftc.vann;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DriveController {

    private final HardwareMap hardwareMap;
    private final WheelType wheelType;

    private DcMotor rightWheel;
    private DcMotor leftWheel;

    private DcMotor rightFrontWheel;
    private DcMotor rightRearWheel;
    private DcMotor leftFrontWheel;
    private DcMotor leftRearWheel;

    public DriveController(HardwareMap hardwareMap, WheelType wheelType) {

        this.hardwareMap = hardwareMap;
        this.wheelType = wheelType;

        if (wheelType.hasFourWheels()) {
            rightFrontWheel = hardwareMap.dcMotor.get("rightFront");
            rightRearWheel = hardwareMap.dcMotor.get("rightRear");
            leftFrontWheel = hardwareMap.dcMotor.get("leftFront");
            leftRearWheel = hardwareMap.dcMotor.get("leftRear");

            rightFrontWheel.setDirection(DcMotorSimple.Direction.FORWARD);
            rightRearWheel.setDirection(DcMotorSimple.Direction.FORWARD);
            leftFrontWheel.setDirection(DcMotorSimple.Direction.REVERSE);
            leftRearWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        } else {
            rightWheel = hardwareMap.dcMotor.get("rightWheel");
            leftWheel = hardwareMap.dcMotor.get("leftWheel");

            rightWheel.setDirection(DcMotorSimple.Direction.FORWARD);
            leftWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        }
    }

    public void drive(double rightWheelPower, double leftWheelPower) throws InvalidDriveException {

        if (wheelType.hasFourWheels()) {
            throw new InvalidDriveException(wheelType);
        }

        rightWheel.setPower(rightWheelPower);
        leftWheel.setPower(leftWheelPower);
    }

    public void drive(double rightFrontPower, double rightRearPower, double leftFrontPower, double leftRearPower) throws InvalidDriveException {

        if (!wheelType.hasFourWheels()) {
            throw new InvalidDriveException(wheelType);
        }

        rightFrontWheel.setPower(rightFrontPower);
        rightRearWheel.setPower(rightRearPower);
        leftFrontWheel.setPower(leftFrontPower);
        leftRearWheel.setPower(leftRearPower);
    }

    public HardwareMap getHardwareMap() {
        return hardwareMap;
    }

    public WheelType getWheelType() {
        return wheelType;
    }

    public DcMotor getRightWheel() {
        return rightWheel;
    }

    public DcMotor getLeftWheel() {
        return leftWheel;
    }

    public DcMotor getRightFrontWheel() {
        return rightFrontWheel;
    }

    public DcMotor getRightRearWheel() {
        return rightRearWheel;
    }

    public DcMotor getLeftFrontWheel() {
        return leftFrontWheel;
    }

    public DcMotor getLeftRearWheel() {
        return leftRearWheel;
    }

    public enum WheelType {

        TANK,
        OMNI_WHEELS,
        MECANUM_WHEELS;

        public boolean hasFourWheels() {
            return this != TANK;
        }
    }
}
