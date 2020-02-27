package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import static com.qualcomm.hardware.bosch.BNO055IMU.AngleUnit.DEGREES;

public class TiredBot {

    DcMotor frontLeft,
            frontRight,
            backLeft,
            backRight,
            scissor;

    BNO055IMU imu;

    Servo hook1, hook2, grabber, slide;

    ColorSensor color;

    HardwareMap hardwareMap;

    public TiredBot(){}

    public void init(HardwareMap hwMap){
        this.hardwareMap = hwMap;

        //Color sensor
        color = hardwareMap.get(ColorSensor.class,"color");

        //Motors
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        scissor = hardwareMap.get(DcMotor.class, "scissor");

        //Servo
        hook1 = hardwareMap.get(Servo.class, "hook1");
        hook2 = hardwareMap.get(Servo.class, "hook2");
        grabber = hardwareMap.get(Servo.class,"grabber");
        slide = hardwareMap.get(Servo.class,"drawer");

        //Gyro
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; //see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }
}
