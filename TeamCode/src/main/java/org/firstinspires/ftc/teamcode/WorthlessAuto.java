package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import static com.qualcomm.hardware.bosch.BNO055IMU.AngleUnit.DEGREES;

@Autonomous(name="Worthless", group="Linear Opmode")
public class WorthlessAuto extends com.qualcomm.robotcore.eventloop.opmode.OpMode {

    DcMotor frontLeft,
            frontRight,
            backLeft,
            backRight;

    BNO055IMU imu;

    double absoluteGyro, prevGyro, currentGyro;

    public void init() {

        //Motors
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

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

        //Variables
        absoluteGyro = imu.getAngularOrientation().firstAngle;
        prevGyro = imu.getAngularOrientation().firstAngle;
    }

    int iterationsV1 = 0;
    boolean initTime = true;
    double initialTime = 0;
    boolean initStartGyro = true;
    double startGyro = 0;
    public void loop() {

        //Set intial time and variables
        if(initTime) {
            initialTime = time;
            initTime = false;
        }

        //Gyro Correction
        currentGyro = imu.getAngularOrientation().firstAngle;
        if(Math.abs(prevGyro - currentGyro) > 180) {
            if (prevGyro < 0 && currentGyro > 0) {
                absoluteGyro += -180-prevGyro + currentGyro-180;
            }
            else if (prevGyro > 0 && currentGyro < 0){
                absoluteGyro += 180-prevGyro + currentGyro+180;
            }
        }
        else{
            absoluteGyro += currentGyro - prevGyro;
        }
        //Set the prevGyro variable for future gyro stuffs
        prevGyro = currentGyro;

//        //Its Switch Time
//        switch(iterationsV1){
//
//        }

        //Outputs for Debugging
        telemetry.addData("Absolute Gyro Position", absoluteGyro);
        telemetry.addData("Gyro Position", imu.getAngularOrientation().firstAngle);
        telemetry.addData("iterations", iterationsV1);
        telemetry.update();

    }

    public boolean turnTo(int degree, double power){
        double actualDegree = ((absoluteGyro%360 + 360)%360 - degree + 360)%360;
        double p = 0;
        double kp = 0.0001;

        //Turn Left
        if(actualDegree >= 180){
            if(actualDegree <= 181){
                frontLeft.setPower(0);
                frontRight.setPower(0);
                backLeft.setPower(0);
                backRight.setPower(0);
                return false;
            }
            p = (actualDegree - 180) * kp;

            frontLeft.setPower(-power - p);
            frontRight.setPower(-power - p);
            backLeft.setPower(-power - p);
            backRight.setPower(-power - p);
        }

        //Turn Right
        else if(actualDegree < 180){
            if(actualDegree <= 1){
                frontLeft.setPower(0);
                frontRight.setPower(0);
                backLeft.setPower(0);
                backRight.setPower(0);
                return false;
            }
            p = actualDegree * kp;

            frontLeft.setPower(power + p);
            frontRight.setPower(power + p);
            backLeft.setPower(power + p);
            backRight.setPower(power + p);
        }
        return true;
    }

    public boolean driveForwardSeconds(double seconds, double power, double startGyro){

        double error = startGyro - absoluteGyro;
        double kp = 0.045;
        double p = error*kp;

        if(time - initialTime > seconds){
            frontLeft.setPower(0);
            backLeft.setPower(0);
            frontRight.setPower(0);
            backRight.setPower(0);
            return false;
        }
        frontLeft.setPower(power - p);
        backLeft.setPower(power - p);
        frontRight.setPower(-power - p);
        backRight.setPower(-power - p);
        return true;
    }

    public boolean driveForwardSecondsNoGyro(double seconds, double power){

        if(time - initialTime > seconds){
            frontLeft.setPower(0);
            backLeft.setPower(0);
            frontRight.setPower(0);
            backRight.setPower(0);
            return false;
        }
        frontLeft.setPower(power);
        backLeft.setPower(power);
        frontRight.setPower(-power);
        backRight.setPower(-power);
        return true;
    }

    public boolean strafeSeconds(double seconds, double power, double startGyro){

        double error = startGyro - absoluteGyro;
        double kp = 0.025;
        double p = error*kp;

        if(time - initialTime > seconds){
            frontLeft.setPower(0);
            backLeft.setPower(0);
            frontRight.setPower(0);
            backRight.setPower(0);
            return false;
        }
        frontLeft.setPower(power - p);
        backLeft.setPower(-power - p);
        frontRight.setPower(power - p);
        backRight.setPower(-power - p);
        return true;
    }

    public boolean chill(double sec){
        if(time - initialTime > sec){
            return false;
        }
        return true;
    }

    public void reset(){
        initialTime = time;
        initStartGyro = true;
    }

    public void stop(){
        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
    }

}
