package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import static com.qualcomm.hardware.bosch.BNO055IMU.AngleUnit.DEGREES;

@Autonomous(name="Drive to line", group="Linear Opmode")
public class DriveToLine extends com.qualcomm.robotcore.eventloop.opmode.OpMode {

    ColorSensor color;

    DcMotor frontLeft,
            frontRight,
            backLeft,
            backRight;

    Servo hook1, hook2;

    BNO055IMU imu;

    double absoluteGyro, prevGyro, currentGyro;

    public void init() {
        //Color sensor
        color = hardwareMap.get(ColorSensor.class,"color");

        //Motors
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        //Servo
        hook1 = hardwareMap.get(Servo.class, "hook1");
        hook2 = hardwareMap.get(Servo.class, "hook2");

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
        if(initTime){
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

        //Init Start Gyro
        if(initStartGyro){
            startGyro = absoluteGyro;
            initStartGyro = false;
        }

        //Its Switch Time
        switch(iterationsV1){
            case 0:
                driveForwardSeconds(4,0.5,startGyro);
                if(color.red() > 45 || color.blue() > 55){
                    reset();
                    iterationsV1++;
                    stop();
                }
                break;
            case 1:
                if(!driveForwardSeconds(0.2,-0.1,startGyro)){
                    reset();
                    iterationsV1++;
                    stop();
                }
                break;
        }

        //Outputs for Debugging
        telemetry.addData("Absolute Gyro Position", absoluteGyro);
        telemetry.addData("Gyro Position", imu.getAngularOrientation().firstAngle);
        telemetry.update();

    }

    public boolean turnDegrees(int degree, double power, double startGyro){
        double actualDegree = startGyro + degree;
        double p = 0;
        double kp = 0.0001;

        //Turn Left
        if(degree > 0){
            if(actualDegree - absoluteGyro <= 1){
                frontLeft.setPower(0);
                frontRight.setPower(0);
                backLeft.setPower(0);
                backRight.setPower(0);
                return false;
            }
            p = (actualDegree - absoluteGyro) * kp;

            frontLeft.setPower(power + p);
            frontRight.setPower(power + p);
            backLeft.setPower(power + p);
            backRight.setPower(power + p);
        }

        //Turn Right
        else if(degree < 0){
            if(absoluteGyro - actualDegree <= 1){
                frontLeft.setPower(0);
                frontRight.setPower(0);
                backLeft.setPower(0);
                backRight.setPower(0);
                return false;
            }
            p = (absoluteGyro - actualDegree) * kp;

            frontLeft.setPower(-power - p);
            frontRight.setPower(-power - p);
            backLeft.setPower(-power - p);
            backRight.setPower(-power - p);
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
        frontLeft.setPower(power + p);
        backLeft.setPower(power + p);
        frontRight.setPower(-power + p);
        backRight.setPower(-power + p);
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
        double kp = 0.06;
        double p = error*kp;

        if(time - initialTime > seconds){
            frontLeft.setPower(0);
            backLeft.setPower(0);
            frontRight.setPower(0);
            backRight.setPower(0);
            return false;
        }
        frontLeft.setPower(power + p);
        backLeft.setPower(-power + p);
        frontRight.setPower(power + p);
        backRight.setPower(-power + p);
        return true;
    }

    /**
     *
     * @param pos the position of the servo, 1 is down and -1 is up
     */
    public void hook(double pos){
        hook1.setPosition((pos+1)/2);
        hook2.setPosition((-pos+1)/2);
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