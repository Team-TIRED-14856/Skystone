package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.Constants.*;

@Autonomous(name = "Linear OPMode", group = "Linear Opmode")
public class LinearOPTest extends LinearOpMode {

    TiredBot robot = new TiredBot();

    private ElapsedTime runtime = new ElapsedTime();

    double startGyro, p, absoluteGyro, prevGyro, currentGyro;

    Thread gyroThread;

    @Override
    public void runOpMode() throws InterruptedException {

    //Initialiation---------------------------------------------------------------------------------------------
        robot.init(hardwareMap);
        gyroThread = new GyroThread();
        gyroThread.start();
    //End Initialization----------------------------------------------------------------------------------------

        //The name is wait for start, what do you think it does?
        waitForStart();

    //Auto Mode-------------------------------------------------------------------------------------------------

        strafeSeconds(1,.5);

        heckingStop();

        gyroThread.interrupt();

    }

    /*
        Mecanum Drive Values
        fLSpeed = gamepad1.right_stick_x + gamepad1.left_stick_y - gamepad1.left_stick_x;
        bLSpeed = gamepad1.right_stick_x + gamepad1.left_stick_y + gamepad1.left_stick_x;
        fRSpeed = gamepad1.right_stick_x - gamepad1.left_stick_y - gamepad1.left_stick_x;
        bRSpeed = gamepad1.right_stick_x - gamepad1.left_stick_y + gamepad1.left_stick_x;
     */


    public void driveForwardSeconds(double sec, double power){
        if(opModeIsActive()){
            runtime.reset();
            startGyro = absoluteGyro;
            while(opModeIsActive() && runtime.milliseconds() < sec*1000){
                p = kpDrive*(startGyro - absoluteGyro);

                robot.frontLeft.setPower(-power - p);
                robot.backLeft.setPower(-power - p);
                robot.frontRight.setPower(power - p);
                robot.backRight.setPower(power - p);
            }
            robot.frontLeft.setPower(0);
            robot.backLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backRight.setPower(0);
        }
    }

    public void driveToColor(double timeout, double power){
        if(opModeIsActive()){
            runtime.reset();
            startGyro = absoluteGyro;
            while(opModeIsActive() && runtime.milliseconds() < timeout*1000 && (robot.color.blue() > kBlueThreshold || robot.color.red() > kRedThreshold)){
                p = kpDrive*(startGyro - absoluteGyro);

                robot.frontLeft.setPower(-power - p);
                robot.backLeft.setPower(-power - p);
                robot.frontRight.setPower(power - p);
                robot.backRight.setPower(power - p);
            }
            robot.frontLeft.setPower(0);
            robot.backLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backRight.setPower(0);
        }
    }

    public void strafeToColor(double timeout, double power){
        if(opModeIsActive()){
            runtime.reset();
            startGyro = absoluteGyro;
            while(opModeIsActive() && runtime.milliseconds() < timeout*1000 && (robot.color.blue() > kBlueThreshold || robot.color.red() > kRedThreshold)){
                p = kpDrive*(startGyro - absoluteGyro);

                robot.frontLeft.setPower(-power - p);
                robot.backLeft.setPower(power - p);
                robot.frontRight.setPower(-power - p);
                robot.backRight.setPower(power - p);
            }
            robot.frontLeft.setPower(0);
            robot.backLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backRight.setPower(0);
        }
    }

    public void turnDegrees(double deg, double power){
        if(opModeIsActive()){
            double actualDegree = absoluteGyro + deg;
            double p;

            //Left
            if(deg > 0) {
                while(opModeIsActive() && actualDegree - absoluteGyro > 1){
                    p = (actualDegree - absoluteGyro) * kpTurn;

                    robot.frontLeft.setPower(-power - p);
                    robot.frontRight.setPower(-power - p);
                    robot.backLeft.setPower(-power - p);
                    robot.backRight.setPower(-power - p);
                }
            }
            //Right
            else{
                while(opModeIsActive() && absoluteGyro - actualDegree > 1){
                    p = (absoluteGyro - actualDegree) * kpTurn;

                    robot.frontLeft.setPower(power + p);
                    robot.frontRight.setPower(power + p);
                    robot.backLeft.setPower(power + p);
                    robot.backRight.setPower(power + p);
                }
            }

            //Done
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);
        }
    }

    public void strafeSeconds(double sec, double power){
        if(opModeIsActive()){
            runtime.reset();
            startGyro = absoluteGyro;
            while(opModeIsActive() && runtime.milliseconds() < sec*1000){
                p = kpDrive*(startGyro - absoluteGyro);

                robot.frontLeft.setPower(-power - p);
                robot.backLeft.setPower(power - p);
                robot.frontRight.setPower(-power - p);
                robot.backRight.setPower(power - p);
            }
            robot.frontLeft.setPower(0);
            robot.backLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backRight.setPower(0);
        }
    }

    public void hook(boolean down){
        if(opModeIsActive()) {
            if (down) {
                robot.hook1.setPosition(kHook1Down);
                robot.hook2.setPosition(kHook2Down);
            } else {
                robot.hook1.setPosition(kHook1Up);
                robot.hook2.setPosition(kHook2Up);
            }
        }
    }

    public void chill(double sec){
        if(opModeIsActive()) {
            runtime.reset();
            while (opModeIsActive() && runtime.milliseconds() < sec * 1000) {
                //do nothing
            }
        }
    }

    public void heckingStop(){
        if(opModeIsActive()){
            robot.frontLeft.setPower(0);
            robot.backLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backRight.setPower(0);
        }
    }

    private class GyroThread extends Thread{

        public GyroThread(){
            absoluteGyro = robot.imu.getAngularOrientation().firstAngle;
            prevGyro = robot.imu.getAngularOrientation().firstAngle;
        }

        public void run(){

            try {
                while(!isInterrupted()) {
                    //Gyro Correction
                    currentGyro = robot.imu.getAngularOrientation().firstAngle;
                    if (Math.abs(prevGyro - currentGyro) > 180) {
                        if (prevGyro < 0 && currentGyro > 0) {
                            absoluteGyro += -180 - prevGyro + currentGyro - 180;
                        } else if (prevGyro > 0 && currentGyro < 0) {
                            absoluteGyro += 180 - prevGyro + currentGyro + 180;
                        }
                    } else {
                        absoluteGyro += currentGyro - prevGyro;
                    }
                    //Set the prevGyro variable for future gyro stuffs
                    prevGyro = currentGyro;

                    telemetry.addData("Gyro", robot.imu.getAngularOrientation().firstAngle);
                    telemetry.addData("Absolute Gyro", absoluteGyro);
                    telemetry.update();
                }
            } catch(Exception e){}
        }
    }
}
