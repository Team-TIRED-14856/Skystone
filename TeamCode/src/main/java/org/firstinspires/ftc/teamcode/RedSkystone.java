package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import static org.firstinspires.ftc.teamcode.Constants.kBlueThreshold;
import static org.firstinspires.ftc.teamcode.Constants.kHook1Down;
import static org.firstinspires.ftc.teamcode.Constants.kHook1Up;
import static org.firstinspires.ftc.teamcode.Constants.kHook2Down;
import static org.firstinspires.ftc.teamcode.Constants.kHook2Up;
import static org.firstinspires.ftc.teamcode.Constants.kRedThreshold;
import static org.firstinspires.ftc.teamcode.Constants.kpDrive;
import static org.firstinspires.ftc.teamcode.Constants.kpStrafe;
import static org.firstinspires.ftc.teamcode.Constants.kpTurn;

@Autonomous(name = "Red Skystone", group = "Linear Opmode")
public class RedSkystone extends LinearOpMode {

    TiredBot robot = new TiredBot();

    private ElapsedTime runtime = new ElapsedTime();

    double startGyro, p, absoluteGyro, prevGyro, currentGyro;

    String stonePos = "middle";

    Thread gyroThread;

    OpenCvCamera phoneCam;

    VisionProcessing vision;

    @Override
    public void runOpMode() throws InterruptedException {

    //Initialiation---------------------------------------------------------------------------------------------
        robot.init(hardwareMap);
        gyroThread = new GyroThread();
        gyroThread.start();

        //Camera Stuff

        //Initialize vision object
        vision = new VisionProcessing();

        //Instantiate the OpenCvCamera object
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        //Open connection to the camera
        phoneCam.openCameraDevice();

        //Set the vision processing pipeline to the one we want
        phoneCam.setPipeline(vision);

        //Tell the camera to start streaming images to us
        phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

    //End Initialization----------------------------------------------------------------------------------------

        //The name is wait for start, what do you think it does?
        waitForStart();

    //Auto Mode-------------------------------------------------------------------------------------------------

        chill(0.1);

        driveForwardSeconds(1.2, -0.5);

        strafeSeconds(1.5, 0.5);

        driveToSkystone(10,0.3);

        strafeSeconds(1.2, 0.5);

        hook(true);

        chill(1.5);

        strafeSeconds(1.2, -0.5);

        turnDegrees(87, 0.5);

        hook(false);

        strafeToColor(9, 0.4);

        strafeSeconds(.7, 1);

        strafeToColor(3, -0.4);

        if(stonePos == "inner" || stonePos == "outer"){
            strafeSeconds(1.75,-0.5);
        }
        else{
            strafeSeconds(2.25, -0.5);
        }

        turnDegrees(-87, 0.5);

        strafeSeconds(1.2, 0.5);

        hook(true);

        strafeSeconds(1.2, -0.5);

        turnDegrees(87, 0.5);

        hook(false);

        strafeToColor(3, 0.4);

        strafeSeconds(.7, 1);

        strafeToColor(3, -0.4);

        heckingStop();

        chill(10);

        gyroThread.interrupt();

    }

    /*
        //Mecanum Drive values
        fLSpeed = - gamepad1.right_stick_x - gamepad1.left_stick_y + gamepad1.left_stick_x;
        bLSpeed = - gamepad1.right_stick_x - gamepad1.left_stick_y - gamepad1.left_stick_x;
        fRSpeed = - gamepad1.right_stick_x + gamepad1.left_stick_y + gamepad1.left_stick_x;
        bRSpeed = - gamepad1.right_stick_x + gamepad1.left_stick_y - gamepad1.left_stick_x;
     */


    public void driveForwardSeconds(double sec, double power){
        if(opModeIsActive()){
            runtime.reset();
            startGyro = absoluteGyro;
            while(opModeIsActive() && runtime.milliseconds() < sec*1000){
                p = kpDrive*(startGyro - absoluteGyro);

                robot.frontLeft.setPower(power + p);
                robot.backLeft.setPower(power + p);
                robot.frontRight.setPower(-power + p);
                robot.backRight.setPower(-power + p);
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
            while(opModeIsActive() && runtime.milliseconds() < timeout*1000 && robot.color.blue() < kBlueThreshold && robot.color.red() < kRedThreshold){
                p = kpDrive*(startGyro - absoluteGyro);

                robot.frontLeft.setPower(power + p);
                robot.backLeft.setPower(power + p);
                robot.frontRight.setPower(-power + p);
                robot.backRight.setPower(-power + p);
            }
            robot.frontLeft.setPower(0);
            robot.backLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backRight.setPower(0);
        }
    }

    public void driveToSkystone(double timeout, double power){
        if(opModeIsActive()){
            runtime.reset();
            startGyro = absoluteGyro;
            while(opModeIsActive() && runtime.milliseconds() < timeout*1000 && !vision.isStone){
                p = kpDrive*(startGyro - absoluteGyro);

                robot.frontLeft.setPower(power + p);
                robot.backLeft.setPower(power + p);
                robot.frontRight.setPower(-power + p);
                robot.backRight.setPower(-power + p);
            }
            robot.frontLeft.setPower(0);
            robot.backLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backRight.setPower(0);

            if(runtime.milliseconds() < 750){
                stonePos = "middle";
            }
            else if(runtime.milliseconds() < 1500){
                stonePos = "outer";
            }
            else{
                stonePos = "inner";
            }
        }
    }

    public void strafeToColor(double timeout, double power){
        if(opModeIsActive()){
            runtime.reset();
            startGyro = absoluteGyro;
            while(opModeIsActive() && runtime.milliseconds() < timeout*1000 && robot.color.blue() < kBlueThreshold && robot.color.red() < kRedThreshold){
                p = kpStrafe*(startGyro - absoluteGyro);

                robot.frontLeft.setPower(power + p);
                robot.backLeft.setPower(-power + p);
                robot.frontRight.setPower(power + p);
                robot.backRight.setPower(-power + p);
            }
            robot.frontLeft.setPower(0);
            robot.backLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backRight.setPower(0);
        }
    }

    public void turnDegrees(double deg, double power){
        if(opModeIsActive()){
            double goal = absoluteGyro + deg;
            double p;

            //Left
            if(deg > 0) {
                while(opModeIsActive() && goal - absoluteGyro > 1){
                    p = (goal - absoluteGyro) * kpTurn;

                    robot.frontLeft.setPower(power + p);
                    robot.frontRight.setPower(power + p);
                    robot.backLeft.setPower(power + p);
                    robot.backRight.setPower(power + p);
                }
            }
            //Right
            else{
                while(opModeIsActive() && absoluteGyro - goal > 1){
                    p = (absoluteGyro - goal) * kpTurn;

                    robot.frontLeft.setPower(-power - p);
                    robot.frontRight.setPower(-power - p);
                    robot.backLeft.setPower(-power - p);
                    robot.backRight.setPower(-power - p);
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
                p = kpStrafe*(startGyro - absoluteGyro);

                robot.frontLeft.setPower(power + p);
                robot.backLeft.setPower(-power + p);
                robot.frontRight.setPower(power + p);
                robot.backRight.setPower(-power + p);
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

                    telemetry.addData("Is Stone", vision.isStone);
                    telemetry.addData("Stone x", vision.stoneX);
                    telemetry.addData("Stone y", vision.stoneY);
                    telemetry.addData("Gyro", robot.imu.getAngularOrientation().firstAngle);
                    telemetry.addData("Absolute Gyro", absoluteGyro);
                    telemetry.addData("Stone Position", stonePos);
                    telemetry.update();
                }
            } catch(Exception e){}
        }
    }
}
