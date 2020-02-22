package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;


/**
 * Created by ryanf on 11/16/2018.
 */
@Autonomous(name="AutoVision", group="Linear Opmode")
public class AutoVision extends OpMode{


    OpenCvCamera phoneCam;

    VisionProcessing vision;


    @Override
    public void init() {

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

    }

    @Override
    public void loop() {

        //Get Data to the phone
        telemetry.addData("Skystone Center X", vision.stoneX);
        telemetry.addData("Skystone Center Y", vision.stoneY);
        telemetry.addData("Is Stone", vision.isStone);
        telemetry.update();
    }
}