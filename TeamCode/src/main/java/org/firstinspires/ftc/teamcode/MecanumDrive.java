package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static org.firstinspires.ftc.teamcode.Constants.*;

/**
 * Created by seanr on 9/16/2019.
 */

@TeleOp(name="Mecanum Drive", group="Linear Opmode")
public class MecanumDrive extends com.qualcomm.robotcore.eventloop.opmode.OpMode {

    double fLSpeed, fRSpeed, bLSpeed, bRSpeed, absoluteGyro, prevGyro, currentGyro;

    boolean aPrevState, bPrevState;

    TiredBot robot = new TiredBot();

    public void init() {

        //Init
        robot.init(hardwareMap);

        //Variables
        absoluteGyro = robot.imu.getAngularOrientation().firstAngle;
        prevGyro = robot.imu.getAngularOrientation().firstAngle;
        aPrevState = false;
        bPrevState = false;

    }

    public void loop(){

        //Gyro Correction
        currentGyro = robot.imu.getAngularOrientation().firstAngle;
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

        //Mecanum Drive
        fLSpeed = - gamepad1.right_stick_x - gamepad1.left_stick_y + gamepad1.left_stick_x;
        bLSpeed = - gamepad1.right_stick_x - gamepad1.left_stick_y - gamepad1.left_stick_x;
        fRSpeed = - gamepad1.right_stick_x + gamepad1.left_stick_y + gamepad1.left_stick_x;
        bRSpeed = - gamepad1.right_stick_x + gamepad1.left_stick_y - gamepad1.left_stick_x;

        //Slow speed button
        if(gamepad1.left_bumper){
            fLSpeed *= kSlowSpeed;
            bLSpeed *= kSlowSpeed;
            fRSpeed *= kSlowSpeed;
            bRSpeed *= kSlowSpeed;
        }

        robot.frontLeft.setPower(fLSpeed);
        robot.backLeft.setPower(bLSpeed);
        robot.frontRight.setPower(fRSpeed);
        robot.backRight.setPower(bRSpeed);

        //Scissor lift
        robot.scissor.setPower(-gamepad2.left_stick_y);

        //Slider
        if(gamepad2.left_bumper == true){
            robot.slide.setPosition(0.2);
        }
        else if(gamepad2.right_bumper == true){
            robot.slide.setPosition(1);
        }

        //Hooks
        if(bPrevState == false && gamepad2.b == true){
            if(robot.hook1.getPosition()<0.5){
                robot.hook1.setPosition(1);
                robot.hook2.setPosition(1);
            }
            else{
                robot.hook1.setPosition(0);
                robot.hook2.setPosition(0);
            }
        }
        bPrevState = gamepad2.b;

        //Graber
        if(aPrevState == false && gamepad2.a == true){
            if(robot.grabber.getPosition()<1){
                robot.grabber.setPosition(1);
            }
            else{
                robot.grabber.setPosition(0);
            }
        }
        aPrevState = gamepad2.a;

        //Outputs for Debugging
        telemetry.addData("Absolute Gyro Position", absoluteGyro);
        telemetry.addData("g", robot.color.green());
        telemetry.addData("b", robot.color.blue());
        telemetry.addData("r", robot.color.red());
        telemetry.addData("Hook 1 pos", robot.hook1.getPosition());
        telemetry.addData("Hook 2 pos", robot.hook2.getPosition());
        telemetry.addData("Slider pos", robot.slide.getPosition());
        telemetry.addData("Scissor pos", robot.backLeft.getCurrentPosition());
        telemetry.update();
    }

    //Squares a value but keeps the sign - used to create more accurate driving at lower power levels
    public double squareKeepSign(double in){
        if(in >= 0)
            return in * in;
        else
            return -1 * in * in;
    }
}





