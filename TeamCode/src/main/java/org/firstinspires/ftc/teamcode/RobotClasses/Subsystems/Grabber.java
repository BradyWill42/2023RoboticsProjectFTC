package org.firstinspires.ftc.teamcode.RobotClasses.Subsystems;

//import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
//import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion;

import android.text.method.Touch;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class Grabber {
    private DcMotor uptakeOg, uptakeNew;
    private Servo grabber;
    private TouchSensor touchSensor;
    private final double grabberOpen, grabberClosed;
    boolean temp = false;

    public Grabber(DcMotor uptakeOg, DcMotor uptakeNew, Servo grabber, TouchSensor touchSensor) {
        this.uptakeOg = uptakeOg;
        this.uptakeNew = uptakeNew;
        this.grabber = grabber;
        this.touchSensor = touchSensor;

        grabberOpen = 1;
        grabberClosed = 0;
        uptakeOg.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        uptakeOg.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        uptakeNew.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        uptakeNew.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public boolean isPressed(){
        if(touchSensor.isPressed()){
            uptakeOg.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            uptakeNew.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            uptakeOg.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            uptakeNew.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        return touchSensor.isPressed();
    }

    public double getUptakePower(){
        return (uptakeOg.getPower() + -uptakeNew.getPower()) / 2;
    }

    public void down() {
        if(!isPressed()) {
            uptakeOg.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            uptakeOg.setPower(1);
            uptakeNew.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            uptakeNew.setPower(-1);
        }
    }

    public void up() {
        uptakeOg.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        uptakeOg.setPower(-1);
        uptakeNew.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        uptakeNew.setPower(1);
    }

    public void upSlow(){
        uptakeOg.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        uptakeOg.setPower(-0.6);
        uptakeNew.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        uptakeNew.setPower(0.6);
    }

    public void downSlow(){
        if(!isPressed()) {
            uptakeOg.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            uptakeOg.setPower(0.6);
            uptakeNew.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            uptakeNew.setPower(-0.6);
        }
    }


    public void uptakeSetDown() {
        //uptakeOg.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        while(uptakeOg.getCurrentPosition() < -1000){
//            uptakeOg.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            uptakeOg.setPower(1);
//            uptakeNew.setPower(-1);
//            //uptakeOg.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            if(touchSensor.isPressed() || uptakeOg.getCurrentPosition() >= -1000){
//                if(touchSensor.isPressed()){
//                    uptakeOg.setPower(0);
//                    uptakeNew.setPower(0);
//                    isPressed();
//                    break;
//                }
//                uptakeOg.setPower(0);
//                uptakeNew.setPower(0);
//                break;
//            }
        uptakeOg.setTargetPosition(-250);
        uptakeNew.setTargetPosition(250);

        uptakeNew.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        uptakeOg.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if(uptakeOg.getCurrentPosition() < uptakeOg.getTargetPosition()){
            uptakeOg.setPower(1);
            uptakeNew.setPower(-1);
        }
    }

    public void uptakeSetUp() {
        uptakeOg.setTargetPosition(-3000);
        uptakeNew.setTargetPosition(3000);

        uptakeNew.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        uptakeOg.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if(uptakeOg.getCurrentPosition() > uptakeOg.getTargetPosition()) {
            uptakeOg.setPower(-1);
            uptakeNew.setPower(1);
        }
    }

    public void off() {
        uptakeOg.setPower(0);
        uptakeNew.setPower(0);
    }

    public void resetuptakeOgEncoder() {
        uptakeOg.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        uptakeOg.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        uptakeNew.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        uptakeNew.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void openGrabber() {
        grabber.setPosition(grabberOpen);
    }

    public void closeGrabber() {
        grabber.setPosition(grabberClosed);
    }

    public double getPosition(){
        return grabber.getPosition();
    }

    public double getEncoder(){
        return (uptakeOg.getCurrentPosition() - uptakeNew.getCurrentPosition()) / 2;
    }

    public void setPosition(double pos){
        grabber.setPosition(pos);

    }

}
