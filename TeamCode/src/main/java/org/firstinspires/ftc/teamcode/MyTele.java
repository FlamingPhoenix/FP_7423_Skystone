package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "TeleOp", group = "none")
public class MyTele extends OpMode {
    DcMotor fr;
    DcMotor fl;
    DcMotor br;
    DcMotor bl;
    float x1, x2, y1, y2;
    DcMotor intakeMotorLeft;
    DcMotor intakeMotorRight;
    DcMotor slideMotorLeft;
    DcMotor slideMotorRight;
    Servo pullerLeft;
    Servo pullerRight;

    Servo wrist;
    Servo finger;
    Servo shoulder;

    Servo flipper;

    DistanceSensor backLeftDistanceSensor;
    DistanceSensor backRightDistanceSensor;

    TouchSensor touchBack;

    Servo light;

    public long tuckStartTime;
    public boolean isTuckStart = false;

    boolean isGrabbing = false;
    boolean isGrabbed = true;

    boolean isIntaking = false;
    long shoulderAdjustingStartTime;
    boolean isAdjustingShoulder = false;
    boolean isFerryBot = false;

    long shoulderAdjustingWaitTime = 1000;
    long tuckWaitTime = 2000;

    boolean isPlacingStone = false;
    boolean isIntakeDone = false;

    int currentHeight = 0;
    int currentLevel = 0;
    int gotoHeight = 0;

    public void drive(float x1, float y1, float x2) {

        if (gamepad1.left_trigger > 0.5) {
            if (Math.abs(x1) < 0.15) {
                x1 = 0;
            } else {
                x1 *= -(float) 1.8;
            }
            y1 *= -1;
        }

        float frontLeft = y1 + x1 + x2;
        float frontRight = y1 - x1 - x2;
        float backLeft = y1 - x1 + x2;
        float backRight = y1 + x1 - x2;

        frontLeft = Range.clip(frontLeft, -1, 1);
        frontRight = Range.clip(frontRight, -1, 1);
        backLeft = Range.clip(backLeft, -1, 1);
        backRight = Range.clip(backRight, -1, 1);

        if(gamepad1.left_trigger > 0.5) {
            telemetry.addData("backLeftDistanceSensor: ", "%f", backLeftDistanceSensor.getDistance(DistanceUnit.INCH));
            telemetry.addData("backRightDistanceSensor: ", "%f", backRightDistanceSensor.getDistance(DistanceUnit.INCH));
            float reductionFactor = 3;

            double distanceToFoundation = backLeftDistanceSensor.getDistance(DistanceUnit.INCH);
            if (backRightDistanceSensor.getDistance(DistanceUnit.INCH) < distanceToFoundation)
                distanceToFoundation = backRightDistanceSensor.getDistance(DistanceUnit.INCH);

            if (distanceToFoundation < 8) {
                if (distanceToFoundation < 4) {
                    reductionFactor = 4;
                }
                else
                    reductionFactor = 3;
            }

            backLeft = backLeft/reductionFactor;
            frontLeft =  frontLeft/reductionFactor;
            backRight = backRight/reductionFactor;
            frontRight = frontRight/reductionFactor;

            if (backLeftDistanceSensor.getDistance(DistanceUnit.INCH) < 5 && backLeft < 0 && frontLeft < 0 && backLeft == frontLeft){//stops wheels to make robot parallel to foundation
                if (backRightDistanceSensor.getDistance(DistanceUnit.INCH) > backLeftDistanceSensor.getDistance(DistanceUnit.INCH)) {
                    backLeft = 0;
                    frontLeft = 0;
                }
            }

            if (backRightDistanceSensor.getDistance(DistanceUnit.INCH) < 5 && backRight < 0 && frontRight < 0 && backRight == frontRight){//same for right side
                if (backLeftDistanceSensor.getDistance(DistanceUnit.INCH) > backRightDistanceSensor.getDistance(DistanceUnit.INCH))
                {
                    backRight = 0;
                    frontRight = 0;
                }
            }
        }

        fl.setPower(frontLeft);
        fr.setPower(frontRight);
        bl.setPower(backLeft);
        br.setPower(backRight);
    }

    @Override
    public void init() {
        fr = hardwareMap.dcMotor.get("frontright");
        fl = hardwareMap.dcMotor.get("frontleft");
        br = hardwareMap.dcMotor.get("backright");
        bl = hardwareMap.dcMotor.get("backleft");

        intakeMotorLeft = hardwareMap.dcMotor.get("intaketh1");
        intakeMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intakeMotorRight = hardwareMap.dcMotor.get("intaketh2");
        intakeMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intakeMotorLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        slideMotorLeft = hardwareMap.dcMotor.get("slideMotorLeft");
        slideMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        slideMotorRight = hardwareMap.dcMotor.get("slideMotorRight");
        slideMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        slideMotorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        slideMotorRight.setDirection(DcMotorSimple.Direction.REVERSE);

        pullerLeft = hardwareMap.servo.get("pullerLeft");
        ServoControllerEx pullerLeftController = (ServoControllerEx) pullerLeft.getController();
        int pullerLeftServoPort = pullerLeft.getPortNumber();
        PwmControl.PwmRange pullerLeftPwmRange = new PwmControl.PwmRange(899, 1300);
        pullerLeftController.setServoPwmRange(pullerLeftServoPort, pullerLeftPwmRange);
        pullerLeft.setPosition(0);

        pullerRight = hardwareMap.servo.get("pullerRight");
        ServoControllerEx pullerRightController = (ServoControllerEx) pullerRight.getController();
        int pullerRightServoPort = pullerRight.getPortNumber();
        PwmControl.PwmRange pullerRightPwmRange = new PwmControl.PwmRange(1680, 2105);
        pullerRightController.setServoPwmRange(pullerRightServoPort, pullerRightPwmRange);
        pullerRight.setPosition(1);

        wrist = hardwareMap.servo.get("wrist");
        ServoControllerEx wristController = (ServoControllerEx) wrist.getController();
        int wristServoPort = wrist.getPortNumber();
        PwmControl.PwmRange wristPwmRange = new PwmControl.PwmRange(899, 1475);
        wristController.setServoPwmRange(wristServoPort, wristPwmRange);
        wrist.setPosition(0.45);

        finger = hardwareMap.servo.get("finger");
        ServoControllerEx fingerController = (ServoControllerEx) finger.getController();
        int fingerServoPort = finger.getPortNumber();
        PwmControl.PwmRange fingerPwmRange = new PwmControl.PwmRange(899, 1720);
        fingerController.setServoPwmRange(fingerServoPort, fingerPwmRange);
        finger.setPosition(0);

        shoulder = hardwareMap.servo.get("shoulder");
        ServoControllerEx shoulderController = (ServoControllerEx) shoulder.getController();
        int shoulderServoPort = shoulder.getPortNumber();
        PwmControl.PwmRange shoulderPwmRange = new PwmControl.PwmRange(1470, 2300);
        shoulderController.setServoPwmRange(shoulderServoPort, shoulderPwmRange);
        shoulder.setPosition(0);

        flipper = hardwareMap.servo.get("flipper");
        ServoControllerEx flipperController = (ServoControllerEx) flipper.getController();
        int flipperServoPort = flipper.getPortNumber();
        PwmControl.PwmRange flipperPwmRange = new PwmControl.PwmRange(995, 1715);
        flipperController.setServoPwmRange(flipperServoPort, flipperPwmRange);
        flipper.setPosition(0);

        light = hardwareMap.servo.get("lights");
        ServoControllerEx lightController = (ServoControllerEx) light.getController();
        int lightServoPort = light.getPortNumber();
        light.setPosition(0.28);

        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        fl.setDirection(DcMotorSimple.Direction.REVERSE);

        backLeftDistanceSensor = hardwareMap.get(DistanceSensor.class, "backLeftDistanceSensor");
        backRightDistanceSensor = hardwareMap.get(DistanceSensor.class, "backRightDistanceSensor");

        touchBack = hardwareMap.get(TouchSensor.class, "touchSensor");
    }

    @Override
    public void loop() {

        x1 = gamepad1.left_stick_x;
        y1 = gamepad1.left_stick_y;
        x2 = gamepad1.right_stick_x;
        y2 = gamepad1.right_stick_y;

        double joystickLeftDistance = Math.pow(x1, 2) + Math.pow(y1, 2);
        if (joystickLeftDistance < 0.9)
        {
            x1 = x1/2;
            y1 = y1/2;
        }
        double joystickRightDistance = Math.pow(x2, 2) + Math.pow(y2, 2);
        if (joystickRightDistance < 0.9)
        {
            x2 = x2/2;
        }
        drive(x1,  y1 * -1, x2);
        if(gamepad1.dpad_up){
            isFerryBot = true;
        }
        else if(gamepad1.dpad_down){
            isFerryBot = false;
        }

        if (gamepad1.a && !isIntaking && isIntakeDone) {
            flipper.setPosition(1);
        } else if (gamepad1.b) {
            flipper.setPosition(0);
        }

        if (gamepad1.right_trigger > 0.2) //in
        {
            intakeMotorLeft.setPower(1);
            intakeMotorRight.setPower(1);
            isIntaking = true;
            isIntakeDone = false;
            //set flippers to open
            if(!isFerryBot){
                if(slideMotorLeft.getCurrentPosition() > -300){
                    slideMotorLeft.setPower(-1);
                    slideMotorRight.setPower(-1);
                }
                else{
                    slideMotorLeft.setPower(0);
                    slideMotorRight.setPower(0);
                }
            }
            else{
                shoulder.setPosition(0.1);
            }

        }
        else if (gamepad1.right_bumper) //out
        {
            intakeMotorLeft.setPower(-1);
            intakeMotorRight.setPower(-1);
            isIntakeDone = false;
        }
        else //stop when not in or out
        {
            intakeMotorLeft.setPower(0);
            intakeMotorRight.setPower(0);
            isIntaking = false;
            isIntakeDone = true;
        }

        if(gamepad1.left_bumper)
        {
            pullerLeft.setPosition(1);
            pullerRight.setPosition(0);
        }
        else if(gamepad1.left_trigger > 0.2)
        {
            pullerLeft.setPosition(0);
            pullerRight.setPosition(1);
        }

        if (gamepad2.a) {
            shoulder.setPosition(0.6);
        } else if (gamepad2.x) {
            shoulder.setPosition(0.8);
        } else if (gamepad2.y) {
            shoulder.setPosition(1);
        }
        grabStone();
        stackStone();
        moveArm();

        if(gamepad2.dpad_down || isTuckStart) {
            tuckArm();
        }

        if(touchBack.isPressed() && !isGrabbing && isGrabbed) {
            isGrabbing = true;
            isGrabbed = false;
            if(shoulder.getPosition() > 0.01) {
                isAdjustingShoulder = true;
                shoulder.setPosition(0);
                shoulderAdjustingStartTime = System.currentTimeMillis();
                if(slideMotorLeft.getCurrentPosition() < -800) {
                    shoulderAdjustingWaitTime = 100;
                }
                else {
                    shoulderAdjustingWaitTime = 1000 - Math.abs(slideMotorLeft.getCurrentPosition());
                }
            }
        }
        else if(isGrabbing){
            if(isAdjustingShoulder && System.currentTimeMillis() - shoulderAdjustingStartTime < shoulderAdjustingWaitTime) {
                //do nothing
            }
            else if(slideMotorLeft.getCurrentPosition() < 0) {
                slideMotorLeft.setPower(0.7);
                slideMotorRight.setPower(0.7);
                finger.setPosition(0);
            }
            else {
                slideMotorLeft.setPower(0);
                slideMotorRight.setPower(0);

                wrist.setPosition(0.45);
                shoulder.setPosition(0.05);
                finger.setPosition(1);
                flipper.setPosition(0);
                isGrabbing = false;
                isAdjustingShoulder = false;
            }
        }
        else if (!touchBack.isPressed() && !isGrabbing && !isGrabbed && slideMotorRight.getCurrentPosition() <-200) {
            isGrabbed = true;
        }

        if (isPlacingStone || gamepad2.left_stick_button) //TODO
            placeStone();

        telemetry.update();
    }

    public void moveArm(){
//        if(gamepad2.right_stick_x > 0.5){
//            if(wrist.getPosition() < 1){
//                wrist.setPosition(wrist.getPosition() + 0.05);
//            }
//        }
//        else if(gamepad2.right_stick_x < -0.5){
//            if(wrist.getPosition() > 0){
//                wrist.setPosition((wrist.getPosition() - 0.05));
//            }
//        }
//
//        if(gamepad2.right_bumper)
//        {
//            finger.setPosition(0);
//        }
//        else if(gamepad2.right_trigger > 0.2)
//        {
//            finger.setPosition(1);
//        }
        if(gamepad2.left_stick_y < -0.5 || gamepad2.left_stick_y > 0.5) {
            isTuckStart = false;
        }

        if(gamepad2.left_stick_y > 0.1) {// The joystick is pointing down
            if (slideMotorRight.getCurrentPosition() >=0 || slideMotorLeft.getCurrentPosition() >= 0) {
                //The slide has reached bottom
                slideMotorLeft.setPower(0);
                slideMotorRight.setPower(0);
            }
            else {
                slideMotorLeft.setPower(gamepad2.left_stick_y / 1.2); //reduce the downward speed to reduce the slack
                slideMotorRight.setPower(gamepad2.left_stick_y / 1.2);
            }
        }
        else if(gamepad2.left_stick_y < -0.1){ //joystick is pointing up
            if (slideMotorRight.getCurrentPosition() <= -900 || slideMotorLeft.getCurrentPosition() <= -900) {
                //over the high limit
                slideMotorLeft.setPower(0);
                slideMotorRight.setPower(0);
            }
            else {
                if ((slideMotorLeft.getCurrentPosition() - slideMotorRight.getCurrentPosition()) < -20){
                    slideMotorLeft.setPower(gamepad2.left_stick_y / 5);
                    slideMotorRight.setPower(gamepad2.left_stick_y);
                }
                else if ((slideMotorRight.getCurrentPosition() - slideMotorLeft.getCurrentPosition()) < -20){
                    slideMotorRight.setPower(gamepad2.left_stick_y / 5);
                    slideMotorLeft.setPower(gamepad2.left_stick_y);
                }
                else {
                    slideMotorLeft.setPower(gamepad2.left_stick_y);
                    slideMotorRight.setPower(gamepad2.left_stick_y);
                }
            }
        }
        else if(!isIntaking && !isTuckStart && !isGrabbing){
            if(slideMotorLeft.getCurrentPosition() < -200){
                slideMotorLeft.setPower(-0.05);
                slideMotorRight.setPower(-0.05);
            }
            else{
                slideMotorLeft.setPower(0);
                slideMotorRight.setPower(0);
            }
        }

        if (slideMotorLeft.getCurrentPosition()<-250 && slideMotorRight.getCurrentPosition() <-250) {
            if(gamepad2.right_stick_x > 0.5){
                if(wrist.getPosition() < 1){
                    wrist.setPosition(wrist.getPosition() + 0.05);
                }
            }
            else if(gamepad2.right_stick_x < -0.5){
                if(wrist.getPosition() > 0){
                    wrist.setPosition((wrist.getPosition() - 0.05));
                }
            }
//            if(gamepad2.right_stick_y < -0.3){
//                if(shoulder.getPosition() < 1){
//                    shoulder.setPosition(shoulder.getPosition() + 0.05);
//                }
//            }
//            else if(gamepad2.right_stick_y > 0.3){
//                if(shoulder.getPosition() > 0){
//                    shoulder.setPosition((shoulder.getPosition() - 0.05));
//                }
//            }
        }

        //telemetry.addData("Slides: ", String.format("left: %5d, right: %5d", slideMotorLeft.getCurrentPosition(), slideMotorRight.getCurrentPosition()));
    }

    public void grabStone(){
        if (gamepad2.right_trigger > 0.2) {
//            wrist.setPosition(0.05);
            finger.setPosition(1);
        }
    }

    public void stackStone() {
        if (gamepad2.right_bumper) {
            finger.setPosition(0);
        }
    }

    public void placeStone() {
        int toleranceHeight = 20;

        if (!isPlacingStone) {
            if (shoulder.getPosition() >= 0.5 && shoulder.getPosition() < 0.7) { // 0.6
                currentHeight = Math.abs(slideMotorLeft.getCurrentPosition()) - 85 - toleranceHeight;
                currentLevel = (int) Math.floor(currentHeight / 130);
                gotoHeight = currentLevel * 130 + 85;
                isPlacingStone = true;
            } else if (shoulder.getPosition() >= 0.7 && shoulder.getPosition() < 0.9) { // 0.8
                currentHeight = Math.abs(slideMotorLeft.getCurrentPosition()) - toleranceHeight;
                currentLevel = (int) (Math.floor(currentHeight / 130)) + 1;
                gotoHeight = currentLevel * 130;
                isPlacingStone = true;
            } else if (shoulder.getPosition() >= 0.9) { // 1
                currentHeight = Math.abs(slideMotorLeft.getCurrentPosition()) - toleranceHeight;
                currentLevel = (int) (Math.floor(currentHeight / 130)) + 2;
                gotoHeight = currentLevel * 130;
                isPlacingStone = true;
            }

            gotoHeight *= -1;
        } else {
            if (slideMotorLeft.getCurrentPosition() < gotoHeight) {
                if (slideMotorLeft.getCurrentPosition() < -200) {
                    slideMotorLeft.setPower(0.8);
                    slideMotorRight.setPower(0.8);
                } else if (slideMotorLeft.getCurrentPosition() < -50) {
                    slideMotorLeft.setPower(0.30);
                    slideMotorRight.setPower(0.30);
                } else if (slideMotorLeft.getCurrentPosition() < -25) {
                    slideMotorLeft.setPower(0.25);
                    slideMotorRight.setPower(0.25);
                } else {
                    slideMotorLeft.setPower(0.1);
                    slideMotorRight.setPower(0.1);
                }
            } else {
                slideMotorLeft.setPower(0);
                slideMotorRight.setPower(0);
                isPlacingStone = false;
            }
        }
    }

    public void tuckArm() {
        //telemetry.addData("tuckArm: ", String.format("isTuckStart: %5b, tuckStartTime: %5d", isTuckStart, tuckStartTime));
        if (!isTuckStart) {
            tuckStartTime = System.currentTimeMillis();
            isTuckStart = true;
            shoulder.setPosition(0);

            if (slideMotorRight.getCurrentPosition() > -300)
                tuckWaitTime = 3000;
            else
                tuckWaitTime = 2000;

            if (shoulder.getPosition() >= 0.85) {
                tuckWaitTime += 1500;
            }  else if (shoulder.getPosition() >= 0.65) {
                tuckWaitTime += 1000;
            }
        } else if (isTuckStart) {

            if ((System.currentTimeMillis() - tuckStartTime) > tuckWaitTime) {
                if (slideMotorLeft.getCurrentPosition() < -200) {
                    slideMotorLeft.setPower(0.85);
                    slideMotorRight.setPower(0.85);
                } else if (slideMotorLeft.getCurrentPosition() < -100) {
                    slideMotorLeft.setPower(0.25);
                    slideMotorRight.setPower(0.25);
                } else if (slideMotorLeft.getCurrentPosition() < -50) {
                    slideMotorLeft.setPower(0.20);
                    slideMotorRight.setPower(0.20);
                } else {
                    slideMotorLeft.setPower(0);
                    slideMotorRight.setPower(0);
                    isTuckStart = false;
                }
            }
        }
    }
}