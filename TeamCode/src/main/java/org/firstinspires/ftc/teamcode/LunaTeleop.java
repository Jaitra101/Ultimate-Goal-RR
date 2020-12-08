package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Luna Teleop")
public class LunaTeleop extends OpMode {
    DcMotor rightFront;
    DcMotor leftFront;
    DcMotor leftRear;
    DcMotor rightRear;
    DcMotor intake;
    DcMotor flywheel;
    DcMotor roller;
    Servo indexer;
    Servo grabber;
    DcMotor arm;

    // above initializes all the aspects we need to make our robot function
    @Override
    public void init() {

        // defining all the hardware
        leftFront = hardwareMap.dcMotor.get("lf");
        leftRear = hardwareMap.dcMotor.get("lr");
        rightRear = hardwareMap.dcMotor.get("rr");
        rightFront = hardwareMap.dcMotor.get("rf");
        intake = hardwareMap.dcMotor.get("intake");
        flywheel = hardwareMap.dcMotor.get("flywheel");
        roller = hardwareMap.dcMotor.get("roller");
        indexer = hardwareMap.servo.get("indexer");
        grabber = hardwareMap.servo.get("grabber");
        arm = hardwareMap.dcMotor.get("arm");
        indexer.setPosition(0.90);
        grabber.setPosition(0.75);

        //this puts the motors in reverse
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        roller.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    @Override
    public void loop() {
        float x1 = -gamepad1.right_stick_x;
        float y1 = gamepad1.left_stick_y;
        float r1 = gamepad1.right_trigger;
        float r2 = gamepad1.left_trigger;
        float i = gamepad2.left_stick_y;
        float r = gamepad2.right_stick_y;
        boolean f1 = gamepad2.right_bumper;
        boolean f2 = gamepad2.left_bumper;
        boolean index = gamepad2.x;
        boolean armUp = gamepad2.dpad_up;
        boolean armDown = gamepad2.dpad_down;
        boolean release = gamepad2.dpad_left;
        boolean grasp = gamepad2.dpad_right;
        telemetry.addData("Servo Position: ", grabber.getPosition());
        telemetry.update();

        // Reset variables
        float leftFrontPower = 0;
        float leftBackPower = 0;
        float rightFrontPower = 0;
        float rightBackPower = 0;

        // Handle regular movement
        leftFrontPower += y1;
        leftBackPower += y1;
        rightFrontPower += y1;
        rightBackPower += y1;

        // Handle strafing movement
        leftFrontPower += x1;
        leftBackPower -= x1;
        rightFrontPower -= x1;
        rightBackPower += x1;

        // Handle clockwise turning movement
        leftFrontPower -= r1*0.75;
        leftBackPower -= r1*0.75;
        rightFrontPower += r1*0.75;
        rightBackPower += r1*0.75;

        // Handle counterclockwise turning movement
        leftFrontPower += r2*0.75;
        leftBackPower += r2*0.75;
        rightFrontPower -= r2*0.75;
        rightBackPower -= r2*0.75;

        if (i < 0) {
            intake.setPower(1.00);
        }
        else {
            intake.setPower(0.00);
        }

        if (i > 0) {
            intake.setPower(-1.00);
        }
        else {
            intake.setPower(0.00);
        }
        if (r < 0) {
            roller.setPower(1.00);
        }
        else {
            roller.setPower(0.00);
        }

        if (r > 0) {
            roller.setPower(-1.00);
        }
        else {
            roller.setPower(0.00);
        }

        if (f1 == true) {
            flywheel.setPower(1.00);
        }

        if (f2 == true) {
            flywheel.setPower(0.00);
        }

        if (index == true) {
            indexer.setPosition(0.75);
        }
        else {
            indexer.setPosition(0.90);
        }
        if (grasp == true) {
            grabber.setPosition(0.75);
        }
        if (release == true) {
            grabber.setPosition(0.30);
        }
        if (armUp == true) {
            arm.setPower(0.60);
        }
        else {
            arm.setPower(0.00);
        }
        if (armDown == true) {
            arm.setPower(-0.75);
        }
        else {
            arm.setPower(0.00);
        }


        // Scale movement
        double max = Math.max(Math.abs(leftFrontPower), Math.max(Math.abs(leftBackPower),
                Math.max(Math.abs(rightFrontPower), Math.abs(rightBackPower))));

        if (max > 1) {
            leftFrontPower = (float) Range.scale(leftFrontPower, -max, max, -.30, .30);
            leftBackPower = (float) Range.scale(leftBackPower, -max, max, -.30, .30);
            rightFrontPower = (float) Range.scale(rightFrontPower, -max, max, -.30, .30);
            rightBackPower = (float) Range.scale(rightBackPower, -max, max, -.30, .30);
        }

        leftRear.setPower(-leftBackPower);
        leftFront.setPower(-leftFrontPower);
        rightFront.setPower(-rightFrontPower);
        rightRear.setPower(-rightBackPower);
    }
}