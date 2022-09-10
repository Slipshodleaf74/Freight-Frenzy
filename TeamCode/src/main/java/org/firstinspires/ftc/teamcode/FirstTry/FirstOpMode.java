package org.firstinspires.ftc.teamcode.FirstTry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "FirstOpMode", group = "FirstTry")
public class FirstOpMode extends OpMode {

    DcMotor FrontLeft, BackLeft;
    DcMotor FrontRight, BackRight;
    DcMotor Arm, Carousel;
    Servo Grab;

    @Override
    public void init(){
        FrontLeft = hardwareMap.get(DcMotor.class,"FrontLeft");
        BackLeft = hardwareMap.get(DcMotor.class,"BackLeft");
        FrontRight = hardwareMap.get(DcMotor.class,"FrontRight");
        BackRight = hardwareMap.get(DcMotor.class,"BackRight");
        Arm = hardwareMap.get(DcMotor.class,"Arm");
        Carousel = hardwareMap.get(DcMotor.class,"Carousel");
        Grab = hardwareMap.get(Servo.class, "Grab");

    }

    @Override
    public void loop(){

        FrontLeft.setPower(Range.clip(gamepad1.left_stick_y, -1, 1));
        BackLeft.setPower(Range.clip(gamepad1.left_stick_y, -1, 1));
        FrontRight.setPower(Range.clip(gamepad1.right_stick_y, -1, 1));
        BackRight.setPower(Range.clip(gamepad1.right_stick_y, -1, 1));
        Arm.setPower(Range.clip(gamepad2.right_stick_y, -.4, .4));
        Carousel.setPower(Range.clip(gamepad2.left_stick_y, -.5, .5));

        if(gamepad2.right_bumper){
            Grab.setPosition(.4);
        }
        if(gamepad2.left_bumper){
            Grab.setPosition(0);
        }
    }
}
