package org.firstinspires.ftc.teamcode.FirstTry;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "NeilBot", group = "FirstTry")
public class NeilBot extends OpMode {

    DcMotor frontleft, backleft;
    DcMotor frontright, backright;

    @Override
    public void init(){
        frontleft = hardwareMap.get(DcMotor.class,"FrontLeft");
        backleft = hardwareMap.get(DcMotor.class,"BackLeft");
        frontright = hardwareMap.get(DcMotor.class,"FrontRight");
        backright = hardwareMap.get(DcMotor.class,"BackRight");

    }

    @Override
    public void loop(){

        frontleft.setPower(Range.clip(gamepad1.left_stick_y, -1, 1));
        backleft.setPower(Range.clip(gamepad1.left_stick_y, -1, 1));
        frontright.setPower(Range.clip(gamepad1.right_stick_y, -1, 1));
        backright.setPower(Range.clip(gamepad1.right_stick_y, -1, 1));

        frontleft.setPower(Range.clip(gamepad1.left_stick_x, -1, 1));
        backleft.setPower(Range.clip(gamepad1.left_stick_x, 1, -1));
        frontright.setPower(Range.clip(gamepad1.right_stick_x, -1, 1));
        backright.setPower(Range.clip(gamepad1.right_stick_x, 1, -1));

    }
}
