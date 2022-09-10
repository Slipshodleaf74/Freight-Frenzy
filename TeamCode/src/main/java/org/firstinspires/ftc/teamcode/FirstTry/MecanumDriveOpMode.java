package org.firstinspires.ftc.teamcode.FirstTry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "Mecanum Opmode", group = "TeleOp")
public class MecanumDriveOpMode extends LinearOpMode {

    Hardware6417 robot = new Hardware6417();

    enum Direction {
        FORWARD, BACKWARD, LEFT, RIGHT, RTLEFT, RTRIGHT;
    }

    public void runOpMode(){

        double leftVert, rightVert, leftHoriz, rightHoriz;

        leftVert = -gamepad1.left_stick_y;
        leftHoriz = gamepad1.left_stick_x;
        rightVert = -gamepad1.right_stick_y;
        rightHoriz = gamepad1.right_stick_x;

        if (Math.abs(leftVert) > 0.3 || Math.abs(rightVert) > 0.3 || Math.abs(leftHoriz) > 0.3 || Math.abs(rightHoriz) > 0.3) {
            robot.setDriveSpeeds(leftVert, rightVert, leftHoriz, rightHoriz);
        } else {
            robot.setDriveSpeeds(0, 0, 0, 0);
        }
    }
}