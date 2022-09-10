package org.firstinspires.ftc.teamcode.FirstTry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Box Test", group = "TeleOp")
public class BoxTest extends LinearOpMode{
    public void runOpMode() throws InterruptedException{
        CRServo intake1 = hardwareMap.get(CRServo.class, "Intake1");
        CRServo intake2 = hardwareMap.get(CRServo.class, "Intake2");

        waitForStart();

        while(opModeIsActive()) {
            if (gamepad1.right_bumper) {
                intake1.setPower(1.0);
                intake2.setPower(-1.0);
            }
            else if (gamepad1.left_bumper) {
                intake1.setPower(-1.0);
                intake2.setPower(1.0);
            }
            else{
                intake1.setPower(0.0);
                intake2.setPower(0.0);
            }
        }
    }
}
