package org.firstinspires.ftc.teamcode.FirstTry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Lucas BOX NO WRIST Opmode", group = "TeleOp")
public class LucasCodeBOXNOWRIST extends LinearOpMode{

    //initialize opmode members
    LucasHardwareBOXNOWRIST robot = new LucasHardwareBOXNOWRIST();
    private ElapsedTime runtime = new ElapsedTime();

    // calculate these using measurement
    // encoder position variables
    private int sharedPos = 2650;
    private int midPos = 1950;
    private int highPos = 2000;
    private int capPos = 2000;
    private int vertPos = 1400;
    private int capGrabPos = 2525;

    private int extendMaxPos = -800;
    private int extendCapPos = -800;
    private int extendMinPos = 0;
    private int extendFloorPos = -510;

    //initialize
    @Override
    public void runOpMode(){
        robot.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !robot.imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "calibrated!");
        telemetry.update();


        waitForStart();
        runtime.reset();

        //variables for controlling
        double forwardleft, forwardright, strafeleft, straferight, armMove, extenderMove;

        //power variables
        double correction = 0; //dunno abt this one, maybe useful for autonomoosssss
        double armSpeed = 0.5;
        double armEncoderSpeed = 0.5;
        double carouselSpeed1 = 0.5;
        double carouselSpeed2 = 1.0;
        double extenderSpeed = 1;
        double extendEncoderSpeed = 0.4;

        double sens = 0.15;

        boolean autoArm = false;
        boolean autoExtend = false;
        boolean floored = false;
        boolean maintainArm = false;
        boolean spinning = false;
        boolean spinningDir = false;

        boolean previousB1 = false;
        boolean previousA1 = false;

        boolean previousDpadDown2 = false;
        boolean previousLeftBumper1 = false;
        boolean previousRightBumper1 = false;
        boolean previousRightStickButton2 = false;

        int targetLevel = 0;

        //control loop
        while(opModeIsActive()){

            /*CONTROLS
            gamepad 1:
            Dpad - nudge wheels
            Left stick - left drive
            Right stick - right drive
            Bumpers - spin carousel
            a - pivot left from back
            b - pivot right from back
            y - cap pos auto move arm
            x - cap extend pos

            gamepad 2:
            Dpad up - reset encoder
            Dpad down - toggle floor extension
            Dpad horizontal - nudge arm (left -> up, right -> down)
            Left stick - extend arm
            Right stick - move arm
            Face buttons - auto move arm
                y - vert
                x - shared
                b - high pos
                a - home
            Left bumper - take in
            right bumper - take out
            */

            //input gamepad joystick variables for control
            forwardleft = Range.clip(gamepad1.left_stick_y,-1,1);
            strafeleft = -1*Range.clip(gamepad1.left_stick_x,-1,1);
            forwardright = Range.clip(gamepad1.right_stick_y,-1,1);
            straferight = -1*Range.clip(gamepad1.right_stick_x, -1, 1);

            armMove = Range.clip(gamepad2.right_stick_y,-1,1);
            extenderMove = Range.clip(gamepad2.left_stick_y, -1, 1);


            //driving
            {
                //move wheels
                if (Math.abs(forwardleft) > sens || Math.abs(forwardright) > sens || Math.abs(strafeleft) > sens || Math.abs(straferight) > sens) {
                    robot.setDriveSpeeds(forwardleft, forwardright, strafeleft, straferight, correction);
                } else if(!(gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_left || gamepad1.dpad_right)){
                    robot.stop();
                }

                //nudge wheels
                if (gamepad1.dpad_up) {
                    robot.setDriveSpeeds(-.3, -.3, 0, 0, correction);
                }
                if (gamepad1.dpad_down) {
                    robot.setDriveSpeeds(.3, .3, 0, 0, correction);
                }
                if (gamepad1.dpad_left) {
                    robot.setDriveSpeeds(0, 0, .3, .3, correction);
                }
                if (gamepad1.dpad_right) {
                    robot.setDriveSpeeds(0, 0, -.3, -.3, correction);
                }

                //pivot
                if(gamepad1.b && !previousB1){
                    //rotate right
                    robot.rotate(30, 0.5, this);

                    //wait for rotation, press other button to cancel
                    while(opModeIsActive() && Math.abs(robot.getAngle()) < 30){
                        if(gamepad1.a){
                            previousA1 = true;
                            break;
                        }
                    }

                    robot.backLeft.setPower(-robot.backLeft.getPower());
                    robot.backRight.setPower(-robot.backLeft.getPower());

                    sleep(50);

                    robot.stop();

                    // wait for rotation to stop.
                    //instance.sleep(1000);
                }
                previousB1 = gamepad1.b;

                if(gamepad1.a && !previousA1){
                    //rotate left
                    robot.rotate(-30, 0.5, this);

                    //wait for rotation, press other button to cancel
                    while(opModeIsActive() && Math.abs(robot.getAngle()) < 30){
                        if(gamepad1.b){
                            previousB1 = true;
                            break;
                        }
                    }

                    robot.backLeft.setPower(-robot.backLeft.getPower());
                    robot.backRight.setPower(-robot.backLeft.getPower());

                    sleep(50);

                    robot.stop();

                    // wait for rotation to stop.
                    //instance.sleep(1000);
                }
                previousA1 = gamepad1.a;
            }

            //manual arm control
            {
                //move arm
                if (Math.abs(armMove) > sens) {
                    robot.moveArm(-armMove * armSpeed);
                    autoArm = false;
                }
                else if (gamepad2.dpad_left){
                    //nudge up
                    robot.moveArm(armSpeed/4.0);
                    autoArm = false;
                }
                else if (gamepad2.dpad_right){
                    //nudge down
                    robot.moveArm(-armSpeed/4.0);
                    autoArm = false;
                }
                else if (!autoArm) {
                    if(maintainArm){
                        //experimental funct
                        robot.maintainArm();
                    }
                    else{
                        robot.moveArm(0);
                    }
                }

                //extend arm
                if (Math.abs(extenderMove) > sens) {
                    floored = false;
                    autoExtend = false;
                    robot.moveExtender(extenderMove);
                } else if (!autoExtend) {
                    robot.moveExtender(0);
                }
            }

            //automatic arm control
            {
                //reset encoder of arm and extender
                if (gamepad2.dpad_up) {
                    robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                    robot.extender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.extender.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }

                //set arm target level and set mode to move
                if (gamepad2.y) {
                    //vert
                    targetLevel = highPos;
                    autoArm = true;
                }
                if (gamepad2.x) {
                    //shared
                    targetLevel = sharedPos;
                    autoArm = true;
                }
                if (gamepad2.b) {
                    //high pos
                    targetLevel = vertPos;
                    autoArm = true;
                }
                if (gamepad2.a) {
                    //home
                    targetLevel = 0;
                    autoArm = true;
                }

                //move arm to cap level automatically
                if (gamepad1.y) {
                    //set it to automatically move
                    targetLevel = capPos;
                    autoArm = true;
                }


                //move arm if automatically moving, maintenance in here
                if (autoArm) {
                    robot.encoderMoveArm(targetLevel, extendEncoderSpeed);

                    if(!robot.arm.isBusy()){
                        autoArm = false;
                        robot.arm.setPower(0);
                    }
                }

                //extend arm to floor for intake "floored"
                if(gamepad2.dpad_down && !previousDpadDown2){
                    autoExtend = true;

                    if(floored){
                        floored = false;

                        robot.encoderExtend(0,extenderSpeed,false);
                    }
                    else{
                        floored = true;

                        robot.encoderExtend(extendFloorPos,extenderSpeed,false);
                    }
                }
                previousDpadDown2 = gamepad2.dpad_down;

                //extend to cap
                if(gamepad1.x){
                    autoExtend = true;
                    floored = false;
                    robot.encoderExtend(extendCapPos,extenderSpeed,false);
                }

                //stop arm from moving once it has reached its position
                if(autoExtend){
                    if(!robot.extender.isBusy()){
                        robot.extender.setPower(0);
                    }
                }

                //toggle arm maintenance
                if(gamepad2.right_stick_button && !previousRightStickButton2){
                    maintainArm = !maintainArm;
                }
                previousRightStickButton2 = gamepad2.right_stick_button;
            }

            //misc functions, intake, carousel
            {
                //control intake
                if (gamepad2.left_bumper){
                    robot.intake(true,true);
                }
                else if (gamepad2.right_bumper){
                    robot.intake(true,false);
                }
                else{
                    robot.intake(false,false);
                }

                //Spin carousel
                if (gamepad1.right_bumper && !previousRightBumper1) {
                    runtime.reset();
                    spinning = !spinning;
                    spinningDir = true;
                }
                else if (gamepad1.left_bumper && !previousLeftBumper1) {
                    runtime.reset();
                    spinning = !spinning;
                    spinningDir = false;
                }
                previousRightBumper1 = gamepad1.right_bumper;
                previousLeftBumper1 = gamepad1.left_bumper;

                if(spinning){
                    if(runtime.milliseconds() < 1000){
                        if(spinningDir){
                            robot.spin(carouselSpeed1);
                        }
                        else{
                            robot.spin(-carouselSpeed1);
                        }
                    }
                    else if(runtime.milliseconds() < 1500){
                        if(spinningDir){
                            robot.spin(carouselSpeed2);
                        }
                        else{
                            robot.spin(-carouselSpeed2);
                        }
                    }
                    else{
                        spinning = false;
                        robot.carousel.setPower(0);
                    }
                }
                else{
                    robot.spin(0);
                }


                //telemetry
                telemetry.addData("Maintain?", maintainArm);
                telemetry.addData("Arm pos", robot.arm.getCurrentPosition());
                telemetry.addData("Ext pos", robot.extender.getCurrentPosition());
                telemetry.addData("Left trigger 1", Float.toString(gamepad1.left_trigger));
                telemetry.addData("Right trigger 1", Float.toString(gamepad1.right_trigger));
                telemetry.addData("Left trigger 2", Float.toString(gamepad2.left_trigger));
                telemetry.addData("Right trigger 2", Float.toString(gamepad2.right_trigger));
                telemetry.update();
            }

            //prev buttons
        }
    }
}
