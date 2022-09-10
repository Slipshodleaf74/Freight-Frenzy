package org.firstinspires.ftc.teamcode.FirstTry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "BC archive")
public class BCarchive extends LinearOpMode{
    //auto arm and extender position variables
    int highArm = 2050;
    int midArm = 2250;
    int lowArm = 2450;

    int highExt = -900;
    int midExt = -700;
    int lowExt = -500;

    AutoHardware robot = new AutoHardware();
    //AutoHardware robot = new AutoHardware();

    //power variables
    double driveSpeed = 0.6;
    double strafeSpeed = 0.4;
    double rotateSpeed = 0.5;
    double armSpeed = 0.2;
    double carouselSpeed = 0.5;
    double extendSpeed = 0.8;

    double grabPos = 0.8;

    //target level variable
    int level = 0; //1 = low, 2 = middle, 3 = high

    //variables for camera operation
    //row, column (y, x)
    int[] subMatZero = {110,110};
    int[] subMatOne = {110,355};
    int[] subMatTwo = {110,600};

    int subMatWidth = 30;
    int subMatHeight = 40;

    static final int STREAM_WIDTH = 640; // modify for your camera
    static final int STREAM_HEIGHT = 360; // modify for your camera
    OpenCvWebcam webcam;
    FreightFrenzyPipeline pipeline;

    int position = 0;
    //end of variables for camera

    public void telemetry(){
        telemetry.addData("wheel position:", Integer.toString(robot.frontRight.getCurrentPosition()));
        telemetry.addData("arm position:", Integer.toString(robot.arm.getCurrentPosition()));
        telemetry.addData("extender position", Integer.toString(robot.extender.getCurrentPosition()));
        telemetry.addData("angle:", Double.toString(robot.getAngle()));
        telemetry.update();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        //initialize camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = null;
        webcamName = hardwareMap.get(WebcamName.class, "webcam"); // put your camera's name here
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        pipeline = new FreightFrenzyPipeline(subMatWidth,subMatHeight,subMatZero,subMatOne,subMatTwo);
        webcam.setPipeline(pipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(STREAM_WIDTH, STREAM_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Failed","");
                telemetry.update();
            }
        }); //done initializing camera

        //telemetry communication
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

        telemetry.addData("Mode", "calibrated");
        telemetry.update();

        while (!isStarted()){
            position = pipeline.position;
            telemetry.addData("Position", position);
            telemetry.update();
        }

        //no more need for camera
        webcam.stopStreaming();

        //redundant
        //waitForStart();
        //calculate target based on position of marker

        if(position == 0){
            //left position, lowest level
            level = 1;
        }
        else if(position == 1){
            //middle position, middle level
            level = 2;
        }
        else if (position == 2){
            //right position, highest level
            level = 3;
        }

        //movements for auto

        //grab block
        robot.grab(grabPos);

        sleep(500);

        //move arm up to avoid friction with block on ground
        robot.moveArm(4,armSpeed,this, true);

        sleep(500);

        //strafe
        robot.strafeToPosition(18,strafeSpeed,this, true);

        sleep(500);

        //drive forward
        robot.driveAndStop(-6,driveSpeed,this, true);

        sleep(500);

        //rotate
        robot.rotate(40,rotateSpeed,this);

        sleep(500);

        //move arm
        robot.moveArm(level,armSpeed,this, true);

        sleep(500);

        //extend arm
        robot.moveExtender(level,extendSpeed);

        //release
        robot.grab(0);

        sleep(200);

        robot.grab(grabPos);

        //retract arm
        robot.moveExtender(0, extendSpeed);

        //move arm back
        robot.moveArm(0, armSpeed, this, true);

        sleep(200);

        //rotate to align with wall (turn 90 overall)
        robot.rotate(45,rotateSpeed,this);

        sleep(500);

        //drive into wall
        robot.strafeToPosition(50,strafeSpeed,this, true);

        //drive into carousel
        robot.driveToPosition(10,driveSpeed,this, true);

        //spin carousel
        robot.spin(carouselSpeed,false);

        sleep(2500);

        robot.spin(0,false);

        //drive backwards to park
        robot.driveAndStop(-10,driveSpeed,this, true);

        //strafe into wall to park
        robot.strafeToPosition(5,strafeSpeed,this, true);

        //stop
        robot.stop();
    }
}
