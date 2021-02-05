package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

@Autonomous
public class Full_Auto_SB extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";

    //Insert Vuforia License Key
    private static final String VUFORIA_KEY =
            "AYoimHf/////AAABmaDxgPkMWkDJv1W3sWAQ2lts4HoYEPS0O19kiA6/0SBVMumCZAHUh+zHVCSrhF6qSvlhjoJpgn4swy0j1vJ1V57siep2bSnVumuGGF49uVVkLIY59Ks8GORIbwdSLmskJJV46Eiz+4Nu09rq+UFGmm/MdZEe7IaqgtqbzySrNIBraXjm1ovgPWYoXmH8i/n/SVRCkKStFU+fkUSQ7OPIjAkZEWiaW972Jw8Ywe1VpkQWAVihARMqw4YP48XBn5Ag2O6p12JyzfA/jZVi5V20RljAkFgYa3jQMxUi+2OkXjouUhWWjLtFMxjsQAt2GlyoPy9kDNzG/xnW0fx5BJv5bLBk4aClzxbRyf/eHWfhP8pe";

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backRightDrive = null;
    private DcMotor leftIntake = null;
    private DcMotor rightIntake = null;
    private DcMotor shooter = null;
    private DcMotor wobble = null;
    private Servo belt = null;
    private Servo arm = null;

    private BNO055IMU gyro = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "left_drive_front");
        backLeftDrive  = hardwareMap.get(DcMotor.class, "left_drive_back");
        frontRightDrive = hardwareMap.get(DcMotor.class, "right_drive_front");
        backRightDrive = hardwareMap.get(DcMotor.class, "right_drive_back");

        leftIntake = hardwareMap.get(DcMotor.class, "left_intake");
        rightIntake = hardwareMap.get(DcMotor.class, "right_intake");
        shooter = hardwareMap.get(DcMotor.class, "shooter");
        wobble = hardwareMap.get(DcMotor.class, "wobble");
        belt = hardwareMap.get(Servo.class, "belt");
        arm = hardwareMap.get(Servo.class, "arm");
        gyro = hardwareMap.get(BNO055IMU.class, "imu");

        shooter.setDirection(DcMotor.Direction.REVERSE);
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        initializeIMU();

        initVuforia();
        initTfod();

        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(2.5, 16.0/9.0);
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());

                        if(updatedRecognitions.size() == 0){
                            // empty list.  no objects recognized.
                            telemetry.addData("TFOD", "No items detected.");
                            telemetry.addData("Target Zone", "A");
                            //This begins the autonomous sequence for target zone A
                            driveAndShoot();

                            //This ends the previous autonomous sequence
                        }

                        // step through the list of recognitions and display boundary info.
                        int i = 0;
                        for (Recognition recognition : updatedRecognitions) {
                            telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                            telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                    recognition.getLeft(), recognition.getTop());
                            telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                    recognition.getRight(), recognition.getBottom());

                            // check label to see which target zone to go after.
                            if (recognition.getLabel().equals("Single")) {
                                telemetry.addData("Target Zone", "B");
                                //This begins the autonomous sequence for target zone B
                                driveAndShoot();

                                //This ends the previous autonomous sequence
                            } else if (recognition.getLabel().equals("Quad")) {
                                telemetry.addData("Target Zone", "C");
                                //This begins the autonomous sequence for target zone C
                                driveAndShoot();

                                //This ends the previous autonomous sequence
                            } else {
                                telemetry.addData("Target Zone", "UNKNOWN");
                                //This begins the autonomous sequence for an unknown case

                                //This ends the previous autonomous sequence
                            }
                        }
                        telemetry.update();
                    }
                }
            }
        }

        if (tfod != null) {
            tfod.shutdown();
        }
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    public void driveForward(double speed, int time){
        frontLeftDrive.setPower(speed);
        frontRightDrive.setPower(speed);
        backLeftDrive.setPower(speed);
        backRightDrive.setPower(speed);

        sleep(time);
    }

    public void driveBackward(double speed, int time){
        frontLeftDrive.setPower(-speed);
        frontRightDrive.setPower(-speed);
        backLeftDrive.setPower(-speed);
        backRightDrive.setPower(-speed);

        sleep(time);
    }

    public void stopAllMotors(){
        frontLeftDrive.setPower(0);
        backLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backRightDrive.setPower(0);

        sleep(100);
    }

    public void turnToAngle (int angle, double speed){
        Orientation angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double yaw = angles.firstAngle;
        double targetAngle = yaw - angle;

        if(angle > 0){      //if angle > 0, turn right
            frontLeftDrive.setPower(speed);
            backLeftDrive.setPower(speed);

            frontRightDrive.setPower(-speed);
            backRightDrive.setPower(-speed);

            while(yaw > targetAngle){
                angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                yaw = angles.firstAngle;
                telemetry.addData("Target Angle", targetAngle);
                telemetry.addData("Current Angle",yaw);
                telemetry.update();
                sleep(10);
            }

            stopAllMotors();
        }
        else if(angle < 0){     //if angle < 0, turn left
            frontLeftDrive.setPower(-speed);
            backLeftDrive.setPower(-speed);

            frontRightDrive.setPower(speed);
            backRightDrive.setPower(speed);

            while(yaw < targetAngle){
                angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                yaw = angles.firstAngle;
                telemetry.addData("Target Angle", targetAngle);
                telemetry.addData("Current Angle",yaw);
                telemetry.update();
                sleep(10);
                yaw = angles.firstAngle;
            }

            stopAllMotors();
        }
    }

    public void driveWithoutTime(double speed){
        frontLeftDrive.setPower(speed);
        frontRightDrive.setPower(speed);
        backLeftDrive.setPower(speed);
        backRightDrive.setPower(speed);
    }

    public void driveAndShoot(){
        driveWithoutTime(.6);
        shooter.setPower(.4);
        sleep(1600);
        stopAllMotors();
        turnToAngle(-10,.3);
        belt.setPosition(1);
        sleep(13000);
        shooter.setPower(0);
        belt.setPosition(.5);
    }

    public void dropWobble(){
        arm.setPosition(-1);
        wobble.setPower(.5);
        sleep(3600);
        arm.setPosition(1);
        wobble.setPower(-1);
        sleep(1500);
        arm.setPosition(.5);
        wobble.setPower(0);
    }

    public void initializeIMU(){
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        gyro.initialize(parameters);
    }
}
