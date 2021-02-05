package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@Autonomous
public class Auto_SB extends LinearOpMode {

    // Declare OpMode members.
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
        initializeIMU();

        shooter.setDirection(DcMotor.Direction.REVERSE);
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // This is were the autonomous sequence begins
        // Move forward, rev up the up the motor
        driveWithoutTime(.6);
        shooter.setPower(.38);
        sleep(1670);
        stopAllMotors();



        belt.setPosition(1);
        sleep(13000);
        shooter.setPower(0);
        belt.setPosition(0.5);
        sleep(100);

        ///////////////////////////////////////Turn Right
        turnToAngle(-125,.4);
        stopAllMotors();
        driveBackward(.4,450);
        stopAllMotors();
        sleep(100);

        ///////////////////////////////////////Clamp Arm
        arm.setPosition(-1);

        ///////////////////////////////////////Arm Down
        wobble.setPower(.5);

        sleep(3600);

        ///////////////////////////////////////Arm Release
        arm.setPosition(1);

        ///////////////////////////////////////Arm Up
        wobble.setPower(-1);

        sleep(1500);

        stopAllMotors();
        stop();

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
