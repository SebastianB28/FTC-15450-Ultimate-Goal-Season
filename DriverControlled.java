package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;



@TeleOp
public class DriverControlled extends OpMode {
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


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

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


        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        leftIntake.setDirection(DcMotor.Direction.REVERSE);
        rightIntake.setDirection(DcMotor.Direction.FORWARD);
        shooter.setDirection(DcMotor.Direction.REVERSE);
        wobble.setDirection(DcMotor.Direction.FORWARD);
        //belt.setPosition(0);
        arm.setPosition(0);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry
        double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
        double rightX = gamepad1.right_stick_x;
        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;

        frontLeftDrive.setPower(v1);
        frontRightDrive.setPower(v2);
        backLeftDrive.setPower(v3);
        backRightDrive.setPower(v4);

        if(gamepad1.dpad_down){
            frontLeftDrive.setPower(-1);
            backRightDrive.setPower(-1);
            backLeftDrive.setPower(-1);
            frontRightDrive.setPower(-1);
        } else
            frontLeftDrive.setPower(0);
        backRightDrive.setPower(0);
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);


        /*WOBBLE*/
        if(gamepad1.left_bumper){wobble.setPower(-1);}
        else if(gamepad1.right_bumper){
            wobble.setPower(1);
            arm.setPosition(1);
        } else {
            wobble.setPower(0);
            arm.setPosition(0);
        }

        /*RING INTAKE*/
        double intakePower = 1;
        if(gamepad2.b){leftIntake.setPower(intakePower);} else leftIntake.setPower(0);
        if(gamepad2.b){rightIntake.setPower(intakePower);} else rightIntake.setPower(0);
        if(gamepad2.a){leftIntake.setPower(-1);} else leftIntake.setPower(0);
        if(gamepad2.a){rightIntake.setPower(-1);} else rightIntake.setPower(0);

        /*Shooter*/
//        double smallShootingPower = 0.30;
        double normalShootingPower = 0.5;
//        double largeShootingPower = 0.75;

        if(gamepad2.dpad_up){
            normalShootingPower=normalShootingPower+.05;
        }
        if(gamepad2.dpad_down){
            normalShootingPower=normalShootingPower-.05;
        }

        if(gamepad2.y && !gamepad2.x){
            shooter.setPower(normalShootingPower);
        } else if(gamepad2.x && !gamepad2.y) {
            shooter.setPower(.45);
        } else if(gamepad2.y && gamepad2.x){
            shooter.setPower(.6);
        } else {
            shooter.setPower(0);
        }
        //if(gamepad2.y && gamepad2.dpad_down){shooter.setPower(smallShootingPower)g;} else shooter.setPower(0);

        /*Belt*/
        if(gamepad2.left_bumper){
            belt.setPosition(1);
        } else if(gamepad2.right_bumper){
            belt.setPosition(-1);
        } else belt.setPosition(.5);




        telemetry.addData("speed", shooter.getPower());
        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", -gamepad1.left_stick_y, -gamepad1.left_stick_y, -gamepad1.right_stick_y, -gamepad1.right_stick_y);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
