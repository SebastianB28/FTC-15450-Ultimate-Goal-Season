/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Basic: Iterative OpMode", group="Iterative Opmode")
//@Disabled
public class DriverControlled extends OpMode
{
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
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);//
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
        double leftPowerFront;
        double rightPowerFront;
        double leftPowerBack;
        double rightPowerBack;

        double leftPowerFrontS;
        double rightPowerFrontS;
        double leftPowerBackS;
        double rightPowerBackS;

        boolean intakeLeft;
        boolean intakeRight;
        boolean shooting;
        boolean wob;
        boolean arms;
        boolean beltMove;

        /////////////////////////////////////////////////////////////////////////////////
       /*
       double wobSpeed = .20;
       double inSpeedLeft = .20;
       double inSpeedRight = .20;
       double shootSpeed = .50;
       */
        ////////////////////////////////////////////////////////////////////////////////

        // Choose to drive using either Tank Mode, or POV Mode
        // Comment out the method that's not used.  The default below is POV.

        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.
        //double drive = -gamepad1.left_stick_y;
        //double turn  =  gamepad1.right_stick_x;
        //leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
        //rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;

        // Tank Mode uses one stick to control each wheel.
        // - This requires no math, but it is hard to drive forward slowly and keep straight.

       /*
       leftPowerFront = 0.50;
       rightPowerFront = 0.50;
       leftPowerBack = 0.50;
       rightPowerBack = 0.50;
       */


        leftPowerFront  = -gamepad1.left_stick_y ;
        leftPowerBack  = -gamepad1.left_stick_y ;
        rightPowerBack = -gamepad1.right_stick_y ;
        rightPowerFront = -gamepad1.right_stick_y ;


//        shooting = gamepad2.dpad_up;

//        wob = gamepad1.left_bumper;


        // Send calculated power to wheels
        /*WHEELS*/

       /*double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
       double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
       double rightX = gamepad1.right_stick_x;
       final double v1 = r * Math.sin(robotAngle) - rightX;
       final double v2 = r * Math.cos(robotAngle) + rightX;
       final double v3 = r * Math.cos(robotAngle) - rightX;
       final double v4 = r * Math.sin(robotAngle) + rightX;
       frontLeftDrive.setPower(-v1);
       frontRightDrive.setPower(-v2);
       backLeftDrive.setPower(-v3);
       backRightDrive.setPower(-v4);
*/
        double wheelPower = 0.80;
        frontLeftDrive.setPower(leftPowerFront*wheelPower);
        backLeftDrive.setPower(leftPowerBack*wheelPower);
        backRightDrive.setPower(rightPowerBack*wheelPower);
        frontRightDrive.setPower(rightPowerFront*wheelPower);

        /*STRAFE*/

        if(gamepad1.dpad_right){
            frontLeftDrive.setPower(1);
            backRightDrive.setPower(1);
            backLeftDrive.setPower(-1);
            frontRightDrive.setPower(-1);
        } else
            frontLeftDrive.setPower(0);
        backRightDrive.setPower(0);
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);

        if(gamepad1.dpad_left){
            frontLeftDrive.setPower(-1);
            backRightDrive.setPower(-1);
            backLeftDrive.setPower(1);
            frontRightDrive.setPower(1);
        } else
            frontLeftDrive.setPower(0);
        backRightDrive.setPower(0);
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);

        if(gamepad1.dpad_up){
            frontLeftDrive.setPower(1);
            backRightDrive.setPower(1);
            backLeftDrive.setPower(1);
            frontRightDrive.setPower(1);
        } else
            frontLeftDrive.setPower(0);
        backRightDrive.setPower(0);
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);

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
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPowerFront, leftPowerBack, rightPowerBack, rightPowerFront);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
