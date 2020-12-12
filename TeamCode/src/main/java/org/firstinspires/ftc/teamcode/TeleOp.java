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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOp", group="Linear Opmode")
//@Disabled
public class TeleOp extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFront = null;
    private DcMotor leftRear = null;
    private DcMotor rightFront = null;
    private DcMotor rightRear = null;
    private Servo scoopServo = null;
    private DcMotor liftMotor = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        telemetry.addData("Status", "Initialized");



        leftFront  = hardwareMap.get(DcMotor.class, "left_front");
        leftRear  = hardwareMap.get(DcMotor.class, "left_rear");
        rightFront = hardwareMap.get(DcMotor.class, "right_front");
        rightRear = hardwareMap.get(DcMotor.class, "right_rear");
        //scoopServo = hardwareMap.get(Servo.class, "scoop_servo");
        liftMotor = hardwareMap.get(DcMotor.class, "lift_motor");

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);



        //scoopServo.setPosition(0);

//        telemetry.addData("Scooper Position", scoopServo.getPosition());
//        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            double leftFrontPower;
            double leftRearPower;
            double rightFrontPower;
            double rightRearPower;



            double drive = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
            double turn  =  Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
            double strafe = gamepad1.right_stick_x;


            double liftMotorPowerDown;
            double liftMotorPowerUp;
            double liftDown = Math.pow(gamepad1.left_trigger, 3);
            double liftUp =   Math.pow(gamepad1.right_trigger, 3);


            rightFrontPower = Range.clip(drive * Math.cos(turn) + strafe, -1.0, 1.0);
            rightRearPower = Range.clip(drive * Math.sin(turn) + strafe, -1.0, 1.0);
            leftFrontPower = Range.clip(drive * Math.sin(turn) - strafe, -1.0, 1.0);
            leftRearPower = Range.clip(drive * Math.cos(turn) - strafe, -1.0, 1.0);

            liftMotorPowerDown = Range.clip(liftDown, -1.0, 1.0);
            liftMotorPowerUp = Range.clip(liftUp, -1.0, 1.0);

            leftFront.setPower(leftFrontPower * 0.9);
            leftRear.setPower(leftRearPower * 0.9);
            rightFront.setPower(rightFrontPower * 0.9);
            rightRear.setPower(rightRearPower * 0.9);
//
//            if(gamepad1.x)
//            {
//                scoopServo.setPosition(.32);
//            }
//
//            if(gamepad1.b)
//            {
//                scoopServo.setPosition(0);
//            }

            if (gamepad1.left_trigger > 0)
            {
                liftMotor.setPower(-liftMotorPowerDown * 0.5);
            }
            else if (gamepad1.right_trigger > 0)
            {
                liftMotor.setPower(liftMotorPowerUp * 0.75 + 0.40);
            }
            else
            {
                liftMotor.setPower(0);
            }




            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left front (%.2f), right front(%.2f), " +
                    "left rear(%.2f), right rear(%.2f)", leftFrontPower, rightFrontPower, leftRearPower,
                    rightRearPower);
            telemetry.update();
        }
    }
}

