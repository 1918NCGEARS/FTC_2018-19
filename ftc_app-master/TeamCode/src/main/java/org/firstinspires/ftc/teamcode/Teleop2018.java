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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="Teleop 2018", group="Iterative Opmode")
//@Disabled
public class Teleop2018 extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor elev = null;
    private DcMotor leftElev = null;
    private DcMotor rightElev = null;
    private CRServo intake = null;

    private DigitalChannel lowerLimit = null;
    private DigitalChannel upperLimit = null;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        leftElev = hardwareMap.get(DcMotor.class, "left_elev");
        rightElev = hardwareMap.get(DcMotor.class, "right_elev");
        intake = hardwareMap.get(CRServo.class,"intake");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        leftElev.setDirection(DcMotor.Direction.REVERSE) ;
        rightElev.setDirection(DcMotor.Direction.FORWARD) ;

        leftElev.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightElev.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lowerLimit = hardwareMap.digitalChannel.get("lower_limit");
        upperLimit = hardwareMap.digitalChannel.get("upper_limit");
        lowerLimit.setMode(DigitalChannel.Mode.INPUT);
        upperLimit.setMode(DigitalChannel.Mode.INPUT);

//        rightElev.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);  // Encoder on Left Elev only

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
        double leftDrivePower;
        double rightDrivePower;
        double leftElevPower;
        double rightElevPower;
        boolean lowerLimitPressed;
        boolean upperLimitPressed;
        double intakeDir;

        // Tank Mode uses one stick to control each wheel.
        // - This requires no math, but it is hard to drive forward slowly and keep straight.
        leftDrivePower  = -gamepad1.left_stick_y * 0.5;
        rightDrivePower = -gamepad1.right_stick_y * 0.5;

        lowerLimitPressed = !lowerLimit.getState();
        upperLimitPressed = !upperLimit.getState();


        if (gamepad2.a && !lowerLimitPressed){
            leftElevPower = -0.25;
            rightElevPower = -0.25;
        }
        else if (gamepad2.x && !lowerLimitPressed) {
            leftElevPower = -1;
            rightElevPower = -1;
        }
        else if (gamepad2.y && !upperLimitPressed){
            leftElevPower = 1;
            rightElevPower = 1;
        }
        else {
            leftElevPower = 0;
            rightElevPower = 0;
        }

        if (gamepad2.right_bumper) {
            intakeDir = 1;
        }
        else if (gamepad2.right_trigger > 0) {
            intakeDir = -1;
        }
        else {
            intakeDir = 0;
        }

        // Send power to actuators
        leftDrive.setPower(leftDrivePower);
        rightDrive.setPower(rightDrivePower);
        leftElev.setPower(leftElevPower);
        rightElev.setPower(rightElevPower);
        intake.setPower(intakeDir);

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Drive", "left (%.2f), right (%.2f)", leftDrivePower, rightDrivePower);
        telemetry.addData( "Elev", "left (%.2f), right (%.2f)", leftElevPower, rightElevPower);
        telemetry.addData("Limit", "lower (%b), upper (%b)", lowerLimitPressed, upperLimitPressed);
//        telemetry.addData("Zero Power", "leftElev (%s), rightElev (%s), leftDrive (%s), rightDrive (%s)",
//                leftElev.getZeroPowerBehavior(), rightElev.getZeroPowerBehavior(), leftDrive.getZeroPowerBehavior(),
//                rightDrive.getZeroPowerBehavior());
//        telemetry.addData("Elev Enc", rightElev.getCurrentPosition());

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
