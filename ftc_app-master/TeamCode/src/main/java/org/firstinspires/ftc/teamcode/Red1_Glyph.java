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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import static org.firstinspires.ftc.teamcode.AutonModes_v2.State.*;
import static org.firstinspires.ftc.teamcode.AutonModes_v2.*;


@Autonomous(name="Red1 - Glyph", group ="Autonomous")
@Disabled
public class Red1_Glyph extends OpMode {
    // Initialize motors and servos and declare private variables

    int stateIndex = 0;
    int statesArrayLength;
    final boolean isBlue = this.getClass().getName().startsWith("B", 31);
    AutonModes_v2 autonModes_v2 = new AutonModes_v2();


    @Override
    public void init() {
        autonModes_v2.initializeAuton(hardwareMap, telemetry);
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        initializeLoopAuton();
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        startAuton();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // List auton actions and move distances
        State[] autonStatesArr = {LIFT_UP, ARM_DOWN, RESET_TIME, ID_BALL, KNOCK_BALL,
                RESET_TIME, ARM_UP, DEP_MOVE, INDEP_MOVE, RESET_TIME, LIFT_DOWN, RESET_TIME, TILT_UP,
                INDEP_MOVE, INDEP_MOVE, RESET_TIME, TILT_DOWN, ALL_STOP};
        statesArrayLength = autonStatesArr.length;

        // Move distances, in inches.  Turns use pivot on one side ==> turn radius of 18 inches
        //                             Circumference = pi * 18^2 = 56.54866776.
        //                             Double to approximately compensate for wheel slip on turns
        double[] leftDriveMoveDist =  {0, 0, 0, 0,  3, 0, 0, 18,      0, 0, 0, 0, 0,  7, -3, 0, 0, 0};
        double[] rightDriveMoveDist = {0, 0, 0, 0,  3, 0, 0, 18, 28.274, 0, 0, 0, 0,  7, -3, 0, 0, 0};

        // Drive motor powers, POSITIVE = BACKWARDS
        // During turns, apply power to both sides so non-turning side can correct if needed
        double[] leftDrivePowerArr =  {0, 0, 0, 0, .3, 0, 0, .5,    .75, 0, 0, 0, 0, .5, .5, 0, 0, 0};
        double[] rightDrivePowerArr = {0, 0, 0, 0, .3, 0, 0, .5,    .75, 0, 0, 0, 0, .5, .5, 0, 0, 0};

        if (isBlue) {
            if (autonStatesArr[stateIndex] == DEP_MOVE){
                leftDriveMoveDist[stateIndex] += 2.25;
                rightDriveMoveDist[stateIndex] += 2.25;
            }
            leftDriveMoveDist[stateIndex] = -1 * leftDriveMoveDist[stateIndex];
            rightDriveMoveDist[stateIndex] = -1 * rightDriveMoveDist[stateIndex];
            leftDrivePowerArr[stateIndex] = -1 * leftDrivePowerArr[stateIndex];
            rightDrivePowerArr[stateIndex] = -1 * rightDrivePowerArr[stateIndex];
        }

        stateIndex = autonModes_v2.runAuton(stateIndex, autonStatesArr[stateIndex], leftDriveMoveDist[stateIndex],
                rightDriveMoveDist[stateIndex], leftDrivePowerArr[stateIndex], rightDrivePowerArr[stateIndex]);
    }
}