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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import static org.firstinspires.ftc.teamcode.AutonFunctions.allStop;
import static org.firstinspires.ftc.teamcode.AutonFunctions.delay;
import static org.firstinspires.ftc.teamcode.AutonFunctions.drive;
import static org.firstinspires.ftc.teamcode.AutonFunctions.elev;
import static org.firstinspires.ftc.teamcode.AutonFunctions.findGold;
import static org.firstinspires.ftc.teamcode.AutonFunctions.index;
import static org.firstinspires.ftc.teamcode.AutonFunctions.initVision;
import static org.firstinspires.ftc.teamcode.AutonFunctions.initializeAuton;
import static org.firstinspires.ftc.teamcode.AutonFunctions.sendTelemetry;
import static org.firstinspires.ftc.teamcode.AutonFunctions.setElevZeroPwr;


@Autonomous(name="Crater Start", group ="Autonomous")
//@Disabled
public class Crater_Start extends OpMode {

    @Override
    public void init() {
        initializeAuton(hardwareMap, telemetry);
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        //initializeLoopAuton();
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        //startAuton();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        switch(index) {
            case 0:
                setElevZeroPwr(DcMotor.ZeroPowerBehavior.FLOAT);
                elev(675);
                break;
            case 1:
                setElevZeroPwr(DcMotor.ZeroPowerBehavior.BRAKE);
                initVision();
                break;
            case 2:
                drive(6,6,0.3);
                break;
            case 3:
                delay(1000);
                break;
            case 4:
                findGold();
                break;
            case 5:
                drive(4, 0, .3);
                break;
            case 6:
                delay(1000);
                break;
            case 7:
                findGold();
                break;
            case 8:
                drive(-4, 0, .3);
                break;
            case 9:
                drive(0, 6, .3);
                break;
            case 10:
                delay(1000);
                break;
            case 11:
                findGold();
                break;
            case 12:
                drive(40, 40, .3);
                break;
            case 13:
                elev(0);
                break;
            default:
                allStop();
        }
        sendTelemetry();
    }
}