/* Copyright (c) 2015 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package org.firstinspires.ftc.team408.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.team408.Auto.AutoLib;

@Autonomous(name = "BEACON BLUE", group = "Beacons")
public class BEACON_BLUE extends AutoLib {

    public void runOpMode() throws InterruptedException {

        //Sets everything up and then waits for the start.
        config();
        waitForStart();
        runBlueAuto();
        return;
    }

    public void runBlueAuto() throws InterruptedException
    {
        //This is so the ball lift and button pusher are stable or out of the way and in the right place

        //Goes until the line and then hits the blue button
        lineThenButton("Blue");
        buttonPusher.setPower(-1.0);
        buttonPusherLeft.setPower(1.0);
        driveStraightFor(200, 0.5); //Backs up to get in the right place for shooting
        drivePower(0);
        rotateLeft(Math.PI / 40, HALF_POWER); //Adjusts for shooting ball
        //sleep(1000);
        shootBalls();
        buttonPusher.setPower(0);
        buttonPusherLeft.setPower(0.0);

        driveStraightFor(10, 0.5);

        drivePower(0);
        sleep(400);
        //We are having it not go as far out so it can shoot better 2-13-17

        //rotates about 90 degrees but not quite which is why 5 is only close to 4
        rotateLeft(0.9 * Math.PI / 2.5, QUARTER_POWER);//Still turns slightly inconsistent, it may be better to over turn than under turn 2-13-17
        //Makes sure the button pusher is in the right place


        //Goes until the line and then hits the blue button
        lineThenButton("Blue");//For some reason it turns too far here and I don't know why 2-14-17
        driveStraightFor(300, 0.5);//Separates from wall so it doesn't stay on the button



        //This is the new cap ball part
        rotateLeft(1 * Math.PI / 2.5, FULL_POWER);

        buttonPusher.setPower(-1.0);
        buttonPusherLeft.setPower(1.0);
        driveStraightFor(750, FULL_POWER);

        buttonPusher.setPower(0);
        buttonPusherLeft.setPower(0);





    }
}