/*
 * Copyright (c) 2018 Craig MacFarlane
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted
 * (subject to the limitations in the disclaimer below) provided that the following conditions are
 * met:
 *
 * Redistributions of source code must retain the above copyright notice, this list of conditions
 * and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions
 * and the following disclaimer in the documentation and/or other materials provided with the
 * distribution.
 *
 * Neither the name of Craig MacFarlane nor the names of its contributors may be used to
 * endorse or promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE. THIS
 * SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

public class BlinkinController
{
    OpMode opMode;

    RevBlinkinLedDriver blinkinLedDriver;

    Telemetry.Item display;
    DisplayKind displayKind;

    double cooldownTime = 0;

    protected enum DisplayKind {
        MANUAL,
        AUTO
    }

    public BlinkinController(OpMode setOpMode)
    {
        opMode = setOpMode;
        displayKind = DisplayKind.AUTO;

        blinkinLedDriver = opMode.hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
    }


    protected void setDisplayKind(DisplayKind displayKind)
    {
        this.displayKind = displayKind;
        display.setValue(displayKind.toString());
    }

    public void SetPattern(RevBlinkinLedDriver.BlinkinPattern pattern){
        blinkinLedDriver.setPattern(pattern);
    }

    public void SetCooldown(double seconds){cooldownTime = opMode.getRuntime()+seconds;}
    boolean IsCooldownUp(){return cooldownTime<opMode.getRuntime();}

    public void Blue(){
        if(!IsCooldownUp()) return;
        blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
    }
    public void Red(){
        if(!IsCooldownUp()) return;
        blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
    }
    public void Green(){
        if(!IsCooldownUp()) return;
        blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
    }
    public void Lime(){
        if(!IsCooldownUp()) return;
        blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.LIME);
    }
    public void Yellow(){
        if(!IsCooldownUp()) return;
        blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
    }
    public void Purple(){
        if(!IsCooldownUp()) return;
        blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
    }
}
