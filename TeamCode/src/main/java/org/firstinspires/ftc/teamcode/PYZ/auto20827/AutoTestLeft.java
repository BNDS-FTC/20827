package org.firstinspires.ftc.teamcode.PYZ.auto20827;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class AutoTestLeft extends AutoMaster{

    int i=5;
    @Override
    public void runOpMode() throws InterruptedException{
        startSide = LEFT;
        firstJunctionPos = Junction.RIGHT;
        try {
            initHardware();
            longMoveNormal();
            while(i>0) {
                eject(Junction.RIGHT, 300);
                intermediate(i-1, Junction.RIGHT);
                intake(Junction.RIGHT);
                intermediate(5, Junction.GRAB);
                i--;
            }
            eject(Junction.RIGHT,300);
            park();
        } catch (Exception e){
            throw new InterruptedException();
        }
    }
}

