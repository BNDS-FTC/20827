package org.firstinspires.ftc.teamcode.PYZ;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class AutoTestRight extends AutoMaster{

   int i=5;
   @Override
   public void runOpMode() throws InterruptedException{
      startSide = RIGHT;
      firstJunctionPos = Junction.RIGHT;
      try {
         initHardware();
         longMoveNormal();
         while(i>0) {
            eject(Junction.RIGHT, 100);
            intermediate(i-1, Junction.RIGHT);
            intake(Junction.RIGHT);
            intermediate(5, Junction.GRAB);
            i--;
         }
         eject(Junction.RIGHT,100);
         park();

      } catch (Exception e){
         throw new InterruptedException();
      }
   }
}
