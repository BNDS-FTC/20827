package org.firstinspires.ftc.teamcode.PYZ;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class AutoTestRight extends AutoMaster{

   @Override
   public void runOpMode() throws InterruptedException{
      startSide = RIGHT;
      firstJunctionPos = Junction.;
      try {
         initHardware();
         longMoveNormal();
         eject(Junction.RIGHT,100);
      } catch (Exception e){
         throw new InterruptedException();
      }
   }
}
