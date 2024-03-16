package org.firstinspires.ftc.teamcode;

public class RunToPosition {

    //constants
    final private double WheelRadius = 1.88976/2; //diameter to radius
    final private double CMperTick = 2 * Math.PI * WheelRadius / 2000;
    final private double NearRadius = 24;

    //IMU stuffs
    //   BNO055IMU imu;
    //   Orientation angles = new Orientation();

    //variables
    public double Xpower;
    public double Ypower;

    public boolean atpos;
    public double DeltaX;
    public double DeltaY;
    //    private double DeltaTheta;
    public double PosX;
    public double PosY;
    //   private double PosTheta; // in radians
//    final private double LeftRightDist=7.625*2.54; //Distance between the left and right wheels Inches converted to CM


    //main program flow
    public void Run(double Left, double Back, double TPosX, double TPosY, double speed, double margin) {

        PosX = CMperTick * -Back;
        PosY = CMperTick * Left;
        // PosTheta = angles.

        DeltaX = TPosX - PosX;
        DeltaY = TPosY - PosY;

               atpos=false;
/*
        while(Math.abs(PosTheta) != 0+5){
            BackLeft = -0.1;
            BackRight = 0.1;
            FrontLeft = -0.1;
            FrontRight = 0.1;
        }
*/
        // below is desmos function
// f\left(x\right)=\left\{\operatorname{abs}\left(x\right)<6:0.25\cdot-\sin\left(x\cdot\pi\cdot\frac{1}{2}\cdot\frac{1}{6}\right),\operatorname{abs}\left(x\right)>6:0.25\cdot-\left(\frac{\operatorname{abs}\left(x\right)}{x}\right)\right\}
        if (Math.abs(DeltaY) > margin) {
            //           atpo=false;
            if (Math.abs(DeltaY) < Math.abs(NearRadius)) {

                Ypower = -speed * Math.sin(DeltaY * Math.PI * 1/2 * 1/NearRadius); // start slowing when inside 24 inches
            } else {
                Ypower = -speed * (Math.abs(DeltaY) / DeltaY);
            }
        } else {
            if (Math.abs(DeltaX) > margin) {
                if (DeltaX < Math.abs(NearRadius)) {
                    Xpower = speed * Math.sin(DeltaX * Math.PI * 1/2 * 1/NearRadius); // start slowing when inside 24 inches
                } else {
                    Xpower = speed * (Math.abs(DeltaX) / DeltaX);
                }
            } else {
                Xpower = 0;
                Ypower = 0;

                atpos=true;
            }

        }
    }

   /* public boolean atpos (){
        if (atpo) {
            return true;
        } else {
            return false;
        }
    } */
}



