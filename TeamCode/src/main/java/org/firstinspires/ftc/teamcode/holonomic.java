package org.firstinspires.ftc.teamcode;

public class holonomic {





    double Vertical;
    double Horizontal;
    double Pivot;
    private double FrontRight;
    private double BackRight;
    private double FrontLeft;
    private double BackLeft;



    public void run(double Xmov, double Ymov, double Thetamov, double speed) {
        Vertical = speed * Ymov;
        Horizontal = speed * -Xmov;
        Pivot = speed * -Thetamov;
        FrontRight = -Pivot + (Vertical - Horizontal);
        BackRight=-Pivot + Vertical + Horizontal;
        FrontLeft=Pivot + Vertical + Horizontal;
        BackLeft=Pivot + (Vertical - Horizontal);
    }

    public double FrontRight(){
        return FrontRight;
    }

    public double BackRight(){
        return BackRight;
    }

    public double BackLeft(){
        return BackLeft;
    }

    public double FrontLeft(){
        return FrontLeft;
    }

}
