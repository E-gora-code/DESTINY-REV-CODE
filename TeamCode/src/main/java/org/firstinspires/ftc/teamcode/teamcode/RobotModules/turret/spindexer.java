package org.firstinspires.ftc.teamcode.teamcode.RobotModules.turret;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.pedropathing.control.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.pedropathing.control.PIDFController;

import java.util.ArrayList;
import java.util.List;

public class spindexer {

    private Servo front_ejector, back_ejector, front_wall, back_wall;



    private CRServo spindexer,spindexer2;
    private AnalogInput spindexerPos;
    boolean colorflag = true;
    double posi = 0;

    private PIDFController
            spindex = new PIDFController(new PIDFCoefficients(0.0035, 0.0001, 0.0005, 0));
    ElapsedTime timer = new ElapsedTime();

    private DcMotorEx front_intake, back_intake;
    public double position=1;
    ColorSensor colr,colrback;
    public ArrayList<String> ball = new ArrayList<>();
    boolean spinflag = true,actflag;
    public void setServoSpeed(Servo servo, double speed){
        servo.setPosition(speed/2+0.5);
    }

    public spindexer(HardwareMap hw) {
        spindexer = hw.get(CRServo.class, "SP");
        spindexer2 = hw.get(CRServo.class, "SP2");
        front_ejector = hw.get(Servo.class, "Fej");
        back_ejector = hw.get(Servo.class, "Bej");
        front_wall = hw.get(Servo.class, "Fwall");
        back_wall = hw.get(Servo.class, "Bwall");

        front_intake = hw.get(DcMotorEx.class, "Fin");
        back_intake = hw.get(DcMotorEx.class, "Bin");
        spindexerPos = hw.get(AnalogInput.class, "SP_POS");

        back_intake.setDirection(DcMotorEx.Direction.REVERSE);

        front_intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        back_intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        front_intake.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        back_intake.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        colr = hw.get(ColorSensor.class, "cv0");
        colrback = hw.get(ColorSensor.class, "cv1");


        spindex.setCoefficients(new PIDFCoefficients(0.7, 0.000, 0.7, 0));


        ball.add("N");
        ball.add("N");
        ball.add("N");
    }

    public void update(boolean confi, boolean frontintaking,boolean backintaking, String color,boolean shooting,float power) {
        spindex.setCoefficients(new PIDFCoefficients(config.spinKp, config.spinKi, config.spinKd, 0));
        if (confi){
            spin((double) power);
            front_ejector.setPosition(config.pos_front_ejector);
        }
        else if (frontintaking) {
            if ((count("G")+count("P")==1)&&(position  == 4 - sumIndices(ball,"N"))){
                position  = ((position+1) % 3)+1;
            }
            else if  ((count("G")+count("P")==2)&& (position%1!=0)){
                position  = (position + 1.5) % 3+1;
            }
            spin(position);
            back_ejector.setPosition(0.85);
            back_wall.setPosition(1);
            front_ejector.setPosition(0.8);
            front_wall.setPosition(0.66);
            front_intake.setPower(1);
            back_intake.setPower(-1);


            if ((chek_green()||chek_purple()) && spinflag){
                if(chek_green())ball.set((int)((position +1) % 3),"G") ;
                if(chek_purple())ball.set((int)((position +1) % 3),"P") ;
                position = (position)%3+1;
                spinflag = false;
                actflag = true;
            }
            else if (!(chek_green()||chek_purple())){
                spinflag = true;
            }
        }
        else if (backintaking) {
            if ((count("G")+count("P")==1)&&(position  == 4 - sumIndices(ball,"N"))){
                position  = ((position)% 3) + 1;
            }
            else if  ((count("G")+count("P")==2)&& (position%1!=0)){
                position  = position+0.5;
            }
            spin(position);
            back_ejector.setPosition(0.85);
            back_wall.setPosition(0.66);
            front_ejector.setPosition(0.8);
            front_wall.setPosition(1
            );
            front_intake.setPower(1);
            back_intake.setPower(-1);


            if ((chek_green_back()||chek_purple_back()) && spinflag){
                if(chek_green())ball.set((int) ((position) % 3),"G") ;
                if(chek_purple())ball.set((int)((position) % 3),"P") ;
                position = ((position+1) % 3 + 1);
                spinflag = false;
                actflag = true;
                timer.reset();
            }
            else if (!(chek_green_back()||chek_purple_back())){
                spinflag = true;
            }
        }
        else if (shooting){
            if (sumIndices(ball,"N")==3){
                for (int i = 0; i < ball.size(); i++) {
                    ball.set(i,"N");
                }
            }
            position =1;
            spindexer.setPower(-1);
            spindexer2.setPower(-1);
            back_ejector.setPosition(0.85);
            back_wall.setPosition(1);
            front_ejector.setPosition(0.6);
            front_wall.setPosition(1);
            front_intake.setPower(1);
            back_intake.setPower(-1);
        }

        else {
            if((!(ball.contains("G")))&&(!(ball.contains("P")))){
                position  = 1;
            }
            else  if (count("G")+count("P")==1){
                 position  = 4 - sumIndices(ball,"N");
            }
            else if  (count("G")+count("P")==2){
                if ((4-sumIndices(ball,"N"))/2 == 2){
                    position  = 0.5;
                }
                else{
                    position  = (4-sumIndices(ball,"N"))/2;
                }
            }
            front_intake.setPower(0);
            back_intake.setPower(0);
            spindexer.setPower(0);
            spindexer2.setPower(0);
            spin(position);
            colorflag = true;
        }

    }

    public double spin(double pos) {
        double targetpos = pos-0.85;
        double currentpos = spindexerPos.getVoltage();
        posi = pos;
        spindex.updateError(targetpos - currentpos);

        if (currentpos < 1 && pos == 3) {
            spindexer.setPower(1);
            spindexer2.setPower(1);
        }
        else if (currentpos > 2&& pos ==1) {
            spindexer.setPower(-0.7);
            spindexer2.setPower(-0.7);
        }else {
            double power = -spindex.run();
            spindexer.setPower(power);
            spindexer2.setPower(power);
       }
        return 0;

         }
    public double v(){
        return posi;
    }

    public double color_green() {
        return colrback.green();
    }
    public List<String>bal(){
        return ball;
    }

    public double color_red() {
        return colrback.red();
    }
    public double color_blue() {
        return colrback.blue();
    }
    public boolean chek_green(){
        return ((colr.red()+25)<(colr.green()))&&((colr.blue()+25)<(colr.green()));
    }
    public boolean chek_purple(){
        return ((colr.green()+15)<(colr.blue()))&&((colr.red()+15)<(colr.blue()));
    }
    public boolean chek_green_back(){
        return ((colrback.red()+25)<(colrback.green()))&&((colrback.blue()+25)<(colrback.green()));
    }
    public boolean chek_purple_back(){
        return ((colrback.green()+15)<(colrback.blue()))&&((colrback.red()+15)<(colrback.blue()));
    }
    public double count(String target){
        double count = 0;

        for (String item : ball) {
            if (item.equals(target)) {
                count++;
            }
        }
        return  count;
    }
    public static <T> int sumIndices(ArrayList<T> list, T target) {
        int sum = 0;

        for (int i = 0; i < list.size(); i++) {
            if (target == null) {
                if (list.get(i) == null) {
                    sum += i;
                }
            } else if (target.equals(list.get(i))) {
                sum += i;
            }
        }

        return sum;
    }
}
