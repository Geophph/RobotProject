#include<FEHUtility.h>
#include<FEHServo.h>
#include<FEHMotor.h>
#include<FEHLCD.h>
#include<FEHIO.h>
#include<FEHSD.h>
#include <math.h>

//define values that are dependent on sensor used
#define redmax 1.9
#define redmin 1.7
#define bluemax 2.5
#define bluemin 2.3
#define lefton 2.104
#define leftoff 0.189
#define middleon 2.320
#define middleoff 0.398
#define righton 2.125
#define rightoff 0.183
#define PI (3.141592653589793)

//define each sensor or motor
FEHMotor l_motor(FEHMotor::Motor3,9.0);
FEHMotor r_motor(FEHMotor::Motor2,9.0);
DigitalEncoder right_encoder(FEHIO::P0_1);
DigitalEncoder left_encoder(FEHIO::P0_2);
AnalogInputPin colorsensor(FEHIO::P0_0);
AnalogInputPin linel(FEHIO::P3_7);
AnalogInputPin linem(FEHIO::P3_6);
AnalogInputPin liner(FEHIO::P3_5);
FEHServo lifter(FEHServo::Servo0);
FEHServo turner(FEHServo::Servo1);

int distance(int dist){
    int travel;
    float counts;
    counts = (127.2/PI)*dist;
    if((left_encoder.Counts() + (right_encoder.Counts())) / 2.0 < counts){
        travel = 1;
    }
    else{
        travel = 0;
    }
    return travel;
}


int color(void){
    float color = colorsensor.Value();
    int color_return;
    if(color<redmax && color>redmin){
        //color return 0 equals red
        color_return = 0;
    }
    else if(color<bluemax&&color>bluemin){
        //color return 1 equals blue
        color_return = 1;
    }
    else{
        //color return 2 equals not red or blue
        color_return = 2;
    }
    return color_return;
}

void track_line(int speed, int dist){
    int state = 1;
    float left, middle, right;
    right_encoder.ResetCounts();
    left_encoder.ResetCounts();
    while(distance(dist)){
        left=linel.Value();
        middle=linem.Value();
        right=liner.Value();

        if(abs(lefton-left)>0.4)
        {
            state=2;                    //too far left
            r_motor.SetPercent(30);
            l_motor.SetPercent(10);
        }
        else if(abs(righton-right)>0.4)
        {
            state=3;                    //too far right
            r_motor.SetPercent(10);
            l_motor.SetPercent(30);
        }
        else
        {
            state=1;
            r_motor.SetPercent(20);
            l_motor.SetPercent(20);
        }
    }
    r_motor.SetPercent(0);
    l_motor.SetPercent(0);
}

void lift_arm(int height){

}

void drive(int speed, int dist){
    right_encoder.ResetCounts();
    left_encoder.ResetCounts();
    while(distance(dist)){
        r_motor.SetPercent(speed);
        l_motor.SetPercent(speed);
        LCD.WriteLine(right_encoder.Counts());
        LCD.WriteLine(left_encoder.Counts());
    }
    r_motor.SetPercent(0);
    l_motor.SetPercent(0);
}

void turn_right(int percent,int counts)//usingencoders
{   right_encoder.ResetCounts();
    left_encoder.ResetCounts();
    r_motor.SetPercent(-percent);
    l_motor.SetPercent(percent);
    while((left_encoder.Counts() + (right_encoder.Counts())) / 2. < counts);
    r_motor.Stop();
    l_motor.Stop();
}

void turn_left(int percent,int counts)//usingencoders
{   right_encoder.ResetCounts();
    left_encoder.ResetCounts();
    r_motor.SetPercent(percent);
    l_motor.SetPercent(-percent);
    while((left_encoder.Counts() + (right_encoder.Counts())) / 2. < counts);
    r_motor.Stop();
    l_motor.Stop();
}

void stop()
{   r_motor.Stop();
    l_motor.Stop();
}

void logrws(){
    SD.Printf("jeff");
}

int main(void)
{   int mode=0, run=1, run_num=0;
    float x, y;
    //open log to track runs
    SD.OpenLog();
    //set values min and max for servos
    lifter.SetMin(500);
    lifter.SetMax(2400);
    turner.SetMin(500);
    turner.SetMax(2400);
    //define Icons used in menu
    FEHIcon::Icon start;
    FEHIcon::Icon quit;
    start.SetProperties("Start", 10, 10, 80, 30,BLACK,WHITE);
    quit.SetProperties("Quit", 10, 50, 80, 30,BLACK,WHITE);
    LCD.Clear();
    while(run){
        //menu to start and quit
        while(mode==0&&run){
            int press=0;
            start.Draw();
            quit.Draw();
            while(LCD.Touch(&x,&y)&&!press){
                if(start.Pressed(x,y,1)){
                    press=1;
                    mode = 1;
                    LCD.Clear();
                }
                else if(quit.Pressed(x,y,1)){
                    press=1;
                    run = 0;
                    LCD.Clear();
                }
            }
        }
        //run program
        while(mode==1&&run){
            run_num++;
            while(color()!=0);
            drive(20, 6);
            /*turn_left(20, 200);
            drive(20, 6);
            stop();
            drive(-20, 6);
            stop();
            turn_right(20, 200);
            drive(20, 6);
            turn_right(20, 200);
            stop();
            drive(-20, 6);
            stop();
            turn_right(20, 200);
            drive(20, 6);*/
            stop();
            mode = 0;
        }
}
    SD.CloseLog();
}
