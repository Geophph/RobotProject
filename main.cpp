#include<FEHUtility.h>
#include<FEHServo.h>
#include<FEHMotor.h>
#include<FEHLCD.h>
#include<FEHIO.h>
#include<FEHSD.h>

#define redmax 10
#define redmin 0
#define bluemax 2.7
#define bluemin 2.6

FEHMotor l_motor(FEHMotor::Motor0,9.0);
FEHMotor r_motor(FEHMotor::Motor1,9.0);
AnalogInputPin colorsensor(FEHIO::P3_7);
FEHServo lifter(FEHServo::Servo0);
FEHServo turner(FEHServo::Servo1);

int color(void){
    int color = colorsensor.Value();
    int color_return;
    if(color<redmax&&color>redmin){
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

void track_line(int speed, int distance){

}

void lift_arm(int height){

}

void drive(int speed, int distance){
    r_motor.SetPercent(speed);
    l_motor.SetPercent(speed);
}

void logrws(){
    SD.Printf("jeff");
}

int main(void)
{   int mode=0, run=1, run_num=0;
    float x, y;
    SD.OpenLog();
    lifter.SetMin(500);
    lifter.SetMax(2400);
    turner.SetMin(500);
    turner.SetMax(2400);
    FEHIcon::Icon start;
    FEHIcon::Icon stop;
    FEHIcon::Icon data;
    FEHIcon::Icon back;
    FEHIcon::Icon quit;
    LCD.Clear();
    start.SetProperties("Start", 10, 10, 80, 30,BLACK,WHITE);
    stop.SetProperties("Stop", 10, 10, 80, 30,BLACK,WHITE);
    data.SetProperties("Data", 10, 50, 80, 30,BLACK,WHITE);
    back.SetProperties("Back", 10, 10, 80, 30,BLACK,WHITE);
    quit.SetProperties("Quit", 10, 50, 80, 30,BLACK,WHITE);
    while(run){
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
        while(mode==1&&run){
            stop.Draw();
            int press=0;
            while(LCD.Touch(&x,&y)&&!press){
                if(stop.Pressed(x,y,1)){
                press=1;
                mode = 0;
                LCD.Clear();
                }
            }
        }
}
    SD.CloseLog();
}
