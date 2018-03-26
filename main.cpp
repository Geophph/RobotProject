#include<FEHUtility.h>
#include<FEHServo.h>
#include<FEHMotor.h>
#include<FEHLCD.h>
#include<FEHIO.h>
#include<FEHSD.h>
#include<math.h>
#include<FEHRPS.h>

//define values that are dependent on sensor used
#define redmax 2.0
#define redmin 1.6
#define bluemax 2.7
#define bluemin 2.2
#define lefton 2.104
#define leftoff 0.189
#define middleon 2.320
#define middleoff 0.398
#define righton 2.125
#define rightoff 0.183
#define motoroffset -0.055
#define liftstart 90
#define PI (3.141592653589793)
#define wheel_dist 8.75
#define wheel_rad 1.25
#define long_sleep 0.3
#define short_sleep 0.15
#define precise 0.2

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

int distance(float dist){
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

void stop()
{   r_motor.Stop();
    l_motor.Stop();
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
    LCD.WriteLine(color_return);
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
    stop();
}

void lift_arm(int degrees){
    Sleep(short_sleep);
    lifter.SetDegree(liftstart - degrees);
    Sleep(1.0);
}

void drive(float speed, float dist){
    right_encoder.ResetCounts();
    left_encoder.ResetCounts();
    while(distance(dist)){
        r_motor.SetPercent(speed+motoroffset);
        l_motor.SetPercent(speed);
    }
    stop();
}

void turn_right(int percent,int counts)//usingencoders
{   right_encoder.ResetCounts();
    left_encoder.ResetCounts();
    r_motor.SetPercent(-percent);
    l_motor.SetPercent(percent);
    while((left_encoder.Counts() + (right_encoder.Counts())) / 2. < counts);
    stop();
}

void turn_left(int percent,int counts)//usingencoders
{   right_encoder.ResetCounts();
    left_encoder.ResetCounts();
    r_motor.SetPercent(percent);
    l_motor.SetPercent(-percent);
    while((left_encoder.Counts() + (right_encoder.Counts())) / 2. < counts);
    stop();
}

void turn_right_degrees(int percent,float degrees)//usingencoders
{   float counts = 318*wheel_dist/8/1.25*degrees/90;
    right_encoder.ResetCounts();
    left_encoder.ResetCounts();
    r_motor.SetPercent(-percent);
    l_motor.SetPercent(percent);
    while((left_encoder.Counts() + (right_encoder.Counts())) / 2. < counts);
    stop();
}

void turn_left_degrees(int percent, float degrees)//usingencoders
{   float counts = 318*wheel_dist/8/wheel_rad*degrees/90;
    right_encoder.ResetCounts();
    left_encoder.ResetCounts();
    r_motor.SetPercent(percent);
    l_motor.SetPercent(-percent);
    while((left_encoder.Counts() + (right_encoder.Counts())) / 2. < counts);
    stop();
}

void pivot_turn_left_time(int percent, float time)//usingencoders
{   r_motor.SetPercent(percent);
    l_motor.SetPercent(0);
    Sleep(time);
    stop();
}

void pivot_turnleft(int percent,int counts)//usingencoders
{   right_encoder.ResetCounts();
    left_encoder.ResetCounts();
    r_motor.SetPercent(percent);
    l_motor.SetPercent(0);
    while(right_encoder.Counts() < counts);
    stop();
}

void pivot_turn_right(int percent,int counts)//usingencoders
{   right_encoder.ResetCounts();
    left_encoder.ResetCounts();
    l_motor.SetPercent(percent);
    r_motor.SetPercent(0);
    while(left_encoder.Counts() < counts);
    stop();
}

void check_heading(float heading)
{
    float lowerheading = heading - 1;
    float upperheading = heading + 1;
    Sleep(short_sleep);
    if(lowerheading<0||upperheading>359.9){
        if(lowerheading<0){
            lowerheading = 359.9 + lowerheading;
        }
        if(upperheading>359.9){
            upperheading = 0 + upperheading;
        }
        Sleep(long_sleep);
        while(RPS.Heading()<lowerheading && RPS.Heading()>upperheading){
            if(RPS.Heading() > heading){
                if(abs(RPS.Heading()-heading)<180){
                    turn_right(13,1);
                    stop();
                }
                else{
                    turn_left(13,1);
                    stop();
                }
            }
            else if(RPS.Heading() < heading){
                if(abs(RPS.Heading()-heading)<180){
                    turn_left(13,1);
                    stop();
                }
                else{
                    turn_right(13,1);
                    stop();
                }
            }
            Sleep(short_sleep);
        }

    }
    else{
    Sleep(long_sleep);
    while(RPS.Heading()<lowerheading||RPS.Heading()>upperheading){
        if(RPS.Heading() > heading){
            if((RPS.Heading()-heading)<180){
                turn_right(13,1);
                stop();
            }
            else{
                turn_left(13,1);
                stop();
            }
        }
        else if(RPS.Heading() < heading){
            if(abs(RPS.Heading()-heading)<180){
                turn_left(13,1);
                stop();
            }
            else{
                turn_right(13,1);
                stop();
            }
        }
         Sleep(short_sleep);
    }
    }
}

void drive_heading(int speed, float dist, int heading){
    right_encoder.ResetCounts();
    left_encoder.ResetCounts();
    while(distance(dist)){
        r_motor.SetPercent(speed+motoroffset);
        l_motor.SetPercent(speed);
        check_heading(heading);
    }
    stop();
}

void check_x_plus(float x_coordinate)
{
    //check whether the robot is within an acceptable range
    Sleep(long_sleep);
    while(RPS.X() < x_coordinate - precise || RPS.X() > x_coordinate + precise)
    {
        Sleep(long_sleep);
        if(RPS.X() > x_coordinate)
        {
            //pulse the motors for a short duration in the correct direction
            drive(-13,0.1);
        }
        else if(RPS.X() < x_coordinate)
        {
            //pulse the motors for a short duration in the correct direction
            drive(13,0.1);
        }
    }
}

void check_x_minus(float x_coordinate) //using RPS while robot is in the -x direction
{
    //check whether the robot is within an acceptable range
    Sleep(long_sleep);
    while(RPS.X() < x_coordinate - precise || RPS.X() > x_coordinate + precise)
    {
        Sleep(long_sleep);
        if(RPS.X() > x_coordinate)
        {
            //pulse the motors for a short duration in the correct direction
            drive(13,0.1);
        }
        else if(RPS.X() < x_coordinate)
        {
            //pulse the motors for a short duration in the correct direction
            drive(-13,0.1);
        }
    }
}

void check_y_minus(float y_coordinate) //using RPS while robot is in the -y direction
{
    //check whether the robot is within an acceptable range
    Sleep(long_sleep);
    while(RPS.Y() < y_coordinate - precise || RPS.Y() > y_coordinate + precise)
    {
        Sleep(long_sleep);
        if(RPS.Y() > y_coordinate)
        {
            //pulse the motors for a short duration in the correct direction
             drive(13,0.1);
        }
        else if(RPS.Y() < y_coordinate)
        {
            //pulse the motors for a short duration in the correct direction
            drive(-13,0.1);
        }
    }
}

void check_y_plus(float y_coordinate) //using RPS while robot is in the +y direction
{
    //check whether the robot is within an acceptable range
    Sleep(long_sleep);
    while(RPS.Y() < y_coordinate - precise || RPS.Y() > y_coordinate + precise)
    {
        Sleep(long_sleep);
        if(RPS.Y() > y_coordinate)
        {
            //pulse the motors for a short duration in the correct direction
            drive(-13,0.1);
        }
        else if(RPS.Y() < y_coordinate)
        {
            //pulse the motors for a short duration in the correct direction
            drive(13,0.1);
        }
    }
}

void drivetime(float percent, float time){
    l_motor.SetPercent(percent);
    r_motor.SetPercent(percent);
    Sleep(time);
    stop();
}

void cranksetup(int crank_dir){
    if(crank_dir == 1){
        turner.SetDegree(180);
    }
    else{
        turner.SetDegree(0);
    }
}

void crankturn(int crank_dir){
    if(crank_dir == 1){
        turner.SetDegree(0);
    }
    else{
        turner.SetDegree(180);
    }
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
    lifter.SetMin(765);
    lifter.SetMax(2460);
    turner.SetMin(500);
    turner.SetMax(2413);
    //define Icons used in menu
    FEHIcon::Icon start;
    FEHIcon::Icon quit;
    start.SetProperties("Start", 10, 10, 80, 30,BLACK,WHITE);
    quit.SetProperties("Quit", 10, 50, 80, 30,BLACK,WHITE);
    RPS.InitializeTouchMenu();
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
                        lifter.SetDegree(liftstart);
                        turner.SetDegree(0);
                        run_num++;
                        while(color()!=0);
                        turner.SetDegree(0);
                        check_heading(270);
                        drive(20, 3);
                        check_y_minus(25.0);
                        turn_right_degrees(20,85.0);
                        check_heading(180);
                        drive(20, 10.75);
                        turn_right_degrees(20, 75.0);
                        check_heading(90);
                        drive(70,28);
                        turn_right_degrees(20, 40);
                        drive(20, 12);
                        turn_right_degrees(20, 48);
                        drive(20, 20);
                        //cranksetup(RPS.FuelType());
                        turner.SetDegree(180);
                        Sleep(1.0);
                        pivot_turn_left_time(-20, 0.5);
                        drivetime(20,0.25);
                        pivot_turn_left_time(-20, 2.5);
                        turner.SetDegree(0);
                        //crankturn(RPS.FuelType());
                        Sleep(1.0);
                        mode = 0;
        }
}
    lifter.SetDegree(liftstart);
    turner.SetDegree(0);
    SD.CloseLog();
}
