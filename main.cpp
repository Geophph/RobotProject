#include<FEHUtility.h>
#include<FEHServo.h>
#include<FEHMotor.h>
#include<FEHLCD.h>
#include<FEHIO.h>
#include<FEHSD.h>
#include<math.h>
#include<FEHRPS.h>

//define values that are dependent on sensor used
#define redmax 2.3
#define redmin 1.7
#define bluemax 3.1
#define bluemin 2.9
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

//when given a distance in inches returns if the encoders counts average out to the required counts found by an equation
int distance(float dist){
    int travel;
    float counts;
    //converts inches desired to counts needed
    counts = (127.2/PI)*dist;
    //checks if distance traveled is enough
    if((left_encoder.Counts() + (right_encoder.Counts())) / 2.0 < counts){
        travel = 1;
    }
    else{
        travel = 0;
    }
    //returns 1 if not done and 0 if done
    return travel;
}

//stops left andd right motor
void stop()
{   r_motor.Stop();
    l_motor.Stop();
}

//returns an integer that reflects the color of LED under the CDS cell
int color(){
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
    //prints out integer of color being sensed and returns it
    LCD.WriteLine(color_return);
    return color_return;
}

//lifts the arm the desired number of degrees
void lift_arm(int degrees){
    Sleep(short_sleep);
    //beginning position is at 90 degrees and lesser degrees are needed lift so start minus desired is used
    lifter.SetDegree(liftstart - degrees);
    Sleep(1.0);
}

//drives straight for a given distance at a given power
void drive(float speed, float dist){
    //resets counts so distance function is accurate
    right_encoder.ResetCounts();
    left_encoder.ResetCounts();
    //set start time of function
    double starttime = TimeNow();
    //while distance traveled is less than distance needed to travel
    while(distance(dist)){
        r_motor.SetPercent(speed+motoroffset);
        l_motor.SetPercent(speed);
        if(TimeNow()-starttime>15){
            stop();
            break;
        }
    }
    stop();
}

//when given power and required counts the robot will turn right
void turn_right(int percent,int counts)//usingencoders
{   //reset counts for accurate count
    right_encoder.ResetCounts();
    left_encoder.ResetCounts();
    //set motor percents, right motor posotive & left motor negative
    r_motor.SetPercent(-percent);
    l_motor.SetPercent(percent);
    //until the average counts equals counts keep motors on
    while((left_encoder.Counts() + (right_encoder.Counts())) / 2. < counts);
    stop();
}

//when given power and required counts the robot will turn left
void turn_left(int percent,int counts)//usingencoders
{   //reset counts for accurate count
    right_encoder.ResetCounts();
    left_encoder.ResetCounts();
    //set motor percents, left motor posotive & right motor negative
    r_motor.SetPercent(percent);
    l_motor.SetPercent(-percent);
    //until the average counts equals counts keep motors on
    while((left_encoder.Counts() + (right_encoder.Counts())) / 2. < counts);
    stop();
}

//when given power and required degrees the robot will turn right the desired degrees
void turn_right_degrees(int percent,float degrees)//usingencoders
{   //calcuate needed counts based on degrees given
    float counts = 318*wheel_dist/8/1.25*degrees/90;
    //reset counts for accurate count
    right_encoder.ResetCounts();
    left_encoder.ResetCounts();
    //set motor percents, left motor posotive & right motor negative
    r_motor.SetPercent(-percent);
    l_motor.SetPercent(percent);
    while((left_encoder.Counts() + (right_encoder.Counts())) / 2. < counts);
    stop();
}

//when given power and required degrees the robot will turn left the desired degrees
void turn_left_degrees(int percent, float degrees)//usingencoders
{   //calcuate needed counts based on degrees given
    float counts = 318*wheel_dist/8/1.25*degrees/90;
    //reset counts for accurate count
    right_encoder.ResetCounts();
    left_encoder.ResetCounts();
    //set motor percents, right motor posotive & left motor negative
    r_motor.SetPercent(percent);
    l_motor.SetPercent(-percent);
    while((left_encoder.Counts() + (right_encoder.Counts())) / 2. < counts);
    stop();
}

//when given power and required counts the robot will turn left the desired counts
void pivot_turn_left(int percent,int counts)//usingencoders
{   //reset counts for accurate count
    right_encoder.ResetCounts();
    left_encoder.ResetCounts();
    //set motor percent, right motor to percent & left motor to 0
    r_motor.SetPercent(percent);
    l_motor.SetPercent(0);
    //continue until right encoder reaches counts
    while(right_encoder.Counts() < counts);
    stop();
}

//when given power and required counts the robot will turn right the desired counts
void pivot_turn_right(int percent,int counts)//usingencoders
{   //reset counts for accurate count
    right_encoder.ResetCounts();
    left_encoder.ResetCounts();
    //set motor percent, left motor to percent & right motor to 0
    l_motor.SetPercent(percent);
    r_motor.SetPercent(0);
    //continue until right encoder reaches counts
    while(left_encoder.Counts() < counts);
    stop();
}

//checks to see if robots heading based on RPS is within 2 degrees of given heading
void check_heading(float heading)
{
    //lower max and upper max of possible acceptable heading
    float lowerheading = heading - 1;
    float upperheading = heading + 1;
    Sleep(short_sleep);
    //if boundary case where heading is whithin 1 of 360 or 0 then the boundries are readjusted to work
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
    //for all other cases
    else{
    Sleep(long_sleep);
    while(RPS.Heading()<lowerheading||RPS.Heading()>upperheading){
        if(RPS.Heading() > heading){
            if((RPS.Heading()-heading)<180){
                turn_right(18,1);
                stop();
            }
            else{
                turn_left(18,1);
                stop();
            }
        }
        else if(RPS.Heading() < heading){
            if(abs(RPS.Heading()-heading)<180){
                turn_left(18,1);
                stop();
            }
            else{
                turn_right(18,1);
                stop();
            }
        }
         Sleep(short_sleep);
    }
    }
}

//splits the distance being driven into the number of parts provided by times and drives that distance
//at the power provided the number of times provided checking heading after each drive
void drive_heading(int speed, float dist, float heading, int times){
    //reset counts for accurate distance
    right_encoder.ResetCounts();
    left_encoder.ResetCounts();
    //for loop for number of times provided
    for(int i=0; i<times; i++){
        //drive partial distance
        drive(speed, dist/times);
        //check heading for given heading
        check_heading(heading);
    }
    //stop motors
    stop();
}

//Adjusts robot position to reach correct x coordinate, used when robot is facing in the posotive x direction
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

//Adjusts robot position to reach correct x coordinate, used when robot is facing in the negative x direction
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

//Adjusts robot position to reach correct y coordinate, used when robot is facing in the negative y direction
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

//Adjusts robot position to reach correct y coordinate, used when robot is facing in the posotive y direction
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

//Makes teh robot drive at a given percent for a given time
void drivetime(float percent, float time){
    //Set the left and right motor to percent
    l_motor.SetPercent(percent);
    r_motor.SetPercent(percent);
    //Waits for given time
    Sleep(time);
    //stop motors
    stop();
}

//Using the fuel crank type integer provided by RPS this function
//positions the crank turning servo so that the crank casn be turned 180 degrees
void cranksetup(int crank_dir){
    //if the fuel crank type which is provided in the function call is 2
    if(crank_dir == 2){
        //set servo degree to 180
        turner.SetDegree(180);
    }
    //if the fuel crank type is 1
    else{
        //set servo degree to 0
        turner.SetDegree(0);
    }
}

//Using the fuel crank type integer provided by RPS this function turns the
//crank servo the correct direction while backing into the crank for better turning
void crankturn(int crank_dir){
    //sets the left and right motor to reverse to back into crank
    l_motor.SetPercent(-40);
    r_motor.SetPercent(-40);
    //Sleep for 1 and a half seconds to ensure contact is made
    Sleep(1.5);
    //if the fuel crank type which is provided in the function call is 2
    if(crank_dir == 2){
        //set servo degree to 0
        turner.SetDegree(0);
    }
    //if the fuel crank is type 1
    else{
        //set servo degree to 180
        turner.SetDegree(180);
    }
    //Sleep for 1 second to keep backing into crank while servo turns
    Sleep(1.0);
    //stop left and right motors
    stop();
}

//Every 5 seconds this function prints out the X coordinate, Y coordinate, and heading from RPS as well as the value from the CdS cell
void checkRPS(){
    while(true){
        //prints out x coordinate
        LCD.Write("X: ");
        LCD.WriteLine(RPS.X());
        //prints out y coordinate
        LCD.Write("Y: ");
        LCD.WriteLine(RPS.Y());
        //prints out heading
        LCD.Write("Heading: ");
        LCD.WriteLine(RPS.Heading());
        //prints out CdS cell value
        LCD.Write("Color Sensor: ");
        LCD.WriteLine(colorsensor.Value());
        //Wait 5 seconds before running again
        Sleep(5.0);
    }
}

//Waits for the light sensed by the CdS cell to be red, if the light isn't sensed within 30 seconds jump out of the loop
void startup(){
    //set the time when the function starts
    double starttime = TimeNow();
    //while red isn't sensed and it hasn't been more than 30 seconds do nothing
    while(color()!=0 && TimeNow()-starttime<30);
}

//Using this function the robot drives from the starting position to the led light, determines the color, presss the correct button
//(or the red button if no light is sensed), and slighty away from wall
void buttonpress(){
    //drive 5 inches forward
    drive_heading(40, 5, 270, 3);
    check_y_minus(22.38);
    //turn left 90 degrees(value varies from 90 due to momentum on turn) and ensure heading with check heading
    turn_left_degrees(40,75);
    check_heading(0);
    //drive 7.5 inches straight
    drive_heading(40, 7.5, 0, 4);
    check_x_plus(25.199);
    //if color is blue do code to press blue button
    if(color()==1){
        //drive backwards 4 inches
        drive(-30, 4);
        check_x_plus(21.1);
        //pivot turn 500 counts and ensure heading with check heading
        pivot_turn_left(30, 500);
        check_heading(90);
        //drive backwards for 6.5 seconds
        drivetime(-30,6.5);
         }
    //if color is red or no color is sensed
     else{
        //drive backwards for 7 inches
        drive(-40, 7);
        check_x_plus(18);
        //pivot turn 500 counts and ensure full 90 degree turn with check heading
        pivot_turn_left(30, 500);
        check_heading(90);
        //drive ackwards for 6.5 seconds
        drivetime(-30,6.5);
        }
    //drive forward 1.5 inches to create distance from wall
     drive(20, 1.5);
     //turn left 90 degrees(value varies from 90 due to momentum on turn) and ensure heading with check heading
     turn_left_degrees(30, 80);
     check_heading(180);
}

//Function makes robot drive to wrench and lift it
void wrench(){
    //drive 16.5 inches across course
    drive(40, 16.5);
    //a check x is needed here because the starting position before the drive is different depending on button pressed
    check_x_minus(11.4);
    //turn 90 degrees(value varies from 90 due to momentum on turn) and ensure heading with check heading
    turn_left_degrees(30, 75);
    check_heading(270);
    //drive straight for 3 inches
    drive_heading(25, 3, 270, 3);
    check_y_minus(19.5);
    //turn 90 degrees(value varies from 90 due to momentum on turn) and ensure heading with check heading
    turn_right_degrees(30, 80);
    check_heading(180);
    //drive straight for 3.5 inches
    drive_heading(25, 3.5, 180, 3);
    //drive straight for 1 second
    drivetime(15, 1);
    //lift arm 45 degrees
    lift_arm(45);
}

//Function makes robot pivot into crank and then correct direction to face the upper section
void carjack(){
    //pivot turn left 440 counts in the negative direction
    pivot_turn_left(-40, 440);
    //drive backwards for 0.6 seconds
    drivetime(-40, 0.6);
    //drive forward for 1.5 seconds
    drive(40, 1.5);
    //turn 10 degrees to the right and check correct heading with check heading
    turn_right_degrees(40, 10);
    check_heading(90);
}

//Fuction drives to ramp and lines up to go to upper level
void toramp(){
    //drive straight for 8 inches
    drive_heading(35, 8, 90, 4);
    check_y_plus(20.099);
    //turn 90 degrees(value isn't 90 because of momentum) and ensure heading with check heading
    turn_left_degrees(20,80);
    check_heading(180);
    //drive 2 inches
    drive(20, 2);
    check_x_minus(8.5);
    //turn 90 degrees(value isn't 90 because of momentum) and ensure heading with check heading
    turn_right_degrees(20, 80);
    check_heading(90);
}

//Function has robot drive up ramp, navigate to the fuel crank, and turn the fuel crank
void crank(){
    //drive forward for 3 seconds at higher power to reach top of ramp
    drivetime(50, 3);
    //negative right pivot turn for 1025 counts and ensure heading with check heading
    pivot_turn_right(-40, 1025);
    check_heading(272.3);
    //drive backwards for 4.1 inches
    drive(-30, 4.1);
    //negative left pivot turn for 250 counts
    pivot_turn_left(-40, 250);
    //call cranksetup with the fueltype as the input
    cranksetup(RPS.FuelType());
    //drive backwards for 7 inches
    drive(-30, 7);
    //call crankturn with the fueltype as the input
    crankturn(RPS.FuelType());
}

//Function has robot navigate back to lower level of course and hit the final button
void finalbutton(){
    //drive 12.5 inches away from crank
    drive(30, 12.5);
    //turn left 90 degrees
    turn_left_degrees(30, 90);
    //drive 6.5 inches to get out of dead zone
    drive(30, 6.5);
    //check heading to ensure robot is facing straight down the ramp
    check_heading(315);
    //drive 13.75 inches down ramp
    drive(30, 13.75);
    check_x_plus(28.099);
    //turn right 45 degrees
    turn_right_degrees(30, 45);
    check_heading(267);
    //drive forward 23.75 inches
    drive(30, 23.75);
    check_y_minus(19.75);
    //negative pivot turn right for 500 counts
    pivot_turn_right(-30,500);
    //drive backwards 4 inches
    drive(-40,4);
    //negative pivot turn left 500 counts
    pivot_turn_left(-30,500);
    //drive backwards 9 inches to press final button
    drive(-40,9);
}

int main(void)
{   int mode=0, run=1, run_num=0;
    float x, y;
    //set values min and max for servos
    lifter.SetMin(765);
    lifter.SetMax(2460);
    turner.SetMin(500);
    turner.SetMax(2413);
    lifter.SetDegree(liftstart);
    turner.SetDegree(0);
    //define Icons used in menu
    FEHIcon::Icon start;
    FEHIcon::Icon quit;
    start.SetProperties("Start", 10, 10, 80, 30,BLACK,WHITE);
    quit.SetProperties("Quit", 10, 80, 80, 30,BLACK,WHITE);
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
        //run each function for completing the course
        while(mode==1&&run){
             startup();
             buttonpress();
             wrench();
             carjack();
             toramp();
             crank();
             finalbutton();
        }
}
    lifter.SetDegree(liftstart);
    turner.SetDegree(0);
}
