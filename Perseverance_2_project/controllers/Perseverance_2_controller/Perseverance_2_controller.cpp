//Team name : Perseverence_2;

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Emitter.hpp>
#include <webots/utils/AnsiCodes.hpp>

#include <bits/stdc++.h>
#include <boost/algorithm/clamp.hpp>
#include <iostream>
#include <sstream>
#include <string>
#include <algorithm>
#include <cstring>
#include <sstream>
#include <stdio.h>
#include <math.h>
#include <cmath>

#define TIME_STEP 32
using namespace std;
using namespace webots;

   
 Robot *robot = new Robot();
 
 DistanceSensor *GS[7];
  char GSNames[7][4] = {"gs7", "gs6", "gs5", "gs4", "gs3", "gs2", "gs1"};
  
 DistanceSensor *gs[3];
  char gsNames[3][4] = {"GS1", "GS2", "GS3"};

 DistanceSensor *DS[4];
  char DSNames[4][4] = {"SRM", "SLM", "SRF", "SLF"};
 
 Motor *right_motor;
 Motor *left_motor;    
  char wheels_names[2][18] = {"right wheel motor", "left wheel motor"};
  
 Emitter *EMI = robot -> getEmitter("emitter");

// PID Variables

 int Lposition; 
 int Lgoal = 400;
 double Lerror;
 double LlastError;

 double Werror;
 double WlastError;

 const double lKp = 0.0185;
 const double lKi = 0.0;
 const double lKd = 0.0;

 const double wKp = 0.3;
 const double wKi = 0.05;
 const double wKd = 0.05;


//best value 2nd
// const double Kp = 0.65;
// const double Ki = 0.001;
// const double Kd = 0.55;
 
//best value 1st
 //const double Kp = 0.45;
 //const double Ki = 0.0;
 //const double Kd = 0.3;
 
 double lP;
 double lI;
 double lD;
 
 double wP;
 double wI;
 double wD;

// Motor related variables
 double LmotorVal;

 double WmotorVal;

 double basespeed = 4.8;
 double LleftSpeed;
 double LrightSpeed;

 double WleftSpeed;
 double WrightSpeed;

//GSensor related variables
 double leftFarReading     = 0.0;
 double leftNearReading    = 0.0;
 double leftCenterReading  = 0.0;
 double MiddleReading      = 0.0;
 double rightCenterReading = 0.0;
 double rightNearReading   = 0.0;
 double rightFarReading    = 0.0; 
 
//gsensor related variables
 double Left   = 0.0;
 double Middle = 0.0;
 double Right  = 0.0;  

 double t  = 150.0;
 double t1 = 210.0; 

//DSensor related variables
  double SRM = 0.0;
  double SLM = 0.0;
  double SRF = 0.0;
  double SLF = 0.0;
//  double SF  = 0.0;

 //int numRightPillars = 0;
 //int numLeftPillars  = 0;
 
 
 int left_sum;
 int right_sum;
 
 int left_sum1;
 int right_sum1;
 
 int left_sum2;
 int right_sum2;
 
 int sum;
 
 int rSLM;
 int rSRM;
 
 bool already_ran = false;
 bool already_ran1 = false;
 bool already_ran2 = false;
 bool already_ran3 = false;
 bool already_ran4 = false;
 bool already_ran5 = false;
 bool already_ran6 = false;
 bool already_ran7 = false;
 bool already_ran8 = false;
 bool already_ran9 = false;
 bool already_ran10 = false;
 bool already_ran11 = false;
 bool already_ran12 = false;
 bool got = false;
 
 int n;
 int n1;
 
 string mesgRed  = "R";
 string mesgBlue = "B";
 
 int lefturn  = 0;
 int righturn = 0;

 string inRange  = "1";
 string outRange = "0";
 string inWhite;
 string inWhite1;

 double starT = 0.0;
 double Time = 0.0;
 
void initGS() {

  for (int c = 0; c <= 6; c++) {
    GS[c] = robot -> getDistanceSensor(GSNames[c]);
    GS[c] -> enable(TIME_STEP);
  } 
}

void initgs() {

  for (int a = 0; a <= 2; a++) {
    gs[a] = robot -> getDistanceSensor(gsNames[a]);
    gs[a] -> enable(TIME_STEP);
  } 
}

 void initDS() {

   for (int j = 0; j <= 3; j++) {
     DS[j] = robot -> getDistanceSensor(DSNames[j]);
     DS[j] -> enable(TIME_STEP);
   } 
 }
 
void initMotor() {
  
    right_motor = robot -> getMotor(wheels_names[0]);
    right_motor -> setPosition(INFINITY);
    
    left_motor = robot -> getMotor(wheels_names[1]);
    left_motor -> setPosition(INFINITY);
    
}

void straight() {

    right_motor -> setVelocity(6);
    left_motor  -> setVelocity(6);
}
  
void readGSensors() {
     
     leftFarReading     = GS[0] -> getValue();
     leftNearReading    = GS[1] -> getValue();
     leftCenterReading  = GS[2] -> getValue();
     MiddleReading      = GS[3] -> getValue();
     rightCenterReading = GS[4] -> getValue();
     rightNearReading   = GS[5] -> getValue();
     rightFarReading    = GS[6] -> getValue();

    // For debbuging purposes only
     
      // std::cout << "leftFarReading       --     "  << leftFarReading     << std::endl;
      // std::cout << "leftNearReading      --     "  << leftNearReading    << std::endl;
      // std::cout << "leftCenterReading    --     "  << leftCenterReading  << std::endl;
      // std::cout << "MiddleReading        --     "  << MiddleReading      << std::endl;
      // std::cout << "rightCenterReading   --     "  << rightCenterReading << std::endl;
      // std::cout << "rightNearReading     --     "  << rightNearReading   << std::endl;
      // std::cout << "rightFarReading      --     "  << rightFarReading    << std::endl;
     
      // std::cout << "#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#-#" << std::endl;
        
      Right  = gs[0] -> getValue();
      Middle = gs[1] -> getValue();
      Left   = gs[2] -> getValue();
      
    // For debbuging purposes only
      
        // std::cout << "Left     --     "  << Right   << std::endl;
        // std::cout << "Middle   --     "  << Middle  << std::endl;
        // std::cout << "Right    --     "  << Left    << std::endl;
          
        // std::cout << "/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/" << std::endl;
      
 }

 void readDSensors() {
        
      SRM = 12.08*pow(DS[0] -> getValue(), -1.058);
      SLM = 12.08*pow(DS[1] -> getValue(), -1.058);
      SRF = 12.08*pow(DS[2] -> getValue(), -1.058);
      SLF = 12.08*pow(DS[3] -> getValue(), -1.058); //y(x) = 0.1594*x^(-0.8533)-0.02916
      // SRB = 12.08*pow(DS[4] -> getValue(), -1.058);
      // SLB = DS[3] -> getValue();

    // For debugging purposes only,

      //  std::cout << "RightMiddle     ~~     "  << SRM << std::endl;
      //  std::cout << "Leftmiddle      ~~     "  << SLM << std::endl;
      //  std::cout << "LeftFront       ~~     "  << SLF << std::endl;
      //  std::cout << "RightFront      ~~     "  << SRF << std::endl;
      //  std::cout << "RightBack       ~~     "  << SRB << std::endl;
      //  std::cout << "LeftBack        ~~     "  << SLB << std::endl;        

      // std::cout << "+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-" << std::endl;

 }

void computeLinePosition() {

    bool error0  = (leftFarReading < t && leftNearReading > t && leftCenterReading > t && MiddleReading > t && rightCenterReading > t && rightNearReading > t && rightFarReading > t );//1
    bool error01 = (leftFarReading < t && leftNearReading < t && leftCenterReading < t && MiddleReading < t && rightCenterReading < t && rightNearReading < t && rightFarReading > t );
    bool error02 = (leftFarReading < t && leftNearReading < t && leftCenterReading < t && MiddleReading < t && rightCenterReading < t && rightNearReading > t && rightFarReading > t );// this one left turn
    bool error3  = (leftFarReading < t && leftNearReading < t && leftCenterReading > t && MiddleReading > t && rightCenterReading > t && rightNearReading > t && rightFarReading > t );//2
    bool error4  = (leftFarReading < t && leftNearReading < t && leftCenterReading < t && MiddleReading > t && rightCenterReading > t && rightNearReading > t && rightFarReading > t );//3
    bool error5  = (leftFarReading > t && leftNearReading < t && leftCenterReading < t && MiddleReading < t && rightCenterReading > t && rightNearReading > t && rightFarReading > t );//new
    bool error6  = (leftFarReading > t && leftNearReading < t && leftCenterReading < t && MiddleReading < t && rightCenterReading < t && rightNearReading > t && rightFarReading > t );//3
    bool error7  = (leftFarReading > t && leftNearReading > t && leftCenterReading < t && MiddleReading < t && rightCenterReading < t && rightNearReading > t && rightFarReading > t );//2
    bool error8  = (leftFarReading > t && leftNearReading > t && leftCenterReading > t && MiddleReading < t && rightCenterReading < t && rightNearReading < t && rightFarReading > t );//1
    bool error9  = (leftFarReading > t && leftNearReading > t && leftCenterReading < t && MiddleReading < t && rightCenterReading < t && rightNearReading < t && rightFarReading > t );//new
    bool error10 = (leftFarReading > t && leftNearReading > t && leftCenterReading > t && MiddleReading > t && rightCenterReading < t && rightNearReading < t && rightFarReading < t );//3
    bool error11 = (leftFarReading > t && leftNearReading > t && leftCenterReading > t && MiddleReading > t && rightCenterReading > t && rightNearReading < t && rightFarReading < t );//2
    bool error03 = (leftFarReading > t && leftNearReading < t && leftCenterReading < t && MiddleReading < t && rightCenterReading < t && rightNearReading < t && rightFarReading < t );
    bool error04 = (leftFarReading > t && leftNearReading > t && leftCenterReading < t && MiddleReading < t && rightCenterReading < t && rightNearReading < t && rightFarReading < t );// this one right turn
    bool error12 = (leftFarReading > t && leftNearReading > t && leftCenterReading > t && MiddleReading > t && rightCenterReading > t && rightNearReading > t && rightFarReading < t );//1

        if (error0) {
            Lposition = 000;
        }
        
        else if (error01) {
        // basespeed = 1;
         for (int i = 0; i < 10; i++) {
           Lposition = 100;
          }
        }
        
         else if (error02) {
        // basespeed = 1;
         for (int i = 0; i < 10; i++) {
           Lposition = 100;
          }
        }

        else if (error3) {
            Lposition = 100;
        }

        else if (error4) {
            Lposition = 200;
        }

        else if (error5) {
            Lposition = 300;
        }
        
        else if (error6) {
            Lposition = 350;
        }

        else if (error7) {
            Lposition = 400;
        }

        else if (error8) {
            Lposition = 450;
        }
        
        else if (error9) {
            Lposition = 500;
        }

        else if (error10) {
            Lposition = 600;
        }

        else if (error11) {
            Lposition = 700;
        }
 
        else if (error03) {
        // basespeed = 1;
         for (int i = 0; i < 100; i++) {
           Lposition = 700;
          }
        }
        
        else if (error04) {
        // basespeed = 1;
         for (int i = 0; i < 100; i++) {
           Lposition = 700;
          }
        }  
          
        else if (error12) {
            Lposition = 800;
        }
    
}

void computeLinePID() {

    Lerror = Lgoal - Lposition;
    
    // std::cout << "error      --     " << Lerror << std::endl;
    // std::cout << "position      --     " << position << std::endl;

    lP = Lerror;
    lI = lI + Lerror;
    lD = Lerror - LlastError;
    LlastError = Lerror;
    
    LmotorVal = lP*lKp + lD*lKd;

    LleftSpeed  = (basespeed - LmotorVal)*1.1;
    LrightSpeed = (basespeed + LmotorVal)*1.1;
    
     // std::cout << "P     --     " << P*Kp << std::endl;
     // std::cout << "I     --     " << I*Ki << std::endl;
     // std::cout << "D     --     " << D*Kd << std::endl;
     
     // std::cout << "right     --     " << leftSpeed << std::endl;
     // std::cout << "left      --     " << rightSpeed << std::endl;
  
      left_motor->setVelocity (boost::algorithm::clamp(LleftSpeed, 0, 15));
      right_motor->setVelocity(boost::algorithm::clamp(LrightSpeed, 0, 15));   
      
}

void computeWallPID() {

    if (SRM < 35 && SLM < 35) {
      got = true;
    }
    
    if (got) {
    already_ran = true;

    Werror =  (8.00 - SLF) + (4.00 - SLM);
     //std::cout<< "SLF : " << SLF <<std::endl;
     //std::cout<< "SLM : " << Werror <<std::endl;
    // std::cout<< "error : " << Werror <<std::endl;
    
    if (SLF > 30) {
       left_motor->setVelocity (4);
       right_motor->setVelocity(4); 
    }

    wP = Werror;
    wI = wI + Werror;
    wD = Werror - WlastError;
    WlastError = Werror;

    WmotorVal = wP*wKp + wI*wKi + wD*wKd;
    
   // std::cout<< "WmotorVal : " << WmotorVal <<std::endl;

    WleftSpeed  = (5 + (WmotorVal)*1.1);
    WrightSpeed = (5 - (WmotorVal)*1.1);

    left_motor->setVelocity (boost::algorithm::clamp(WleftSpeed, 0, 15));
    right_motor->setVelocity(boost::algorithm::clamp(WrightSpeed, 0, 15)); 

    }
}

void Decision() {

  sum = right_sum + left_sum;

  if ((already_ran) && (leftFarReading < t && leftNearReading < t && leftCenterReading < t && MiddleReading < t 
        && rightCenterReading < t && rightNearReading < t && rightFarReading < t && Left > t1 && Middle > t1 && Right > t1)) {
        
        if (sum % 2 == 0) {
        already_ran8 = true;
        // cout<<"left"<<endl;
        for (int i = 0; i < 1000; i++) {
          left_motor->setVelocity (-10);
          right_motor->setVelocity(10);
         } 
        }
        
        else if (sum % 2 != 0){
        already_ran9 = true;
        // cout<<"right"<<endl;
        for (int i = 0; i < 1000; i++) {
          left_motor->setVelocity (10);
          right_motor->setVelocity(-10);
          }
        }  
     }   
          if (!(already_ran10) && (already_ran9) && (leftFarReading < t && leftNearReading < t && leftCenterReading < t && MiddleReading < t 
              && rightCenterReading < t && rightNearReading < t && rightFarReading < t && Left < t1 && Middle < t1 && Right < t1)) {
              already_ran10 = true;
              cout << AnsiCodes::RED_FOREGROUND << "Red" << AnsiCodes::RESET << ":" << sum << endl;
              
          }
         
          if (!(already_ran11) && (already_ran8) && (leftFarReading > t && leftNearReading > t && leftCenterReading > t && MiddleReading > t 
              && rightCenterReading > t && rightNearReading > t && rightFarReading > t && Left > t1 && Middle > t1 && Right > t1)) {
              already_ran11 = true;
              cout << AnsiCodes::BLUE_FOREGROUND << "Blue" << AnsiCodes::RESET << ":" << sum << endl;
          }
               
}   

void countPillar() {

    rSRM = round(SRM);
    rSLM = round(SLM);

    bool left  = (leftFarReading > t && leftNearReading > t && leftCenterReading < t && MiddleReading < t && rightCenterReading < t && rightNearReading < t && rightFarReading < t && Left > t1 && Middle < t1 && Right < t1);
    bool right = (leftFarReading < t && leftNearReading < t && leftCenterReading < t && MiddleReading < t && rightCenterReading < t && rightNearReading > t && rightFarReading > t && Left < t1 && Middle < t1 && Right > t1);
    bool white = (leftFarReading < t && leftNearReading < t && leftCenterReading < t && MiddleReading < t && rightCenterReading < t && rightNearReading < t && rightFarReading < t && Left < t1 && Middle < t1 && Right < t1);
    bool down  = (rSLM < 35 || rSRM < 35);
    bool down1 = (rSLM < 35);
    bool down2 = (rSRM < 35);
      
      if ((right) && (down)) {

        std::cout<< "Pillar at right side – " << rSRM+3 <<std::endl;
        right_sum1 = right_sum + (rSRM+3) * 5;

      }
      
      if ((left) && (down)) {

        std::cout<< "Pillar at left side – "  << rSLM+3 <<std::endl;
        left_sum1  = left_sum + (rSLM+3) * 3;

      }
      
      if (!(already_ran4) && (white) && (down1) && (down2)) {
          already_ran4 = true;
          left_sum2  =+ (rSLM+3) * 3;
          right_sum2 =+ (rSRM+3) * 5;
        
        if (rSLM < rSRM) {
            already_ran5 = true;
            std::cout<< "Left side pillar is the closest"  <<std::endl;
        }
        else {
            already_ran6 = true;
            std::cout<< "Right side pillar is the closest"  <<std::endl;
        }
      }
      
      right_sum = right_sum1 + right_sum2;
      left_sum  = left_sum1  + left_sum2;
      
      // cout<< "right " << right_sum+right_sum1 <<endl;
      // cout<< "left "  << left_sum+left_sum1   <<endl;

        if ((n!=2) && (already_ran5) && (leftFarReading < t && leftNearReading < t && leftCenterReading < t && MiddleReading < t 
             && rightCenterReading < t && rightNearReading < t && rightFarReading < t && Left > t1 && Middle > t1 && Right > t1)) {

                //already_ran1 = true;
                //for (int i = 0; i < 1000; i++) {
                left_motor->setVelocity (-15);
                right_motor->setVelocity(15);
                n = n+1;
                //cout<<n<<endl;
                //}
                  //if (n == 1000){
                 // cout<<"done"<<endl;
                    //already_ran1 = true;
                    //}
        }
            
        if ((n1!=2) && (already_ran6) && (leftFarReading < t && leftNearReading < t && leftCenterReading < t && MiddleReading < t 
             && rightCenterReading < t && rightNearReading < t && rightFarReading < t && Left > t1 && Middle > t1 && Right > t1)) {

                //already_ran7 = true;
                //for (int i = 0; i < 1000; i++) {
                left_motor->setVelocity (15);
                right_motor->setVelocity(-15);
                n1 = n1+1;
                //cout<<n1<<endl;
                //}
                //if (n1 == 1000){
                 // cout<<"done"<<endl;
                    //already_ran7 = true;
                    //}
        }    
        // std::cout<< "Pillar at left side – "  << whiteleft <<std::endl;
        // std::cout<< "Pillar at right side – "  << whiteright <<std::endl;
    
} 

void inWhitedo() {

  if (!(already_ran2)) {
    already_ran2 = true;
    inWhite = inWhite.append(outRange);
    
  }

    if (leftFarReading < t && leftNearReading < t && leftCenterReading < t && MiddleReading < t 
        && rightCenterReading < t && rightNearReading < t && rightFarReading < t && Left < t1 && Middle < t1 && Right < t1) {

        inWhite = inWhite.append(inRange);
      }
            
    else {
        
        inWhite = inWhite.append(outRange);
     }
         
    inWhite.erase(std::unique(inWhite.begin(), inWhite.end()), inWhite.end());
    
     if (inWhite == "01") {
        inWhite1 = inWhite1.append(inWhite);
        inWhite.erase();
     }
    
     else if (inWhite == "10") {
        inWhite1 = inWhite1.append(inWhite);
        inWhite.erase();
     }
    
     if (inWhite == "01") {
        inWhite1 = inWhite1.append(inWhite);
        inWhite.erase();
     }
    
     else if (inWhite == "10") {
        inWhite1 = inWhite1.append(inWhite);
        inWhite.erase();
     }
    

    //std::cout<< "inWhite1  " << inWhite1 <<std::endl;   
}

void EMItalk_Stop() {

  if (already_ran10) {  
    EMI->send(mesgRed.c_str(), (int)strlen(mesgRed.c_str()) + 1);
    left_motor-> setVelocity(0);
    right_motor->setVelocity(0);
  }
  
  else if (already_ran11) {
    EMI->send(mesgBlue.c_str(), (int)strlen(mesgBlue.c_str()) + 1);
    left_motor-> setVelocity(0);
    right_motor->setVelocity(0);
  } 

}

int main(int argc, char **argv) {

 initGS();
 initgs();
 initDS();
 initMotor();
 
 cout << AnsiCodes::CLEAR_SCREEN <<endl;
 
   while (robot -> step(TIME_STEP) != -1) {
   

     left_motor-> setVelocity(0);
     right_motor->setVelocity(0);
                      
     readGSensors();
     readDSensors();
          
     starT = robot->getTime();
     
     if (starT < 0.1) {
     
       readGSensors();
        
       left_motor-> setVelocity(0);
       right_motor->setVelocity(0);
     }
     
     else {
     
     string pattern = "0110";
     inWhitedo();
     //cout<< inWhite1 <<endl;
     if (!(already_ran3) && inWhite1 == pattern) {
       already_ran3 = true;
       //cout<< inWhite1 <<endl;
       inWhite1.erase();  
     } 
     
     if ((already_ran3) && inWhite1 == "01")   {
       countPillar();
       
     }

        if (leftFarReading < t && leftNearReading < t && leftCenterReading < t && MiddleReading < t 
            && rightCenterReading < t && rightNearReading < t && rightFarReading < t && Left < t1 && Middle < t1 && Right < t1) {
           
             straight();
             
         }
         
        else if (leftFarReading < t && leftNearReading < t && leftCenterReading < t && MiddleReading < t 
            && rightCenterReading < t && rightNearReading < t && rightFarReading < t && Left > t1 && Middle < t1 && Right > t1) {
           
             straight();

         }
            
        else {

           computeLinePosition();
           computeLinePID();    
           countPillar(); 
           Decision();
           
           if (leftFarReading > t && leftNearReading > t && leftCenterReading > t && MiddleReading > t 
              && rightCenterReading > t && rightNearReading > t && rightFarReading > t && Left > t1 && Middle > t1 && Right > t1) {

                   readDSensors();
                   computeWallPID();           
            } 
          }
      }

    Decision();
    
    if (already_ran10 || already_ran11) {
      
      if (!(already_ran12)) {
        already_ran12 = true;
        Time = starT;
      }

      starT = starT - Time;
      //cout<<"S"<<starT<<endl;
      //cout<<"T"<<Time<<endl;

      if (starT <= 0.25) {
        left_motor-> setVelocity(10);
        right_motor->setVelocity(10);
      }
      else {
        EMItalk_Stop();
        left_motor-> setVelocity(0);
        right_motor->setVelocity(0);
        break;
      }
    }

};

  delete robot;
  return 0;
}
