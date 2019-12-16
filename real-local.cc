/**
 * real-local.cc
 * 
 * Sample code for a robot that has two front bumpers and a laser scanner,
 * and. Suitable for use with the roomba.
 *
 * This version is intended to run with the AMCL localization proxy,
 * which provides multiple hypotheses.
 *
 * Written by: Simon Parsons
 * Date:       22nd November 2009
 *  
 **/


#include <iostream>
#include <libplayerc++/playerc++.h>
using namespace PlayerCc;  

/**
 * Function headers
 *
 **/

player_pose2d_t readPosition(LocalizeProxy& lp);
void printLaserData(LaserProxy& sp);
void printRobotData(BumperProxy& bp, player_pose2d_t pose);

/**
 * main()
 *
 **/

int main(int argc, char *argv[])
{  
  double coords[11][2] = {{-6,-6},{1,-5},{3.7,-7.3},{-6.5,-2},{-7,5.5},{-5,7},{-4,5.5},{5,5.5},{5,0},{5,-3.5},{1.5,-7.8}};
  int map[11] = {9,9,9,1,6,6,7,8,9,-1,2};
  // Variables
  int hc = 0;
  int counter = 0, main_counter = 0;
  int started = 1, arrived = 0, bumped = 0;
  int finding_angle = 0, traveling = 0;
  int curr_coord = 0, next_coord = 0;
  double curr_x, curr_y, curr_a;
  double targ_x=0, targ_y=0, targ_a=0;
  double angle_away, dist_away, dx, dy;
  double speed;            // How fast do we want the robot to go forwards?
  double turnrate;         // How fast do we want the robot to turn?
  player_pose2d_t  pose;   // For handling localization data
  player_laser_data laser; // For handling laser data

  // Set up proxies. These are the names we will use to connect to 
  // the interface to the robot.
  PlayerClient    robot("localhost");  
  BumperProxy     bp(&robot,0);  
  Position2dProxy pp(&robot,0);
  LocalizeProxy   lp (&robot, 0);
  LaserProxy      sp (&robot, 0);

  // Allow the program to take charge of the motors (take care now)
  pp.SetMotorEnable(true);

  // Main control loop
  while(true) 
    {    
      // Update information from the robot.
      robot.Read();
      // Read new information about position
      pose = readPosition(lp);
      // Print information about the laser. Check the counter first to stop
      // problems on startup
      if(counter > 2){
	printLaserData(sp);
      }

      // Print data on the robot to the terminal
      printRobotData(bp, pose);
      hc = lp.GetHypothCount();
      // If either bumper is pressed, stop. Otherwise just go forwards
      if (bumped) {
        if (counter < 50) {
          speed = -0.5;
          turnrate = 0.4;
        } else {
          counter = 0;
          bumped = 0;
          turnrate = 0.0;
          speed = 0.5;
        }
        counter ++;
      } else if(bp[0] || bp[1]){
	speed= 0;
	turnrate= 0;
        bumped = true;
      } else if (main_counter > 750 &&hc <= 2) {
        player_localize_hypoth hypo;
        player_pose2d_t pose;
       	double best = 0.0;
        int best_index = 0;
        for (int i = 0; i < hc; i++) {
          hypo = lp.GetHypoth(i);
          pose = hypo.mean;
          if (hypo.alpha > best) {
            best = hypo.alpha;
            best_index = i;
          } 
        }
        hypo = lp.GetHypoth(best_index);
        pose = hypo.mean;
        std::cout << "Best hypothesis...." << std::endl;
        std::cout << "X: " << pose.px << std::endl;        
        std::cout << "Y: " << pose.py << std::endl;        
        std::cout << "A: " << pose.pa << std::endl;        
        std::cout << "W: " << best << std::endl;        
        
        if (best > 0.99) {
          std::cout << "Success!!" << std::endl;
          pp.SetSpeed(0, 0);
          break;
        }
        counter = 200;
      } else {
        double totaldist = sp.MinLeft() + sp.MinRight(); 
        
        // If distance of LEFT+RIGHT is less than 2.1, we are close
        // to the wall, so swap turnrates and SLOW DOWN!
        // (This prevents circular motion at the goal zone) 
        if (totaldist > 0 && totaldist < 2.1) {
          speed = 0.5;
          if (sp.MinLeft() < sp.MinRight()) {
            turnrate = 0.8;
          } else {
            turnrate = -0.8;
          }
        // If there are no limiations, stay evenly in the middle
        // by turning TOWARDS the further wall and closing the gap.
        } else {
          speed = 1.0;
          if (sp.MinLeft() < sp.MinRight()) {
            turnrate = -0.4;
          } else {
            turnrate = 0.4;
          }
        }
      }     

      // What are we doing?
      std::cout << "Speed: " << speed << std::endl;      
      std::cout << "Turn rate: " << turnrate << std::endl << std::endl;
      std::cout << "Counter: " << main_counter << std::endl << std::endl;

      // Send the commands to the robot
      pp.SetSpeed(speed, turnrate);  
      // Count how many times we do this
      main_counter++;
    }
  
} // end of main()

int indexOfClosest(double x, double y, double coords[11][2]) {
  double minDist = 99999999, dist;
  double dx, dy;
  int idx = -1;
  for (int i = 0; i < 11; i++) {
    dx = x-coords[i][0];
    dy = y-coords[i][1];
    dist = sqrt(dx*dx+dy*dy);
    if (dist < minDist) {
      minDist = dist;
      idx = i;
    }
  } 
  return idx;
}

/**
 * readPosition()
 *
 * Read the position of the robot from the localization proxy. 
 *
 * The localization proxy gives us a set of "hypotheses", each of
 * which is a number of possible locations for the robot, and from
 * each we extract the mean, which is a pose.
 *
 * As the number of hypotheses drops, the robot should be more sure
 * of where it is.
 *
 **/

player_pose2d_t readPosition(LocalizeProxy& lp)
{

  player_localize_hypoth_t hypothesis;
  player_pose2d_t          pose;
  uint32_t                 hCount;
  double                   weight;

  // Need some messing around to avoid a crash when the proxy is
  // starting up.

  hCount = lp.GetHypothCount();

  std::cout << "AMCL gives us " << hCount + 1 
            << " possible locations:" << std::endl;

  if(hCount > 0){
    for(int i = 0; i <= hCount; i++){
      hypothesis = lp.GetHypoth(i);
      pose       = hypothesis.mean;
      weight     = hypothesis.alpha;
      std::cout << "X: " << pose.px << "\t";
      std::cout << "Y: " << pose.py << "\t";
      std::cout << "A: " << pose.pa << "\t";
      std::cout << "W: " << weight  << std::endl;
    }
  }

  // This just returns the mean of the last hypothesis, it isn't necessarily
  // the right one.

  return pose;
} // End of readPosition()

void printLaserData(LaserProxy& sp)
{

  double maxRange, minLeft, minRight, range, bearing;
  int points;

  maxRange  = sp.GetMaxRange();
  minLeft   = sp.MinLeft();
  minRight  = sp.MinRight();
  range     = sp.GetRange(5);
  bearing   = sp.GetBearing(5);
  points    = sp.GetCount();

  //Print out useful laser data
  std::cout << "Laser says..." << std::endl;
  std::cout << "Maximum distance I can see: " << maxRange << std::endl;
  std::cout << "Number of readings I return: " << points << std::endl;
  std::cout << "Closest thing on left: " << minLeft << std::endl;
  std::cout << "Closest thing on right: " << minRight << std::endl;
  std::cout << "Range of a single point: " << range << std::endl;
  std::cout << "Bearing of a single point: " << bearing << std::endl;

  return;
} // End of printLaserData()

/**
 *  printRobotData
 *
 * Print out data on the state of the bumpers and the current location
 * of the robot.
 *
 **/

void printRobotData(BumperProxy& bp, player_pose2d_t pose)
{

  // Print out what the bumpers tell us:
  std::cout << "Left  bumper: " << bp[0] << std::endl;
  std::cout << "Right bumper: " << bp[1] << std::endl;
  // Can also print the bumpers with:
  //std::cout << bp << std::endl;

  // Print out where we are
  std::cout << "We are at" << std::endl;
  std::cout << "X: " << pose.px << std::endl;
  std::cout << "Y: " << pose.py << std::endl;
  std::cout << "A: " << pose.pa << std::endl;

  
} // End of printRobotData()
