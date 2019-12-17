/*
 *  CISC-3415 Robotics
 *  Project 4 - Part 2
 *  Credit To: Simon Parsons
 *
 ** Group Members *************************************************************
 *    
 *  Benjamin Yi
 *  Emmanuel Desdunes
 *  Montasir Omi
 *  Shahzad Ahmad
 *
 ** Description ***************************************************************
 * 
 *  In this program, the simulated robot uses its laser functionality to first
 *  scan the map in order to determine its position. Once the robot has
 *  traveled some distance, if the robot has only two hypotheses of its
 *  location remaining, it will choose the best hypotheses, only if it is at
 *  least 99% sure, otherwise it will continue to scan. 
 */


#include <iostream>
#include <fstream>
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
  // Variables
  int counter = 0;
  int main_counter = 0;
  int bumped = 0;
  int hc = 0;
  double speed;            // How fast do we want the robot to go forwards?
  double turnrate;         // How fast do we want the robot to turn?
  player_pose2d_t  pose;   // For handling localization data
  player_laser_data laser; // For handling laser data
  std::ofstream ofs;
  ofs.open("log.txt");
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
          turnrate = -0.4;
        } else {
          counter = 0;
          bumped = 0;
          speed = 0.5;
          turnrate = 0.0;
        }
        counter ++;
      } else if(bp[0] || bp[1]){
	speed= 0;
	turnrate= 0;
        bumped = 1;
      // After 1000 iterations, and if hypoth count <= 2
      // Record the current best hypothesis
      } else if (main_counter > 1000 && hc <= 2) {
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
        std::cout << "Best hypothesis..." << std::endl;
        std::cout << "X: " << pose.px  << std::endl;
        std::cout << "Y: " << pose.py  << std::endl;
        std::cout << "A: " << pose.pa  << std::endl;
        std::cout << "W: " << best << std::endl;
        ofs << "Best hypothesis..." << std::endl;
        ofs << "X: " << pose.px  << std::endl;
        ofs << "Y: " << pose.py  << std::endl;
        ofs << "A: " << pose.pa  << std::endl;
        ofs << "W: " << best << std::endl;
        // If the best hypothesis is 99% certain, we've done a successful run.
        if (best > 0.99) {
          std::cout << "Success!" << std::endl;
          std::cout << "I am " << best << " sure that I am at ";
          std::cout << "(" << pose.px << ", " << pose.py << ")..." << std::endl;
          ofs << "Success!" << std::endl;
          ofs << "I am " << best*100 << "% sure that I am at ";
          ofs << "(" << pose.px << ", " << pose.py << ")..." << std::endl;
          pp.SetSpeed(0, 0);
          break;
        }
        main_counter = 200;
      } else {
        // Navigation adjustments using laser data
        speed = 1.0;
        if (sp.MinLeft() < 1.2) {
          turnrate = -0.8;
        } else if (sp.MinRight() < 1.2) {
          turnrate = 0.8;
        } else {
          if (sp.MinLeft() < sp.MinRight()) turnrate = -0.4;
          else turnrate = 0.4;
        }
      }     

      // What are we doing?
      std::cout << "Speed: " << speed << std::endl;      
      std::cout << "Turn rate: " << turnrate << std::endl;
      std::cout << "Counter: " << main_counter << std::endl << std::endl;
      ofs << "Speed: " << speed << std::endl;      
      ofs << "Turn rate: " << turnrate << std::endl;
      ofs << "Counter: " << main_counter << std::endl << std::endl;
      // Send the commands to the robot
      pp.SetSpeed(speed, turnrate);  
      // Count how many times we do this
      counter++;
      main_counter++;
    }
  ofs.close();
  
} // end of main()

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
