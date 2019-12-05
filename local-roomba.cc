/**
 * local-roomba.cc
 * 
 * Sample code for a roomba-like robot that has two front bumpers and
 * magically knows where it is. 
 *
 * Written by: Simon Parsons
 * Date:       24th October 2011
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
void printRobotData(BumperProxy& bp, player_pose2d_t pose);
int indexOfClosest(double, double, double[11][2]);
/**
 * main()
 *
 **/

int main(int argc, char *argv[])
{  
  // Mapping of graph nodes
  double coords[11][2] = {{-6,-6},{1,-5},{3.7,-7.3},{-6.5,-2},{-7,5.5},{-5,7},{-4,5.5},{5,5.5},{5,0},{5,3.5},{1.5,-7.8}};
  int map[11] = {9,9,9,1,6,6,7,8,9,-1,2};
  // Variables
  int counter = 0;
  int nextIDX = 0;
  int started = 1;
  int finding_angle = 0;
  int curr_coord = 0, next_coord;
  double speed;            // How fast do we want the robot to go forwards?
  double turnrate;         // How fast do we want the robot to turn?
  double curr_x, curr_y, curr_a;
  double targ_x, targ_y, targ_a;
  double dist_away, dx, dy;
  player_pose2d_t  pose;   // For handling localization data

  // Set up proxies. These are the names we will use to connect to 
  // the interface to the robot.
  PlayerClient    robot("localhost");  
  BumperProxy     bp(&robot,0);  
  Position2dProxy pp(&robot,0);
  LocalizeProxy   lp (&robot, 0);

  // Allow the program to take charge of the motors (take care now)
  pp.SetMotorEnable(true);

  // Main control loop
  while(true) 
    {    
      // Update information from the robot.
      robot.Read();
      // Read new information about position
      pose = readPosition(lp);

      // Print data on the robot to the terminal
      // printRobotData(bp, pose);

      // This part of the code should be very familiar by now.
      //
      // If either bumper is pressed, stop. Otherwise just go forwards
      
      if (curr_coord == 9) {
         pp.SetSpeed(0, 0);
         break;
      }

      curr_x = pose.px;
      curr_y = pose.py;
      curr_a = pose.pa;
      // if (curr_a < 0) curr_a = 2*M_PI + curr_a;
      std::cout << "X: " << curr_x << std::endl;
      std::cout << "Y: " << curr_y << std::endl;
      std::cout << "A: " << rtod(curr_a) << std::endl;

      if (bp[0] || bp[1] || started) {
        if (started) {
          next_coord = indexOfClosest(curr_x, curr_y, coords);
          started = 0;
        } else {
          next_coord = map[curr_coord];
        }
        targ_x = coords[next_coord][0];
        targ_y = coords[next_coord][1];
        targ_a = atan2(targ_y-curr_y, targ_y-curr_y);           
        finding_angle = 1;
        speed = 0;
        turnrate = 0.4;
      } else if (finding_angle) {
        if (abs(rtod(targ_a)-rtod(curr_a)) < 5) {
          turnrate = 0;
          speed = 1.0;
          finding_angle = 0;
        } else {
          turnrate = 0.4;
          speed = 0;
        }
      } else {
        speed = 1.0;
        turnrate = 0.0;
      }

      dx = curr_x-targ_x;
      dy = curr_y-targ_y;
      dist_away = sqrt(dx*dx+dy*dy);
      
      if (dist_away < 0.01) {
        started = 1;
        curr_coord = next_coord;
        speed = 0.0;
      }

      std::cout << "TX: " << targ_x << std::endl;
      std::cout << "TY: " << targ_y << std::endl;
      std::cout << "TA: " << rtod(targ_a) << std::endl;
      // What are we doing?
      //std::cout << "Speed: " << speed << std::endl;      
      //std::cout << "Turn rate: " << turnrate << std::endl << std::endl;

      // Send the commands to the robot
      pp.SetSpeed(speed, turnrate);  
      // Count how many times we do this
      counter++;
    }
  
} // end of main()


/**
 * readPosition()
 *
 * Read the position of the robot from the localization proxy. 
 *
 * The localization proxy gives us a hypothesis, and from that we extract
 * the mean, which is a pose. 
 *
 **/

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

player_pose2d_t readPosition(LocalizeProxy& lp)
{

  player_localize_hypoth_t hypothesis;
  player_pose2d_t          pose;
  uint32_t                 hCount;

  // Need some messing around to avoid a crash when the proxy is
  // starting up.

  hCount = lp.GetHypothCount();

  if(hCount > 0){
    hypothesis = lp.GetHypoth(0);
    pose       = hypothesis.mean;
  }

  return pose;
} // End of readPosition()


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

  // Print out where we are
  std::cout << "We are at" << std::endl;
  std::cout << "X: " << pose.px << std::endl;
  std::cout << "Y: " << pose.py << std::endl;
  std::cout << "A: " << pose.pa << std::endl;

  
} // End of printRobotData()
