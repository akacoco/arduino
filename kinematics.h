#ifndef _Kinematics
#define _Kinematics_h

#define PI 3.1415926535897932384626433832795

//You may want to use some/all of these variables
const float WHEEL_DIAMETER              = 70.0f; // Diameter in mm.
const float WHEEL_RADIUS                = 35.0f; // Radius in mm.
const float WHEEL_SEPERATION            = 140.0f; // Distance between wheels.

const float GEAR_RATIO                  = 1.0f/120.0f; // Gearbox --- 1 : 120.
const float COUNTS_PER_SHAFT_REVOLUTION = 12.0f; // 1 Shaft Revolution = 12 counts
const float COUNTS_PER_WHEEL_REVOLUTION = 1440.0f; // 1 Wheel Revolution = 1440 counts.
const float COUNTS_PER_MM               = (1 / (WHEEL_DIAMETER * PI)) * COUNTS_PER_WHEEL_REVOLUTION; 


// Build up your Kinematics class.
class Kinematics
{
  public:
    
    Kinematics();   // Constructor, required.
    void setPose( float _x, float _y, float _theta );
    void update(long count_left, long count_right);  // should calucate an update to pose.
float x;
    float y; 
    float theta_;
    float get_theta();
    float get_xpos();
    float get_ypos();
    
    void reset_theta();

    
  private:
    
    //Private variables and methods go here
    float xpos;
    float ypos;
    float theta;

    long old_count_left;
    long old_count_right;
    
};


// Required constructor. Initialise variables.
Kinematics::Kinematics() // works as the setup();
{
  xpos = 0;
  ypos = 0;
  theta = 0;

  old_count_left = 0;
  old_count_right = 0;
}


void Kinematics ::setPose( float _x, float _y, float _theta ) {
  x     = _x;
  y     = _y;
  theta_ = _theta;}
  
void Kinematics :: update(long count_left, long count_right) 
{
  // count the change of count of both wheels:
  long left_count_change = count_left - old_count_left;
  long right_count_change = count_right - old_count_right;

  // calculate the distance (mm) of two wheels:
  float left_distance = (float)left_count_change / COUNTS_PER_MM;
  float right_distance = (float)right_count_change / COUNTS_PER_MM;

  // use the mean change in encoder count between the two wheels.
  // avg_distance
  float d = (left_distance + right_distance) / 2.0f;

  // from the view of the map: going right, x++; going up, y--;
  xpos = xpos + d * cos(theta*180/PI); 
  ypos = ypos + d * sin(theta*180/PI); 

  // turn right / rot --- theta ++ ; turn left / anti-rot --- theta -- ;
  theta = theta + (left_distance - right_distance) / WHEEL_SEPERATION; // unit rad.


  /*while( theta < 0 ) theta += 2*PI;
  while( theta > 2*PI ) theta -= 2*PI;*/

  old_count_left = count_left;
  old_count_right = count_right;
  
  return;
}

float Kinematics :: get_theta() 
{
  return theta;
}

float Kinematics :: get_xpos() 
{
  return xpos;
}

float Kinematics :: get_ypos() 
{
  return ypos;
}


#endif
// end kinematics
