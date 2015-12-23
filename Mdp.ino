#include <Sensors.h>

#include <PololuWheelEncoders.h>
#include <DualVNH5019MotorShield.h>

const int DELAY_MOVEMENT = 500;

//Motor 1 Encoder Pins
//Yellow is encoder A, connected to arduino board pin 3
//White is encoder  B, connected to arduino board pin 5
const int m1_inA = 3;
const int m1_inB = 5;


//Motor 2 Encoder Pins
//Yellow is encoder A, connected to arduino board pin 11
//White is encoder  B, connected to arduino board pin 13
const int m2_inA = 11;
const int m2_inB = 13;

//Speed of motor
const int initialSpeed = 200;
const int slowSpeed = 70;

//Number of tick for a 10cm
int forwardGoalTicks, forwardGoalTicks_long;
int forwardSlowTicks;

int leftGoalTicks;
int leftSlowTicks;

int rightGoalTicks;
int rightSlowTicks;

//Calibration
const float calibrateLimit = 0.1;

// Sensor label
const int LEFT_SENSOR = 0;
const int FRONT_LEFT = 1;
const int FRONT_CENTER = 2;
const int FRONT_RIGHT = 3;
const int RIGHT_SENSOR = 4;



// Direction in order
const int WEST = 0, NORTH = 1, EAST = 2, SOUTH = 3;
const int dx[] = { -1, 0, 1, 0 };
const int dy[] = { 0, 1, 0, -1 };

// Relative direction
const int STRAIGHT = 0, LEFT = 1, RIGHT = 2;
 
// Maze map: 
// 0: unexplored
// 1: obstacle
// 2: clear
// 3: robot path
const int MAX_X = 15, MAX_Y = 20;
char maze[MAX_X][MAX_Y];
const short MAX_VALUE = 1000;
String shortest_ins = "";
bool goalArrived;

// Start position
const int START_X = 1;
const int START_Y = 1;
const int START_D = NORTH;

// End position
const int END_X = 13;
const int END_Y = 18;

// for calibrate
const float firstDistance = 7;
const int calibrateSpeed = 70;
int calibrate_shaking_step = 3;


// Current position and direction of robot
int x,y,d;

// Number of step robot has made
int step, step2;

// Rpi data store
char *inData;

//Motor Driver
DualVNH5019MotorShield md, md1, md2;

Sensors sensors;


void setup() {
  Serial.begin(9600);
  md.init(); 
  md1.init();
  md2.init();
  PololuWheelEncoders::init(m1_inA, m1_inB, m2_inA, m2_inB);

  /**
   * Hardware lab 2 maze 2nd battery
   */
//  forwardGoalTicks = 620;
//  forwardSlowTicks = forwardGoalTicks * 9 / 10;
//
//  rightGoalTicks = 848;
//  rightSlowTicks = rightGoalTicks * 9 / 10;
//
//  leftGoalTicks = 845;
//  leftSlowTicks = leftGoalTicks * 9 / 10;


  /**
   *  2nd battery 
   *  Speed 200
   */
  forwardGoalTicks = 580;
  forwardSlowTicks = forwardGoalTicks * 9 / 10;

  forwardGoalTicks_long = 598;

  rightGoalTicks = 795;
  rightSlowTicks = rightGoalTicks * 9 / 10;

  leftGoalTicks = 795;
  leftSlowTicks = leftGoalTicks * 9 / 10;

  /**
   * MONDAY 26-10
   */
//  forwardGoalTicks = 615;
//  forwardSlowTicks = forwardGoalTicks * 9 / 10;
//
//  forwardGoalTicks_long = 620;
//
//  rightGoalTicks = 837;
//  rightSlowTicks = rightGoalTicks * 9 / 10;
//
//  leftGoalTicks = 832;
//  leftSlowTicks = leftGoalTicks * 9 / 10;




}

bool testing = false;
void loop() {
  //Serial.println("Hello");
  //delay(1000);
  if (testing) {

    /****
     * Needed testing for function
     *   - try_calibration(); *
     *   - doExploration();
     *   
     *   - int getWallCell(int &left_sensor, int &right_sensor, int x, int y, int dir);
     *   - int getFreeCell(int x, int y, int dir)
     *   - int atCorner()
     *   - int try_calibration_ahead()
     *   - int try_calibration()
     *   - void try_alignt_distance(int cnt)
     *   - void do_calibration(int left_sensor, int right_sensor) 
     *   - void alight_distance(int left_sensor, int right_sensor)
     *   - void alight_distance(int sensor)
     */

    initExploration();
    
//    Serial.println(getSensorValue(FRONT_LEFT));  //  4.00
//    Serial.println(getSensorValue(FRONT_CENTER));// 5.4
//    Serial.println(getSensorValue(FRONT_RIGHT));  // 3.8
//    Serial.println(getSensorValue(LEFT_SENSOR));
//    Serial.println(getSensorValue(RIGHT_SENSOR));
//    Serial.println("");

   // doExploration();
      
    shortest_ins = "ffffffffffrfffffffflfffffffrffff";
   //   startShortestPath();

    //turnLeft();
    //moveRight();
    //forward();
    //updateFromSensor();
    
    //Serial.println(try_calibration_ahead());
    //try_alignt_distance(0);
    
    //do_calibration(FRONT_CENTER, FRONT_RIGHT);

    //moveLeft();
    //moveRight();
    goStraight(25);

  //prepareShortestPath();
         
    delay(100000);
    
  } else {
    String order = "FFFFFFFFFF";
    order = communicateWithRpi();
    if (order == "explore") {
      initExploration();
      doExploration();
      sendEnding();
      prepareShortestPath();
    } else if (order ==  "start") {
      startShortestPath();
    } else
      shortest_ins.concat(order);
  }


}

/**********************************************************
 * SHORTEST PATH PHASE
 *********************************************************/
bool canRobotLocated(int x, int y) {
  for (int i = -1; i <= 1; i++)
    for (int j = -1; j <= 1; j++) {
      int u = x + i, v = y + j;
      if (!inMaze(u, v) || maze[u][v] != 2) return false;
    }
  return true;
}

void prepareShortestPath() {
  turnToDir(SOUTH);
  do_calibration();
  turnToDir(WEST);
  do_calibration();
  turnToDir(NORTH);
}

const int MAX_STEP = 5;

void startShortestPath() {
  rightSlowTicks = rightGoalTicks;
  leftSlowTicks = leftGoalTicks;
  calibrate_shaking_step = 1;
  //Serial.println(shortest_ins);
  int i = 0;
  while ( i < shortest_ins.length()) {
    try_alignt_distance(0);
    char c = shortest_ins[i];
    if ( c != 'l' && c != 'r' && c != 'f' ) {
      i++;
      continue;
    }
    if (c == 'l') {
      turnLeft();
      i++;
      continue;
    } 
    if (c == 'r') {
      turnRight();
      i++;
      continue;
    }
    int j = i;
    while (j < shortest_ins.length() && shortest_ins[j] == c && j - i < MAX_STEP) j++;
    goStraight(j-i);
    i = j;
  }
}

/**********************************************************
 * EXPLORATION PHASE
 *********************************************************/
// Check of cell (u,v) is in maze
bool inMaze(int u, int v) {
  return 0 <= u && u < MAX_X && 0 <= v && v < MAX_Y;
}

bool inRobot(int u, int v) {
  return u >= x - 1 && u <= x + 1 && v >= y - 1 && v <= y + 1;
}

int dist(int u, int v, int x, int y) {
  return abs(u - x) + abs(v - y);
}

// Get a relative direction from given direction
int getDirRelative(int curD, int moveD) {
  switch(moveD) {
    case LEFT:
      return (curD + 3) % 4;
    case RIGHT:
      return (curD + 1) % 4;
  }
  return curD;
}

// Return true if block 3x3 center at (x,y) can put robot - in Maze and no obstacle
bool canRobotIn(int x, int y) {
  for (int i = -1; i <= 1; i++)
    for (int j = -1; j <= 1; j++) {
      int u = x + i , v = y + j;
      if (!inMaze(u, v) || maze[u][v] == 1 ) 
        return false;
    }
  return true;
}

// Return true if block 3x3 center at (x,y) is all explored - no unexplored block
bool fullyExploration(int x, int y) {
  for (int i = -1; i <= 1; i++) 
    for (int j = -1; j <= 1; j++) {
      int u = x + i, v = y + j;
      if (inMaze(u, v) && maze[u][v] == 0) return false;
    }
  return true;
}

/**
 * MOST IMPORTANT
 */
// Turn robot into specific direction
void turnToDir(int dir) {
  int diff = (dir + 4 - d) % 4;
  if (diff <= 2) {
    for (int i = 0; i < diff; i++) { 
      turnRight();
    }
  } else {
    turnLeft();
  }
}

/**
 * MOST IMPORTANT
 */
// Turn tobot into specific direction and go ahead one step
void goWithDir(int dir) {
  turnToDir(dir);
  goStraight();
}

/**
 * MOST IMPORTANT
 */
// If this function return true, robot make a turn already, just move it ahead
bool canGo(int dir) {
  int u = x + dx[dir], v = y + dy[dir];
  if (!canRobotIn(u,v)) return false;
  if (!fullyExploration(u, v)) {
    int currentDir = d;
    turnToDir(dir);
    updateFromSensor();
    bool res = canRobotIn(u, v);
    if (!res) {
      turnToDir( currentDir );
    }
    return res;
  } else return true;
}

void initExploration() {
  for (int i = 0; i < MAX_X; i++) 
    for (int j = 0; j < MAX_Y; j++)
       maze[i][j] = 0;
  x = START_X;
  y = START_Y;
  d = START_D;
  for (int i = -1; i <= 1; i++) 
    for (int j = -1; j <= 1; j++)
      maze[x+i][y+j] = 2;

  rightSlowTicks = rightGoalTicks * 9 / 10;
  leftSlowTicks = leftGoalTicks * 9 / 10;
  calibrate_shaking_step = 2;
}

/**
 * MOST IMPORTANT
 */
void doExploration() {
  step = 0;
  step2 = 0;
  while (!atStartPoint() || !goalArrived) {

    if (x == END_X && y == END_Y) goalArrived = true;
    updateFromSensor();
    try_calibration(4);
    try_alignt_distance(5);
    
    int left = getDirRelative(d, LEFT);
    int right = getDirRelative(d, RIGHT);
    if (canGo(left)) { 
      goWithDir(left);
      continue;
    }
    
    if (canGo(d)) {
      goWithDir(d);
      continue;
    }

    turnToDir(right);
  }

}

bool atStartPoint() {
  return x == START_X && y == START_Y;
}

/**
 * MOST IMPORTANT
 */
// Read data from sensor and update our map
void updateFromSensor() {
  char sensor[10];
  getSensorInBlock(sensor);
  sendSensorResultRpi(sensor);
  updateMaze(sensor);
}

const int SENSOR_FAR = 2;

// x, y , d


void updateMaze(char* sensor) {
  int u, v, dir;
  
  // LEFT_SENSOR
  updateMazeInternal(x + dx[d], y + dy[d], getDirRelative(d, LEFT), sensor[0] - '0' + 2, 2 + SENSOR_FAR);

  // FRONT_LEFT
  dir = getDirRelative(d, LEFT);
  updateMazeInternal(x + dx[dir], y + dy[dir], d, sensor[1] - '0' + 2, 2 + SENSOR_FAR);

  // FRONT_CENTER
  updateMazeInternal(x, y, d, sensor[2] - '0' + 1, 1 + SENSOR_FAR);

  // FRONT_RIGHT
  dir = getDirRelative(d, RIGHT);
  updateMazeInternal(x + dx[dir], y + dy[dir], d, sensor[3] - '0' + 2, 2 + SENSOR_FAR);

  // RIGHT_SENSOR
  updateMazeInternal(x + dx[d], y + dy[d], getDirRelative(d, RIGHT), sensor[4] - '0' + 2, 2 + SENSOR_FAR);

}

void updateMazeInternal(int u, int v, int dir, int obs, int far) {
  while (inMaze(u, v) && dist(u, v, x, y) <= far) {
    if (inRobot(u, v)) {
      u += dx[dir];
      v += dy[dir];
      continue;
    }
    if (dist(u, v, x, y) == obs) {
      maze[u][v] = 1;
      break;
    }

      maze[u][v] = 2;
    u += dx[dir];
    v += dy[dir];
  }
}

/********************
 * CALIBRATE PHASE
 ******************/

bool obstacle(int u,int v) {
  return !inMaze(u, v) || maze[u][v] == 1;
}

bool Free(int u, int v) {
  return inMaze(u, v) && maze[u][v] == 2;
}

// If 3 block ahead is wall or obstacle
bool fullWallAhead(int x, int y, int dir) {
  int left = getDirRelative(dir, LEFT);
  int right = getDirRelative(dir, RIGHT);
  int u = x + 2 * dx[dir];
  int v = y + 2 * dy[dir];
  return obstacle(u, v) && obstacle(u + dx[left], v + dy[left]) && obstacle( u + dx[right], v + dy[right] );
}

bool atCorner() {
  int any1, any2;
  if ( getWallCell(any1, any2, x, y, d) < 2 ) return false;
  int left = getDirRelative(d, LEFT);
  int right = getDirRelative(d, RIGHT);
  return getWallCell(any1, any2, x, y, left) >= 2 || getWallCell(any1, any2, x, y , right) >= 2;
}

int getFreeCell(int x, int y, int dir) {
  int left = getDirRelative(dir, LEFT);
  int right = getDirRelative(dir, RIGHT);
  int u = x + 2 * dx[dir];
  int v = y + 2 * dy[dir];
  return Free(u, v) + Free(u + dx[left], v + dy[left]) + Free( u + dx[right], v + dy[right] );
}

int getWallCell(int &left_sensor, int &right_sensor, int x, int y, int dir) {
  int left = getDirRelative(dir, LEFT);
  int right = getDirRelative(dir, RIGHT);
  int u = x + 2 * dx[dir];
  int v = y + 2 * dy[dir];
  int cnt = 0;
  for(int i = FRONT_LEFT; i <= FRONT_RIGHT; i++) {
    int uu = u, vv = v;
    switch(i) {
      case FRONT_LEFT: 
        uu = u + dx[left];
        vv = v + dy[left];
        break;
      case FRONT_RIGHT:
        uu = u + dx[right];
        vv = v + dy[right];
        break;
    }
    if (obstacle(uu, vv)) {
      cnt++;
      if (left_sensor == -1) left_sensor = i;
      else right_sensor = i;
    }
  }
  return cnt;
}

/**
 * MOST IMPORTANT
 */
bool atGoal() {
  return x == END_X && y == END_Y;
}

int try_calibration_ahead() {
  updateFromSensor();
  if (fullWallAhead(x, y, d)) {
    do_calibration();
    return true;
  } 
  int left_sensor = -1;
  int right_sensor = -1;
  if (getWallCell(left_sensor, right_sensor, x, y, d) == 2) {
    do_calibration(left_sensor, right_sensor);    
    return true;
  }
  return false;
}

// Try to calibrate robot to correct position
void try_calibration(int cnt) {
  if (step >= cnt || (atCorner() && step >= 3) || (atCorner() && atGoal() && step > 0) )  {
    int left = getDirRelative(d, LEFT);
    int right = getDirRelative(d, RIGHT);
    int leftOrRight = false;
    int currentDir = d;
    
    if (getFreeCell(x, y, left) <= 1 && !leftOrRight) {
      turnToDir(left);
      leftOrRight |= try_calibration_ahead();
      turnToDir(currentDir);
      delay(100);
    }
    if (getFreeCell(x, y, left) <= 1 && !leftOrRight) {
      turnToDir(right);
      leftOrRight |= try_calibration_ahead();
      turnToDir(currentDir);
      delay(100);
    }
    
    try_calibration_ahead();
    
    delay(100);
  }
}

void try_alignt_distance(int cnt) {
  if (step2 >= cnt) {
    int left_sensor = -1;
    int right_sensor = -1;
    if (getWallCell(left_sensor, right_sensor, x, y, d) != 0) {
      if (right_sensor == FRONT_CENTER) {
        alight_distance(FRONT_CENTER);
        step2 = 0;
      } else if (left_sensor != -1) 
        alight_distance(left_sensor);
        step2 = 0;
    }
  }
}




/**********************************************************
 * LOGICAL MOVEMENT OF ROBOT
 * - Call physical move
 * - update position
 *********************************************************/
void turnLeft() {
  d = (d + 3) % 4;
  sendMoveRpi(LEFT);
  moveLeft();
}

void turnRight() {
  d = (d + 1) % 4;
  sendMoveRpi(RIGHT);
  moveRight();
}

void goStraight() {
  x += dx[d];
  y += dy[d];
  step2++;
  step++;
  sendMoveRpi(STRAIGHT);
  forward();
}

void goStraight(int step) {
  x += dx[d] * step;
  y += dy[d] * step;
  forward(step);
}

/**********************************************************
 * PHYSICAL MOVEMENT OF ROBOT
 *********************************************************/



const int calibrateValue = 0.2;
const float frontCenterDistance = 4.6;
const float frontRightOffset = 0.0;
const float distanceAlign = 7;

const float FRONT_LEFT_DISTANCE = 4.3; // index 1
const float FRONT_CENTER_DISTANCE = 4.5; // index 2
const float FRONT_RIGHT_DISTANCE = 4.3; // index 3

const float SENSOR_DISTANCE[] = { 0 , FRONT_LEFT_DISTANCE, FRONT_CENTER_DISTANCE, FRONT_RIGHT_DISTANCE, 0 };

// Apply two sensor to align distance
void alight_distance(int left_sensor, int right_sensor) {
  for (int i = 0; i < 50; i++) {
      float distance_left = getSensorValue(left_sensor) - SENSOR_DISTANCE[left_sensor];
      float distance_right = getSensorValue(right_sensor) - SENSOR_DISTANCE[right_sensor];

      if (abs(distance_left - distance_right) <= calibrateValue) 
        break;
        
      if (distance_left > distance_right) { // tiled toward left
        md.setSpeeds(-calibrateSpeed, calibrateSpeed);
      }
      else {
        md.setSpeeds(calibrateSpeed, -calibrateSpeed);
      } 
    }
  md.setBrakes(400, 400);
}

// Apply which sensor to align distance
void alight_distance(int sensor) {
  for (int i = 0; i < 200; i++) {
    int distance = getSensorValue(sensor);
    int expected = SENSOR_DISTANCE[sensor];
    if (distance > 10) break;
    if (abs(distance - expected) < 0.2) {
      break;
    }
    if (distance > expected) {
      md.setSpeeds(calibrateSpeed, calibrateSpeed);
    }
    else {
      md.setSpeeds(-calibrateSpeed, -calibrateSpeed);
    }
  }
  md.setBrakes(400, 400);
}


void do_calibration(int left_sensor, int right_sensor) {
  int sensor = (left_sensor == FRONT_CENTER) ? left_sensor : right_sensor;
  for (int k = 0; k < calibrate_shaking_step; k++) {
    alight_distance(sensor);
    delay(50);
    alight_distance(left_sensor, right_sensor);
    delay(50);
  }
  delay(DELAY_MOVEMENT);
  step = 0;
}

void do_calibration() {
  alight_distance(FRONT_CENTER);
  for (int k = 0; k < calibrate_shaking_step; k++) {
    alight_distance(FRONT_LEFT, FRONT_RIGHT);
    delay(50);
    alight_distance(FRONT_CENTER);
    delay(50);
  }
  delay(DELAY_MOVEMENT);
  step = 0;
}


/**********************************************************
 * SENSOR FUNCTION
 *********************************************************/
float getSensorValue(int sensorLabel) {
  switch(sensorLabel) {
    case FRONT_LEFT: 
      return sensors.getDistance(3);
    case FRONT_CENTER: 
      return sensors.getDistance(2);
    case FRONT_RIGHT: 
      return sensors.getDistance(1);
    case LEFT_SENSOR: 
      return sensors.getDistance(4);
    case RIGHT_SENSOR: 
      return sensors.getDistance(6);
  }
}

const int valueObs2 = 2;

int hasObstacle(int sensorLabel) {
  float dist = getSensorValue(sensorLabel);
  switch(sensorLabel) {
    case FRONT_LEFT:
      if (dist > 0 && dist < 9.5) return 1;
      else if (dist > 0 && dist < 16) return valueObs2;
      else return 0;
    case FRONT_RIGHT:
      if (dist > 0 && dist < 9) return 1;
      else if (dist > 0 && dist < 17) return valueObs2;
      else return 0;
    case FRONT_CENTER: 
      if (dist > 0 && dist < 9.5) return 1;
      else if (dist > 0 && dist < 18) return valueObs2;
      else return 0;
    case LEFT_SENSOR: 
      if (dist > 0 && dist < 12) return 1;
      else if (dist > 0 && dist < 20) return valueObs2;
      else return 0;
    case RIGHT_SENSOR: 
      if ( dist > 0 && dist < 17) return 1;
      else if (dist > 0 && dist < 27) return valueObs2;
      else return 0;  
  }
}

void getSensorInBlock(char *res) {
  res[0] = ('0' + hasObstacle(LEFT_SENSOR));
  res[1] = ('0' + hasObstacle(FRONT_LEFT));
  res[2] = ('0' + hasObstacle(FRONT_CENTER));
  res[3] = ('0' + hasObstacle(FRONT_RIGHT));
  res[4] = ('0' + hasObstacle(RIGHT_SENSOR));
  res[5] = '\0';
}



/**********************************************************
 * RPI INTERACTION
 *********************************************************/

String communicateWithRpi() {
  //Rpi bluetooth to arduino
  int index = 0;
  String incomingMsg = "";
  bool dd = true;
  while (dd) {
    if (Serial.available())
    {

      char tempMsg = Serial.read();

      if (tempMsg != '\n')
      {
        //tempMsg++; //a change to b
        incomingMsg += tempMsg;
      }
      else
      {
        dd = false;
        break;
      }
    }
  }
  return incomingMsg;
}

char* getRpiMsg() {
  memset(inData, 0, sizeof(inData));
  Serial.readBytes(inData, 100);
  return inData;
}

void sendMaze() {
  String res = "aMAZE:";
  for (int i = 0; i < MAX_X; i++)
    for (int j = 0; j < MAX_Y; j++)
      res.concat('0' + maze[i][j]);
  Serial.println(res);
}

void sendEnding() {
 Serial.println("aSTATUS:ready");
}

void sendSensorResultRpi(char* sensor) {
  String res = "aSENSOR:";
  for (int i = 0; i <= RIGHT_SENSOR; i++) 
    res.concat(sensor[i]);
  Serial.println(res);
}

void sendMoveRpi(int move) {
  String res;
  switch(move) {
    case LEFT:
      res = "aSTATUS:LEFT";
      break;
    case RIGHT:
      res = "aSTATUS:RIGHT";
      break;
    default:
      res = "aSTATUS:FORWARD";
  }
  Serial.println(res);
}

void send_current_maze() {
  for(int j = MAX_Y-1; j >= 0; j--) {
    String res = "MAZE:";
    for(int i = 0; i < MAX_X; i++)
      res.concat( (char) (maze[i][j] + '0') );
    Serial.println(res);
  }
}

/**********************************************************
 * PRIVATE HELPER FUNCTION
 *********************************************************/
void resetCount() {
  PololuWheelEncoders::getCountsAndResetM1();
  PololuWheelEncoders::getCountsAndResetM2();
}

void getCount(int &m1, int &m2) {
  m1 = abs(PololuWheelEncoders::getCountsM1());
  m2 = abs(PololuWheelEncoders::getCountsM2()); 
}

void setSpeeds(int m1Speed, int m2Speed) {
  md1.setM1Speed(m1Speed);
  md2.setM2Speed(m2Speed);
}

void setBrakes(int m1Brake, int m2Brake) {
  md1.setM1Brake(m1Brake);
  md2.setM2Brake(m2Brake);
}

void setM1Brake(int m1Brake) {
  md1.setM1Brake(m1Brake);
}

void setM2Brake(int m2Brake) {
  md2.setM2Brake(m2Brake);
}


/**********************************************************
 * ROBOT MOVEMENT IMPLEMENTATION
 *********************************************************/

 void forward(int step) {
  moveForwardInTick2( (forwardGoalTicks_long) * step, (forwardGoalTicks - 20) * step ,(long) 150000 * step);
}

// These function is for exploration
// Move forward 10cm
void forward() {
  moveForwardInTick(forwardGoalTicks , forwardSlowTicks);
}

const int forward_initialSpeed = 250;

void moveForwardInTick(int goalTick, int slowTick) {
  boolean okLeft = false;
  boolean okRight = false;
  int m1, m2;
  
  resetCount();
  
  setSpeeds(initialSpeed, initialSpeed - 7 );
//  setSpeeds(initialSpeed, initialSpeed - 7 );
  
  long cnt = 0;
  while (!okLeft || !okRight) {
    getCount(m1, m2); 
    cnt++;
    if (cnt >= 150000) {
      //md.setBrakes(400, 400);
      setBrakes(400, 400);
      break;
    }
    if (m1 > goalTick) {
      okLeft = true;
      //md.setM1Brake(400);
      setM1Brake(400);
    }
    if (m2 > goalTick) {
      okRight = true;
//      md.setM2Brake(400);
        setM2Brake(400);
    }   
    if (!okLeft && !okRight && (m1 > slowTick || m2 > slowTick)) {
      //md.setSpeeds(slowSpeed, slowSpeed - 4);
      setSpeeds(slowSpeed, slowSpeed );
    }
  }
  delay(DELAY_MOVEMENT);
}

const int long_initialSpeed = 250;

void moveForwardInTick2(int goalTick, int slowTick, long cntMax) {
  boolean okLeft = false;
  boolean okRight = false;
  int m1, m2;
  resetCount();
  md.setSpeeds(long_initialSpeed, long_initialSpeed - 7 );
  //md.setSpeeds(long_initialSpeed, long_initialSpeed - 7 );
  long cnt = 0;
  while (!okLeft || !okRight) {
    getCount(m1, m2); 
    cnt++;
    if (cnt >= cntMax) {
      md.setBrakes(400, 400);
      break;
    }
    if (m1 > goalTick) {
      okLeft = true;
      md.setM1Brake(400);
    }
    if (m2 > goalTick) {
      okRight = true;
      md.setM2Brake(400);
    }   
    if (!okLeft && !okRight && (m1 > slowTick || m2 > slowTick)) {
      md.setSpeeds(slowSpeed, slowSpeed );
    }
  }
  delay(DELAY_MOVEMENT);
}

void moveLeft() {
  bool okLeft = false;
  bool okRight = false;
  int m1, m2;
  resetCount();
  //md.setSpeeds(initialSpeed, -initialSpeed );
  md.setSpeeds(initialSpeed, -initialSpeed  + 5);
  
  long cnt = 0;
  while (!okLeft || !okRight) {
    getCount(m1, m2);
    cnt++;
    if (cnt >= 150000) {
      md.setBrakes(400, 400);
      break;
    }
    if (m1 > leftGoalTicks) {
      okLeft = true;
      md.setM1Brake(400);
    }
    if (m2 > leftGoalTicks) {
      okRight = true;
      md.setM2Brake(400);
    }   
    if (!okLeft && !okRight && (m1 > leftSlowTicks || m2 > leftSlowTicks)) {
      md.setSpeeds(slowSpeed, -slowSpeed );
    }
  }
  delay(DELAY_MOVEMENT);
}

void moveRight() {
  bool okLeft = false;
  bool okRight = false;
  int m1, m2;
  resetCount();
//  md.setSpeeds(-initialSpeed, initialSpeed  );
  md.setSpeeds(-initialSpeed, initialSpeed - 5 );

  long cnt = 0;
  while (!okLeft || !okRight) {
    getCount(m1, m2);
    cnt++;
    if (cnt >= 150000) {
      md.setBrakes(400, 400);
      break;
    }
    if (m1 > rightGoalTicks) {
      okLeft = true;
      md.setM1Brake(400);
    }
    if (m2 > rightGoalTicks) {
      okRight = true;
      md.setM2Brake(400);
    }   
    if (!okLeft && !okRight && (m1 > rightSlowTicks || m2 > rightSlowTicks)) {
      md.setSpeeds(-slowSpeed, slowSpeed );
    }
  }
  delay(DELAY_MOVEMENT);
}

