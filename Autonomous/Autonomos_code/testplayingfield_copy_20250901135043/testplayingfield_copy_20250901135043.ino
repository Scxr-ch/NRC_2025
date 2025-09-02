#include <smorphi.h>

// Create Smorphi object
Smorphi robot;

// Mapping Configuration
const int GRID_SIZE = 20;  // Reduced for memory efficiency
const int EXPLORATION_DISTANCE = 30; // cm to move in each step
const int OBSTACLE_THRESHOLD = 50;   // Sensor threshold for obstacle detection
const int MOVEMENT_SPEED = 40;       // Default movement speed (0-100)

// Direction definitions
enum Direction {
  NORTH = 0,
  EAST = 1,
  SOUTH = 2,
  WEST = 3
};

// Grid mapping variables
bool explored_grid[GRID_SIZE][GRID_SIZE];
bool obstacle_grid[GRID_SIZE][GRID_SIZE];
int robot_x = GRID_SIZE / 2;  // Start in center
int robot_y = GRID_SIZE / 2;
Direction current_heading = NORTH;

// Sensor configuration - using all 4 modules for perimeter coverage
struct SensorReading {
  bool front_clear;
  bool back_clear;
  bool left_clear;
  bool right_clear;
};

class SmorphiMapper {
private:
  unsigned long last_position_update = 0;
  int exploration_step_count = 0;
  
public:
  void initializeMapping() {
    Serial.println("Initializing Smorphi Mapping System...");
    
    // Initialize Smorphi robot
    robot.BeginSmorphi();
    delay(1000);
    
    // Set robot to optimal shape for exploration (I-shape for maneuverability)
    robot.I();
    delay(500);
    
    // Initialize grids
    for(int i = 0; i < GRID_SIZE; i++) {
      for(int j = 0; j < GRID_SIZE; j++) {
        explored_grid[i][j] = false;
        obstacle_grid[i][j] = false;
      }
    }
    
    // Mark starting position as explored
    explored_grid[robot_x][robot_y] = true;
    
    Serial.println("Mapping system initialized. Starting exploration...");
  }
  
  // Read sensors from all modules for 360-degree coverage
  SensorReading readAllSensors() {
    SensorReading reading;
    
    // Module 1: Front sensors (assuming module 1 is front-facing)
    int front_sensor1 = robot.module1_sensor_status(0);
    reading.front_clear = (front_sensor1 < OBSTACLE_THRESHOLD);
    
    // Module 2: Right sensors  
    int right_sensor1 = robot.module1_sensor_status(4);
    reading.right_clear = (right_sensor1 < OBSTACLE_THRESHOLD);
    
    // Module 3: Back sensors
    int back_sensor1 = robot.module4_sensor_status(0);
    reading.back_clear = (back_sensor1 < OBSTACLE_THRESHOLD);
    
    // Module 4: Left sensors
    int left_sensor1 = robot.module2_sensor_status(0);
    reading.left_clear = (left_sensor1 < OBSTACLE_THRESHOLD);
    
    return reading;
  }
  
  // Convert sensor readings to world directions based on robot heading
  SensorReading getWorldDirectionReadings(SensorReading local_reading) {
    SensorReading world_reading;
    
    switch(current_heading) {
      case NORTH:
        world_reading.front_clear = local_reading.front_clear;  // North
        world_reading.right_clear = local_reading.right_clear;  // East
        world_reading.back_clear = local_reading.back_clear;    // South
        world_reading.left_clear = local_reading.left_clear;    // West
        break;
      case EAST:
        world_reading.front_clear = local_reading.left_clear;   // North
        world_reading.right_clear = local_reading.front_clear;  // East
        world_reading.back_clear = local_reading.right_clear;   // South
        world_reading.left_clear = local_reading.back_clear;    // West
        break;
      case SOUTH:
        world_reading.front_clear = local_reading.back_clear;   // North
        world_reading.right_clear = local_reading.left_clear;   // East
        world_reading.back_clear = local_reading.front_clear;   // South
        world_reading.left_clear = local_reading.right_clear;   // West
        break;
      case WEST:
        world_reading.front_clear = local_reading.right_clear;  // North
        world_reading.right_clear = local_reading.back_clear;   // East
        world_reading.back_clear = local_reading.left_clear;    // South
        world_reading.left_clear = local_reading.front_clear;   // West
        break;
    }
    
    return world_reading;
  }
  
  // Update obstacle map based on sensor readings
  void updateObstacleMap(SensorReading world_reading) {
    // Mark obstacles in adjacent grid cells
    if(!world_reading.front_clear && robot_y + 1 < GRID_SIZE) {
      obstacle_grid[robot_x][robot_y + 1] = true; // North
    }
    if(!world_reading.right_clear && robot_x + 1 < GRID_SIZE) {
      obstacle_grid[robot_x + 1][robot_y] = true; // East
    }
    if(!world_reading.back_clear && robot_y - 1 >= 0) {
      obstacle_grid[robot_x][robot_y - 1] = true; // South
    }
    if(!world_reading.left_clear && robot_x - 1 >= 0) {
      obstacle_grid[robot_x - 1][robot_y] = true; // West
    }
  }
  
  // Find the best unexplored direction
  Direction findBestDirection() {
    SensorReading local_reading = readAllSensors();
    SensorReading world_reading = getWorldDirectionReadings(local_reading);
    
    // Update obstacle map
    updateObstacleMap(world_reading);
    
    // Priority: unexplored areas that are accessible
    Direction priorities[4];
    bool available[4];
    
    // Check each world direction
    available[0] = world_reading.front_clear && (robot_y + 1 < GRID_SIZE) && !explored_grid[robot_x][robot_y + 1]; // North
    available[1] = world_reading.right_clear && (robot_x + 1 < GRID_SIZE) && !explored_grid[robot_x + 1][robot_y]; // East  
    available[2] = world_reading.back_clear && (robot_y - 1 >= 0) && !explored_grid[robot_x][robot_y - 1];        // South
    available[3] = world_reading.left_clear && (robot_x - 1 >= 0) && !explored_grid[robot_x - 1][robot_y];       // West
    
    // Find first available unexplored direction
    for(int i = 0; i < 4; i++) {
      if(available[i]) {
        return static_cast<Direction>(i);
      }
    }
    
    // If no unexplored areas, choose any clear direction
    if(world_reading.front_clear) return NORTH;
    if(world_reading.right_clear) return EAST;
    if(world_reading.back_clear) return SOUTH;
    if(world_reading.left_clear) return WEST;
    
    return static_cast<Direction>(-1); // No clear direction
  }
  
  // Turn robot to face the desired world direction
  void turnToDirection(Direction target_direction) {
    if(target_direction == current_heading) return;
    
    // Calculate turns needed (clockwise)
    int turns_needed = (target_direction - current_heading + 4) % 4;
    
    for(int i = 0; i < turns_needed; i++) {
      robot.CenterPivotRight(MOVEMENT_SPEED);
      delay(750); // Quarter turn duration
      robot.stopSmorphi();
      delay(200);
    }
    
    current_heading = target_direction;
    Serial.print("Turned to face: ");
    Serial.println(directionToString(current_heading));
  }
  
  // Move robot forward one grid cell
  void moveForwardOneCell() {
    robot.MoveForward(MOVEMENT_SPEED);
    delay(1500); // Time to move one grid cell
    robot.stopSmorphi();
    
    // Update position based on current heading
    switch(current_heading) {
      case NORTH: robot_y++; break;
      case EAST:  robot_x++; break;
      case SOUTH: robot_y--; break;
      case WEST:  robot_x--; break;
    }
    
    // Ensure position stays within bounds
    robot_x = constrain(robot_x, 0, GRID_SIZE - 1);
    robot_y = constrain(robot_y, 0, GRID_SIZE - 1);
    
    // Mark new position as explored
    explored_grid[robot_x][robot_y] = true;
    exploration_step_count++;
    
    Serial.print("Moved to position: (");
    Serial.print(robot_x);
    Serial.print(", ");
    Serial.print(robot_y);
    Serial.println(")");
  }
  
  // Check if there are still unexplored areas
  bool hasUnexploredAreas() {
    for(int i = 0; i < GRID_SIZE; i++) {
      for(int j = 0; j < GRID_SIZE; j++) {
        if(!explored_grid[i][j] && !obstacle_grid[i][j]) {
          // Check if this area is reachable (adjacent to explored area)
          if(isReachable(i, j)) {
            return true;
          }
        }
      }
    }
    return false;
  }
  
  // Check if a position is reachable (adjacent to explored area)
  bool isReachable(int x, int y) {
    if(x > 0 && explored_grid[x-1][y]) return true;
    if(x < GRID_SIZE-1 && explored_grid[x+1][y]) return true;
    if(y > 0 && explored_grid[x][y-1]) return true;
    if(y < GRID_SIZE-1 && explored_grid[x][y+1]) return true;
    return false;
  }
  
  // Calculate exploration coverage percentage
  float calculateCoverage() {
    int explored_count = 0;
    int total_accessible = 0;
    
    for(int i = 0; i < GRID_SIZE; i++) {
      for(int j = 0; j < GRID_SIZE; j++) {
        if(!obstacle_grid[i][j]) {
          total_accessible++;
          if(explored_grid[i][j]) {
            explored_count++;
          }
        }
      }
    }
    
    return (total_accessible > 0) ? (float(explored_count) / total_accessible * 100.0) : 0.0;
  }
  
  // Adaptive shape morphing based on environment
  void adaptRobotShape() {
    SensorReading reading = readAllSensors();
    int clearPaths = 0;
    if(reading.front_clear) clearPaths++;
    if(reading.back_clear) clearPaths++;
    if(reading.left_clear) clearPaths++;
    if(reading.right_clear) clearPaths++;
    
    char currentShape = robot.sm_getShape();
    
    // Change shape based on space constraints
    if(clearPaths <= 1) {
      // Tight space - use linear I-shape
      if(currentShape != 'I') {
        robot.I();
        delay(500);
        Serial.println("Switched to I-shape for tight spaces");
      }
    } else if(clearPaths == 4) {
      // Open area - use O-shape for stability
      if(currentShape != 'O') {
        robot.O();
        delay(500);
        Serial.println("Switched to O-shape for open area");
      }
    } else if(clearPaths == 2 || clearPaths == 3) {
      // Moderate complexity - use L-shape
      if(currentShape != 'L') {
        robot.L();
        delay(500);
        Serial.println("Switched to L-shape for moderate complexity");
      }
    }
  }
  
  // Main mapping algorithm
  void runMapping() {
    Serial.println("=== Starting Area Mapping ===");
    unsigned long mapping_start_time = millis();
    
    while(hasUnexploredAreas()) {
      // Stop and assess environment
      robot.stopSmorphi();
      delay(500); // Stabilization time
      
      // Adapt robot shape based on environment
      adaptRobotShape();
      
      // Find next direction to explore
      Direction next_direction = findBestDirection();
      
      if(next_direction == Direction(-1)) {
        Serial.println("No accessible directions found. Mapping complete.");
        break;
      }
      
      // Turn to face the chosen direction
      turnToDirection(next_direction);
      
      // Move forward one grid cell
      moveForwardOneCell();
      
      // Progress reporting
      if(exploration_step_count % 5 == 0) {
        float coverage = calculateCoverage();
        Serial.print("Steps taken: ");
        Serial.print(exploration_step_count);
        Serial.print(", Coverage: ");
        Serial.print(coverage, 1);
        Serial.print("%, Shape: ");
        Serial.println(robot.sm_getShape());
      }
      
      // Safety timeout (prevent infinite loops)
      if(millis() - mapping_start_time > 300000) { // 5 minutes max
        Serial.println("Mapping timeout reached.");
        break;
      }
    }
    
    // Final stop and report
    robot.stopSmorphi();
    
    Serial.println("=== MAPPING COMPLETE ===");
    Serial.print("Total steps taken: ");
    Serial.println(exploration_step_count);
    Serial.print("Final coverage: ");
    Serial.print(calculateCoverage(), 1);
    Serial.println("%");
    Serial.print("Total time: ");
    Serial.print((millis() - mapping_start_time) / 1000);
    Serial.println(" seconds");
    
    printMap();
  }
  
  // Helper function to convert direction to string
  String directionToString(Direction dir) {
    switch(dir) {
      case NORTH: return "North";
      case EAST: return "East";
      case SOUTH: return "South";
      case WEST: return "West";
      default: return "Unknown";
    }
  }
  
  // Print exploration map to serial monitor
  void printMap() {
    Serial.println("\n=== EXPLORATION MAP ===");
    Serial.println("Legend: X=Explored, O=Obstacle, .=Unexplored, R=Robot");
    
    for(int j = GRID_SIZE - 1; j >= 0; j--) {
      for(int i = 0; i < GRID_SIZE; i++) {
        if(i == robot_x && j == robot_y) {
          Serial.print("R ");
        } else if(obstacle_grid[i][j]) {
          Serial.print("O ");
        } else if(explored_grid[i][j]) {
          Serial.print("X ");
        } else {
          Serial.print(". ");
        }
      }
      Serial.println();
    }
    Serial.println("========================\n");
  }
};

// Global mapper instance
SmorphiMapper mapper;

void setup() {
  Serial.begin(115200);
  delay(2000); // Give time for serial monitor to connect
  
  Serial.println("Smorphi Area Mapping System");
  Serial.println("===========================");
  
  // Initialize the mapping system
  mapper.initializeMapping();
}

void loop() {
  // Run the mapping algorithm
  mapper.runMapping();
  
  // After mapping is complete, enter idle state
  while(true) {
    Serial.println("Mapping complete. Reset to restart.");
    delay(5000);
  }
}