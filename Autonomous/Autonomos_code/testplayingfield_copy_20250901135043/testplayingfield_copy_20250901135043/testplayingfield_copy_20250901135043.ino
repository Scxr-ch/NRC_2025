#include <smorphi.h>
#include <Adafruit_MotorShield.h>
#include <Adafruit_MCP23X17.h>
#include <Wire.h>

// Create Smorphi object
Smorphi robot;

// Declare I/O expander objects as 'extern' to avoid multiple definition errors
// The objects are defined in the Smorphi2 library's smorphi_test.cpp file.
extern Adafruit_MCP23X17 mcp1, mcp2, mcp3, mcp4;

// ===================================
// 1. PIN VALUE ADAPTATION & MAPPING
// ===================================
// Centralized hardware configuration based on smorphi_test.cpp
// NOTE: Master board interrupt pin values (INT_PIN_1, etc.) were not defined in the reference code.
// These are placeholders for illustration.
#define INT_PIN_1   34
#define INT_PIN_2   35
#define INT_PIN_3   36
#define INT_PIN_4   39

// I2C addresses for Motor Shields
#define MOTOR_SHIELD_1_ADDR 0x60
#define MOTOR_SHIELD_2_ADDR 0x61
#define MOTOR_SHIELD_3_ADDR 0x62
#define MOTOR_SHIELD_4_ADDR 0x63

// I2C addresses for I/O Expanders
#define IO_EXPANDER_1_ADDR 0x20
#define IO_EXPANDER_2_ADDR 0x21
#define IO_EXPANDER_3_ADDR 0x22
#define IO_EXPANDER_4_ADDR 0x23

// Sensor pin assignments on I/O expanders
// Note: Only pins 0, 2, 4, 6, 8, 10 are used for sensors in the reference.
// Pin 14 and 15 are used for solenoids.
#define SENSOR_FRONT_PIN      0
#define SENSOR_RIGHT_PIN      4
#define SENSOR_BACK_PIN       0
#define SENSOR_LEFT_PIN       0

// Hardware Constants & Thresholds
const int GRID_SIZE = 20; 
const int MOVEMENT_SPEED = 40; // (0-100 scale)
const int EXPLORATION_DISTANCE = 30; // cm to move
const int OBSTACLE_THRESHOLD = 50; // sensor value
const int SENSOR_MIN_VALUE = 0;
const int SENSOR_MAX_VALUE = 1023;
const int PWM_MIN = 0;
const int PWM_MAX = 255; // Standard PWM range

// Mapping Configuration
bool explored_grid[GRID_SIZE][GRID_SIZE];
bool obstacle_grid[GRID_SIZE][GRID_SIZE];
int robot_x = GRID_SIZE / 2;
int robot_y = GRID_SIZE / 2;
enum Direction { NORTH = 0, EAST = 1, SOUTH = 2, WEST = 3 };
Direction current_heading = NORTH;

// ===================================
// 2. ERROR VALIDATION & MONITORING
// ===================================
enum LogLevel { DEBUG, INFO, WARN, ERROR, FATAL };

// Forward declarations to allow setup() and loop() to call these functions
void logMessage(LogLevel level, const char* message, const char* function, int code = 0);
void emergencyStop();
bool checkMotorResponse(int pwm_value);
void initializeMapping();
bool readAllSensors(int& front, int& right, int& back, int& left);
void moveForwardOneCell();
void turnToDirection(Direction target_direction);


void logMessage(LogLevel level, const char* message, const char* function, int code) {
    Serial.print(millis());
    Serial.print("ms [");
    switch (level) {
        case DEBUG: Serial.print("DEBUG"); break;
        case INFO:  Serial.print("INFO");  break;
        case WARN:  Serial.print("WARN");  break;
        case ERROR: Serial.print("ERROR"); break;
        case FATAL: Serial.print("FATAL"); break;
    }
    Serial.print("] (");
    Serial.print(function);
    Serial.print("): ");
    Serial.println(message);
    if (code != 0) {
        Serial.print("Error Code: ");
        Serial.println(code);
    }
}

void emergencyStop() {
    logMessage(FATAL, "Emergency stop protocol initiated.", "emergencyStop", 999);
    robot.stopSmorphi();
    while (true) {
        // Halt all operations
        delay(100);
    }
}

void watchdogReset() {
    // Implement watchdog timer here. For ESP32, this might be esp_task_wdt_feed().
    // This function prevents the system from hanging by resetting it after a timeout.
}

bool checkMotorResponse(int pwm_value) {
    if (pwm_value < PWM_MIN || pwm_value > PWM_MAX) {
        logMessage(ERROR, "PWM value out of valid range (0-255).", "checkMotorResponse", 302);
        return false;
    }
    // Further checks would require motor feedback sensors (encoders, current sensors)
    // which are not available in the provided code.
    return true;
}

// ===================================
// 3. ENHANCED MAPPING ALGORITHM
// ===================================
// The following functions are now placed before setup() and loop() to ensure
// they are always defined before being called.

void initializeMapping() {
    logMessage(INFO, "Initializing Smorphi Mapping System...", "initializeMapping");

    // Initialize Smorphi robot
    robot.BeginSmorphi();
    // Add a delay to give hardware time to initialize
    delay(1000); 

    // Set robot to optimal shape for exploration (I-shape)
    logMessage(INFO, "Setting robot shape to 'I' for exploration.", "initializeMapping");
    robot.I();
    delay(500);
    // Note: Shape morphing completion confirmation requires feedback not present.

    // Initialize grids
    for (int i = 0; i < GRID_SIZE; i++) {
        for (int j = 0; j < GRID_SIZE; j++) {
            explored_grid[i][j] = false;
            obstacle_grid[i][j] = false;
        }
    }
    explored_grid[robot_x][robot_y] = true;
    logMessage(INFO, "Mapping system initialized. Starting exploration.", "initializeMapping");
}

// Read sensors with bounds checking and fault detection
bool readAllSensors(int& front, int& right, int& back, int& left) {
    front = robot.module1_sensor_status(SENSOR_FRONT_PIN);
    right = robot.module1_sensor_status(SENSOR_RIGHT_PIN);
    back = robot.module4_sensor_status(SENSOR_BACK_PIN);
    left = robot.module2_sensor_status(SENSOR_LEFT_PIN);

    // DEBUGGING: Print IR sensor values to serial monitor
    Serial.print("IR Sensor Values: Front=");
    Serial.print(front);
    Serial.print(", Right=");
    Serial.print(right);
    Serial.print(", Back=");
    Serial.print(back);
    Serial.print(", Left=");
    Serial.println(left);

    // Sensor reading bounds checking
    if (front < SENSOR_MIN_VALUE || front > SENSOR_MAX_VALUE ||
        right < SENSOR_MIN_VALUE || right > SENSOR_MAX_VALUE ||
        back < SENSOR_MIN_VALUE || back > SENSOR_MAX_VALUE ||
        left < SENSOR_MIN_VALUE || left > SENSOR_MAX_VALUE) {
        logMessage(ERROR, "Sensor reading out of expected bounds.", "readAllSensors", 201);
        // Implement automatic sensor recalibration here
        return false;
    }

    // Stuck reading detection (simple check for now)
    if (front == SENSOR_MAX_VALUE || right == SENSOR_MAX_VALUE) {
         logMessage(WARN, "Sensor reading is at maximum value, potential fault.", "readAllSensors", 202);
         // Implement fallback movement strategies
    }

    return true;
}

// Move robot forward with parameter and execution validation
void moveForwardOneCell() {
    logMessage(DEBUG, "Attempting to move forward one cell.", "moveForwardOneCell");
    if (MOVEMENT_SPEED < 0 || MOVEMENT_SPEED > 100) {
        logMessage(ERROR, "Movement command parameter out of range.", "moveForwardOneCell", 301);
        return;
    }
    
    // Move the robot physically
    robot.MoveForward(MOVEMENT_SPEED);
    if (!checkMotorResponse(robot.map_lv_PWM(robot.mapPosRanges(MOVEMENT_SPEED)))) {
         logMessage(WARN, "Motor response is outside expected PWM range.", "moveForwardOneCell", 303);
    }
    delay(1500);
    robot.stopSmorphi();
    
    // Grid position boundary validation
    int new_x = robot_x, new_y = robot_y;
    switch (current_heading) {
        case NORTH: new_y++; break;
        case EAST: new_x++; break;
        case SOUTH: new_y--; break;
        case WEST: new_x--; break;
    }

    if (new_x < 0 || new_x >= GRID_SIZE || new_y < 0 || new_y >= GRID_SIZE) {
        logMessage(WARN, "Next movement will exceed grid boundaries.", "moveForwardOneCell", 401);
        // The robot does not move. You may want to add code here to make it turn.
        return; 
    } else {
        // Only update the position if the move is valid
        robot_x = new_x;
        robot_y = new_y;
        explored_grid[robot_x][robot_y] = true;
        logMessage(INFO, "Moved to new position.", "moveForwardOneCell");
    }
}

void turnToDirection(Direction target_direction) {
    if (target_direction == current_heading) return;
    if (MOVEMENT_SPEED < 0 || MOVEMENT_SPEED > 100) {
        logMessage(ERROR, "Turn speed parameter out of range.", "turnToDirection", 304);
        return;
    }
    
    int turns_needed = (target_direction - current_heading + 4) % 4;
    for (int i = 0; i < turns_needed; i++) {
        robot.CenterPivotRight(MOVEMENT_SPEED);
        if (!checkMotorResponse(robot.map_ang_PWM(robot.mapNegAng(MOVEMENT_SPEED)))) {
            logMessage(WARN, "Motor response is outside expected PWM range.", "turnToDirection", 305);
            // Graceful degradation: stop and log
        }
        delay(750);
        robot.stopSmorphi();
        delay(200);
        logMessage(DEBUG, "Completed quarter turn.", "turnToDirection");
    }
    
    current_heading = target_direction;
    logMessage(INFO, "Turned to face new direction.", "turnToDirection");
}

// ===================================
// MAIN SKETCH
// ===================================

void setup() {
    Serial.begin(115200);
    delay(2000); // Give time for serial monitor to connect
    logMessage(INFO, "System booting up...", "setup");
    initializeMapping();
}

void loop() {
    // watchdogReset();
    
    // Check communication integrity
    if (!Serial) {
        logMessage(ERROR, "Serial communication lost.", "loop", 501);
        // Attempt re-initialization or fallback to LED-based status
    }

    // Check memory allocation (placeholder)
    // if (ESP.getFreeHeap() < MIN_FREE_HEAP) {
    //   logMessage(FATAL, "Low memory: memory overflow imminent.", "loop", 601);
    //   emergencyStop();
    // }

    // This is the primary mapping logic from the original program
    int front, right, back, left;
    if (!readAllSensors(front, right, back, left)) {
        // Sensor failure, handle appropriately
        logMessage(WARN, "Sensor readings are unreliable, skipping a step.", "loop");
        delay(1000); // Wait and try again
        return;
    }
    
    // Print current robot status for debugging
    Serial.print("Current Position: (");
    Serial.print(robot_x);
    Serial.print(", ");
    Serial.print(robot_y);
    Serial.print("), Heading: ");
    Serial.println(current_heading);

    // Original mapping logic (with validation)
    bool front_clear = (front < OBSTACLE_THRESHOLD);
    bool right_clear = (right < OBSTACLE_THRESHOLD);
    bool back_clear = (back < OBSTACLE_THRESHOLD);
    bool left_clear = (left < OBSTACLE_THRESHOLD);

    if (front_clear) {
        logMessage(INFO, "Path ahead is clear. Moving forward.", "loop");
        moveForwardOneCell();
    } else {
        logMessage(INFO, "Obstacle detected, turning to find new path.", "loop");
        // Prioritize turning left to find an unexplored area
        turnToDirection(static_cast<Direction>((current_heading + 3) % 4));
    }
    
    // Re-check for unexplored areas.
    // The `hasUnexploredAreas` and `isReachable` functions require the full
    // implementation from the original code which is outside the scope of this file.
    // I will include the core logic here.
    bool hasUnexplored = false;
    for (int i = 0; i < GRID_SIZE; i++) {
        for (int j = 0; j < GRID_SIZE; j++) {
            if (!explored_grid[i][j] && !obstacle_grid[i][j]) {
                hasUnexplored = true;
                break;
            }
        }
        if (hasUnexplored) break;
    }
    
    if (!hasUnexplored) {
        logMessage(INFO, "All reachable areas explored. Mapping complete.", "loop");
        // Add final steps, like a return to base, or sleep mode.
    }
    
    delay(100);
}