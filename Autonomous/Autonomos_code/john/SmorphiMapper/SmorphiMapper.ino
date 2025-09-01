// SmorphiMapper.ino

// Include necessary headers
#include "smorphi.h"
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <Adafruit_MCP23X17.h>
#include "HUSKYLENS.h"
#include "SoftwareSerial.h"

// Define Direction enum with unique names to avoid conflict
enum Direction { DIR_FORWARD = 0, DIR_BACKWARD = 1, DIR_LEFT = 2, DIR_RIGHT = 3 };
enum RobotShape { I_SHAPE, O_SHAPE, J_SHAPE, L_SHAPE };

// Configuration Constants
const int IR_SENSORS = 4;
const int GRID_SIZE = 50;
const float D_TOLERANCE = 15.0;
const int TRIMPOT_PIN = A0;
const int ULTRASONIC_TRIG = 12;
const int ULTRASONIC_ECHO = 13;

// HUSKYLENS and Serial Setup
SoftwareSerial myHuskySerial(19, 18); // RX, TX
HUSKYLENS huskylens;

// Global variables
bool sensor_states[IR_SENSORS];
float sensor_distances[IR_SENSORS];
bool coverage_grid[GRID_SIZE][GRID_SIZE];
int current_x = GRID_SIZE / 2, current_y = GRID_SIZE / 2;
Direction current_heading = DIR_FORWARD;

// Instantiate Smorphi object
Smorphi robot;

void printResult(HUSKYLENSResult result) {
    if (result.command == COMMAND_RETURN_BLOCK) {
        Serial.println(result.ID);
    } else if (result.command == COMMAND_RETURN_ARROW) {
        Serial.println("Wrong mode");
    } else {
        Serial.println("Object unknown!");
    }
}

class SmorphiMotorAccelerometer {
private:
    double prev_pwm[4][4];
    double current_pwm[4][4];
    unsigned long prev_time;
    unsigned long current_time;
    int prev_speed = 0;
    int current_speed = 0;
    Direction current_direction = DIR_FORWARD;
    Direction prev_direction = DIR_FORWARD;
    float linear_acceleration = 0.0;
    float angular_acceleration = 0.0;

public:
    SmorphiMotorAccelerometer() {
        prev_time = millis();
        initializePWMArrays();
    }

    void initializePWMArrays() {
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                prev_pwm[i][j] = 0.0;
                current_pwm[i][j] = 0.0;
            }
        }
    }

    float calculateLinearAcceleration(int new_speed, Direction direction) {
        current_time = millis();
        float dt = (current_time - prev_time) / 1000.0;
        if (dt <= 0) return 0.0;

        float effective_prev_speed = prev_speed;
        if (direction != prev_direction) {
            effective_prev_speed = -prev_speed;
        }

        linear_acceleration = (new_speed - effective_prev_speed) / dt;

        prev_speed = current_speed;
        current_speed = new_speed;
        prev_direction = current_direction;
        current_direction = direction;
        prev_time = current_time;

        return linear_acceleration;
    }

    float calculatePWMAcceleration() {
        float total_pwm_change = 0.0;
        int active_wheels = 0;
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                float pwm_change = current_pwm[i][j] - prev_pwm[i][j];
                if (abs(pwm_change) > 1.0) {
                    total_pwm_change += pwm_change;
                    active_wheels++;
                }
            }
        }
        if (active_wheels == 0) return 0.0;
        float avg_pwm_change = total_pwm_change / active_wheels;
        float dt = (current_time - prev_time) / 1000.0;
        return (dt > 0) ? (avg_pwm_change / dt) : 0.0;
    }

    void updatePWMState() {
        memcpy(prev_pwm, current_pwm, sizeof(current_pwm));
    }

    bool detectMotionChange() {
        float accel_magnitude = sqrt(linear_acceleration * linear_acceleration + angular_acceleration * angular_acceleration);
        return (accel_magnitude > 10.0);
    }

    float getAccelerationGradient() {
        static float prev_total_accel = 0.0;
        float current_total_accel = sqrt(linear_acceleration * linear_acceleration + angular_acceleration * angular_acceleration);
        float gradient = current_total_accel - prev_total_accel;
        prev_total_accel = current_total_accel;
        return gradient;
    }
};

class SmorphiMapper {
private:
    SmorphiMotorAccelerometer motor_accel;
    bool movement_locked = false;
    Direction fixed_direction;
    int movement_speed = 60; // Match working code speed
    RobotShape current_shape = I_SHAPE;
    unsigned long last_milestone_time = 0;

    struct MovementState {
        int direction;
        int speed;
        unsigned long timestamp;
        float acceleration;
    };

    MovementState movement_history[10];
    int history_index = 0;

    // Sensor positions based on working code
    int getSensorPin(RobotShape shape, int sensor_dir) {
        switch (shape) {
            case I_SHAPE:
                if (sensor_dir == 0) return 0;  // Front: Module 1, Pin 0
                if (sensor_dir == 1) return 4;  // Right: Module 1, Pin 4
                if (sensor_dir == 2) return 0;  // Rear: Module 3, Pin 0
                if (sensor_dir == 3) return 10; // Left: Module 1, Pin 10
                break;
            case O_SHAPE:
                if (sensor_dir == 0) return 0;  // Front: Module 1, Pin 0
                if (sensor_dir == 1) return 4;  // Right: Module 1, Pin 4
                if (sensor_dir == 2) return 6;  // Rear: Module 3, Pin 6
                if (sensor_dir == 3) return 0;  // Left: Module 4, Pin 0
                break;
            case L_SHAPE:
                // Assume similar to O shape for now (adjust based on hardware)
                if (sensor_dir == 0) return 0;
                if (sensor_dir == 1) return 4;
                if (sensor_dir == 2) return 6;
                if (sensor_dir == 3) return 0;
                break;
        }
        return -1; // Invalid
    }

public:
    SmorphiMapper() {
        initializeGrid();
    }

    void initializeGrid() {
        for (int i = 0; i < GRID_SIZE; i++) {
            for (int j = 0; j < GRID_SIZE; j++) {
                coverage_grid[i][j] = false;
            }
        }
        coverage_grid[current_x][current_y] = true;
    }

    float readTolerance() {
        int pot_value = analogRead(TRIMPOT_PIN);
        return map(pot_value, 0, 4095, 5.0, 30.0);
    }

    void pollSensors() {
        RobotShape shape = (robot.sm_getShape() == 'i') ? I_SHAPE : (robot.sm_getShape() == 'o') ? O_SHAPE : L_SHAPE;
        sensor_distances[0] = robot.module1_sensor_status(getSensorPin(shape, 0)); // Front
        sensor_distances[1] = robot.module1_sensor_status(getSensorPin(shape, 1)); // Right
        sensor_distances[2] = (shape == I_SHAPE) ? robot.module3_sensor_status(getSensorPin(shape, 2)) : robot.module3_sensor_status(getSensorPin(shape, 2)); // Rear
        sensor_distances[3] = (shape == I_SHAPE) ? robot.module1_sensor_status(getSensorPin(shape, 3)) : robot.module4_sensor_status(getSensorPin(shape, 3)); // Left
        for (int i = 0; i < IR_SENSORS; i++) {
            sensor_states[i] = (sensor_distances[i] < D_TOLERANCE);
        }
    }

    Direction selectRandomClearDirection() {
        int clear_directions[IR_SENSORS];
        int clear_count = 0;

        for (int i = 0; i < IR_SENSORS; i++) {
            if (!sensor_states[i]) {
                clear_directions[clear_count++] = i;
            }
        }

        if (clear_count == 0) return Direction(-1);

        return static_cast<Direction>(clear_directions[random(clear_count)]);
    }

    void calculateNextPosition(Direction dir, int& next_x, int& next_y) {
        next_x = current_x;
        next_y = current_y;
        switch (dir) {
            case DIR_FORWARD: next_y++; break;
            case DIR_BACKWARD: next_y--; break;
            case DIR_LEFT: next_x--; break;
            case DIR_RIGHT: next_x++; break;
        }
    }

    bool isValidPosition(int x, int y) {
        return (x >= 0 && x < GRID_SIZE && y >= 0 && y < GRID_SIZE);
    }

    void updatePosition(Direction dir) {
        calculateNextPosition(dir, current_x, current_y);
        current_heading = dir;
    }

    void markCurrentAreaCovered() {
        if (isValidPosition(current_x, current_y)) {
            coverage_grid[current_x][current_y] = true;
        }
    }

    void updatePathMemory(float gradient) {
        if (gradient > 0) {
            Serial.println("Moving into open area");
        }
    }

    void reinforceDirectionMemory(Direction dir, bool positive_polarity) {
        static int direction_success[4] = {0, 0, 0, 0};
        if (positive_polarity) {
            direction_success[dir]++;
        } else {
            direction_success[dir]--;
        }
    }

    bool hasUnexploredAreas() {
        for (int i = 0; i < GRID_SIZE; i++) {
            for (int j = 0; j < GRID_SIZE; j++) {
                if (!coverage_grid[i][j] && isAccessibleArea(i, j)) {
                    return true;
                }
            }
        }
        return false;
    }

    bool isAccessibleArea(int x, int y) {
        return (abs(x - current_x) + abs(y - current_y)) < GRID_SIZE;
    }

    void optimizeShape() {
        if (huskylens.available()) {
            HUSKYLENSResult result = huskylens.read();
            printResult(result);
            if (result.command == COMMAND_RETURN_BLOCK) {
                switch (result.ID) {
                    case 1: morphTo(I_SHAPE); break;
                    case 2: morphTo(O_SHAPE); break;
                    case 3: morphTo(L_SHAPE); break;
                }
            }
        }
    }

    void morphTo(RobotShape new_shape) {
        if (new_shape != current_shape) {
            current_shape = new_shape;
            robot.sm_reset_M1();
            robot.sm_reset_M2();
            robot.sm_reset_M3();
            robot.sm_reset_M4();
            switch (new_shape) {
                case I_SHAPE: robot.I(); break;
                case O_SHAPE: robot.O(); break;
                case L_SHAPE: robot.L(); break;
            }
            delay(500);
        }
    }

    float calculateCoveragePercentage() {
        int covered_count = 0;
        int total_accessible = 0;
        for (int i = 0; i < GRID_SIZE; i++) {
            for (int j = 0; j < GRID_SIZE; j++) {
                if (isAccessibleArea(i, j)) {
                    total_accessible++;
                    if (coverage_grid[i][j]) covered_count++;
                }
            }
        }
        return (total_accessible > 0) ? ((float)covered_count / total_accessible * 100.0) : 0.0;
    }

    void executeMovement(Direction dir) {
        if (!movement_locked) {
            fixed_direction = dir;
            movement_locked = true;
        }

        float accel_before = motor_accel.calculateLinearAcceleration(movement_speed, dir);

        moveRobotSmorphi(dir, movement_speed);

        trackAccelerationDuringMovement(dir, movement_speed);

        updatePosition(dir);
        markCurrentAreaCovered();

        recordMovementHistory(dir, movement_speed, accel_before);
    }

private:
    void moveRobotSmorphi(Direction dir, int speed) {
        switch (dir) {
            case DIR_FORWARD:
                robot.MoveForward(speed);
                delay(1000);
                robot.stopSmorphi();
                break;
            case DIR_BACKWARD:
                robot.MoveBackward(speed);
                delay(1000);
                robot.stopSmorphi();
                break;
            case DIR_LEFT:
                robot.MoveLeft(speed);
                delay(1000);
                robot.stopSmorphi();
                break;
            case DIR_RIGHT:
                robot.MoveRight(speed);
                delay(1000);
                robot.stopSmorphi();
                break;
        }
    }

    void trackAccelerationDuringMovement(Direction dir, int speed) {
        unsigned long movement_start = millis();
        unsigned long sample_interval = 100;
        for (int i = 0; i < 9; i++) {
            motor_accel.updatePWMState();
            float current_accel = motor_accel.calculateLinearAcceleration(speed, dir);
            if (motor_accel.detectMotionChange()) {
                float gradient = motor_accel.getAccelerationGradient();
                updatePathMemory(gradient);
            }
            if (current_accel > 0) {
                reinforceDirectionMemory(fixed_direction, true);
            }
            delay(sample_interval);
        }
    }

    void recordMovementHistory(Direction dir, int speed, float acceleration) {
        movement_history[history_index].direction = static_cast<int>(dir);
        movement_history[history_index].speed = speed;
        movement_history[history_index].timestamp = millis();
        movement_history[history_index].acceleration = acceleration;
        history_index = (history_index + 1) % 10;
    }

    Direction selectOptimalDirection() {
        Direction clear_direction = selectRandomClearDirection();
        if (clear_direction == Direction(-1)) return clear_direction;

        if (wasRecentlyVisited(clear_direction)) {
            for (int i = 0; i < IR_SENSORS; i++) {
                if (!sensor_states[i] && !wasRecentlyVisited(static_cast<Direction>(i))) {
                    return static_cast<Direction>(i);
                }
            }
        }
        return clear_direction;
    }

    bool wasRecentlyVisited(Direction dir) {
        int recent_count = 0;
        for (int i = 0; i < 10; i++) {
            if (movement_history[i].direction == static_cast<int>(dir) &&
                (millis() - movement_history[i].timestamp) < 10000) {
                recent_count++;
            }
        }
        return (recent_count > 2);
    }

public:
    void runMappingAlgorithm() {
        Serial.println("Starting Smorphi area mapping...");
        robot.BeginSmorphi();
        huskylens.begin(myHuskySerial);

        while (hasUnexploredAreas()) {
            robot.stopSmorphi();
            delay(200);

            pollSensors();
            optimizeShape();

            Direction next_direction = selectOptimalDirection();

            if (next_direction == Direction(-1)) {
                Serial.println("No clear path available");
                break;
            }

            executeMovement(next_direction);

            if (millis() - last_milestone_time > 5000) {
                float coverage = calculateCoveragePercentage();
                Serial.print("Coverage: ");
                Serial.print(coverage);
                Serial.print("%, Speed: ");
                Serial.println(movement_speed);
                last_milestone_time = millis();
            }
        }

        Serial.println("Mapping complete!");
        Serial.print("Final coverage: ");
        Serial.print(calculateCoveragePercentage());
        Serial.println("%");
        robot.stopSmorphi();
    }
};

// Global mapper instance
SmorphiMapper mapper;

void setup() {
    Serial.begin(115200);
    pinMode(ULTRASONIC_TRIG, OUTPUT);
    pinMode(ULTRASONIC_ECHO, INPUT);
    myHuskySerial.begin(9600);

    while (!huskylens.begin(myHuskySerial)) {
        Serial.println(F("Begin failed!"));
        Serial.println(F("1.Please recheck the \"Protocol Type\" in HUSKYLENS (General Settings>>Protocol Type>>Serial 9600)"));
        Serial.println(F("2.Please recheck the connection."));
        delay(100);
    }

    delay(2000);
    Serial.println("Smorphi Mapper Initialized");
}

void loop() {
    mapper.runMappingAlgorithm();

    while (true) {
        delay(1000);
        Serial.println("Mapping algorithm finished. Reset to restart.");
    }
}