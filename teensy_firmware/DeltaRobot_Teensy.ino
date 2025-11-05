/*
 * Delta Robot Control Firmware for Teensy 4.1
 * Ethernet-based control with NEMA 23 stepper motors
 * 
 * Requirements:
 * - Teensy 4.1 (with Ethernet shield or built-in Ethernet)
 * - 3x Stepper motor drivers (e.g., DRV8825, TMC2208, TMC2209)
 * - AccelStepper library: https://www.airspayce.com/mikem/arduino/AccelStepper/
 * - NativeEthernet library (for Teensy 4.1)
 * 
 * Connections:
 * - Motor 1: STEP pin 2, DIR pin 3, ENABLE pin 4
 * - Motor 2: STEP pin 5, DIR pin 6, ENABLE pin 7
 * - Motor 3: STEP pin 8, DIR pin 9, ENABLE pin 10
 * - End-stop switches (optional): pins 11, 12, 13
 */

#include <AccelStepper.h>
#include <MultiStepper.h>
#include <NativeEthernet.h>

// Network configuration
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
IPAddress ip(192, 168, 1, 100);  // Change to match your network
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);
EthernetServer server(8080);

// Motor pin definitions
#define MOTOR1_STEP_PIN 2
#define MOTOR1_DIR_PIN 3
#define MOTOR1_ENABLE_PIN 4

#define MOTOR2_STEP_PIN 5
#define MOTOR2_DIR_PIN 6
#define MOTOR2_ENABLE_PIN 7

#define MOTOR3_STEP_PIN 8
#define MOTOR3_DIR_PIN 9
#define MOTOR3_ENABLE_PIN 10

// End-stop pins (optional)
#define ENDSTOP1_PIN 11
#define ENDSTOP2_PIN 12
#define ENDSTOP3_PIN 13

// Create stepper objects
AccelStepper motor1(AccelStepper::DRIVER, MOTOR1_STEP_PIN, MOTOR1_DIR_PIN);
AccelStepper motor2(AccelStepper::DRIVER, MOTOR2_STEP_PIN, MOTOR2_DIR_PIN);
AccelStepper motor3(AccelStepper::DRIVER, MOTOR3_STEP_PIN, MOTOR3_DIR_PIN);

MultiStepper steppers;

// Motor configuration
struct MotorConfig {
    float maxSpeed = 1000.0;      // Steps per second
    float acceleration = 500.0;   // Steps per second squared
    int microstepping = 16;      // Microstepping factor
    bool enabled = false;
};

MotorConfig motorConfigs[3];

// Current positions (in steps)
long positions[3] = {0, 0, 0};

// Command parsing
String commandBuffer = "";
bool commandComplete = false;

void setup() {
    Serial.begin(115200);
    
    // Initialize stepper motors
    motor1.setEnablePin(MOTOR1_ENABLE_PIN);
    motor1.setPinsInverted(false, false, true);  // Invert enable pin if needed
    motor1.setMaxSpeed(motorConfigs[0].maxSpeed);
    motor1.setAcceleration(motorConfigs[0].acceleration);
    
    motor2.setEnablePin(MOTOR2_ENABLE_PIN);
    motor2.setPinsInverted(false, false, true);
    motor2.setMaxSpeed(motorConfigs[1].maxSpeed);
    motor2.setAcceleration(motorConfigs[1].acceleration);
    
    motor3.setEnablePin(MOTOR3_ENABLE_PIN);
    motor3.setPinsInverted(false, false, true);
    motor3.setMaxSpeed(motorConfigs[2].maxSpeed);
    motor3.setAcceleration(motorConfigs[2].acceleration);
    
    // Add steppers to MultiStepper
    steppers.addStepper(motor1);
    steppers.addStepper(motor2);
    steppers.addStepper(motor3);
    
    // Initialize end-stop pins (if used)
    pinMode(ENDSTOP1_PIN, INPUT_PULLUP);
    pinMode(ENDSTOP2_PIN, INPUT_PULLUP);
    pinMode(ENDSTOP3_PIN, INPUT_PULLUP);
    
    // Disable motors initially
    motor1.disableOutputs();
    motor2.disableOutputs();
    motor3.disableOutputs();
    
    // Initialize Ethernet
    Ethernet.begin(mac, ip, gateway, gateway, subnet);
    server.begin();
    
    Serial.print("Server IP: ");
    Serial.println(Ethernet.localIP());
    Serial.println("Delta Robot Control Server Ready");
}

void loop() {
    // Check for incoming client connections
    EthernetClient client = server.available();
    
    if (client) {
        Serial.println("Client connected");
        
        while (client.connected()) {
            if (client.available()) {
                char c = client.read();
                
                if (c == '\n' || c == '\r') {
                    if (commandBuffer.length() > 0) {
                        processCommand(client, commandBuffer);
                        commandBuffer = "";
                    }
                } else {
                    commandBuffer += c;
                }
            }
            
            // Run steppers
            steppers.run();
        }
        
        Serial.println("Client disconnected");
        client.stop();
    }
    
    // Run steppers even when no client connected
    steppers.run();
}

void processCommand(EthernetClient& client, String command) {
    command.trim();
    command.toUpperCase();
    
    Serial.print("Command: ");
    Serial.println(command);
    
    // Parse G-code like commands
    if (command.startsWith("G1")) {
        // G1 - Move to absolute position
        // Format: G1 X<steps> Y<steps> Z<steps>
        long x = parseAxis(command, 'X', positions[0]);
        long y = parseAxis(command, 'Y', positions[1]);
        long z = parseAxis(command, 'Z', positions[2]);
        
        positions[0] = x;
        positions[1] = y;
        positions[2] = z;
        
        steppers.moveTo(positions);
        client.println("OK");
        
    } else if (command.startsWith("G28")) {
        // G28 - Home all motors
        homeMotors(client);
        client.println("OK");
        
    } else if (command.startsWith("G91")) {
        // G91 - Set relative positioning
        // This is handled in G1 parsing
        client.println("OK");
        
    } else if (command.startsWith("G92")) {
        // G92 - Set current position
        // Format: G92 X<steps> or Y<steps> or Z<steps>
        if (command.indexOf('X') >= 0) {
            positions[0] = parseAxis(command, 'X', positions[0]);
            motor1.setCurrentPosition(positions[0]);
        }
        if (command.indexOf('Y') >= 0) {
            positions[1] = parseAxis(command, 'Y', positions[1]);
            motor2.setCurrentPosition(positions[1]);
        }
        if (command.indexOf('Z') >= 0) {
            positions[2] = parseAxis(command, 'Z', positions[2]);
            motor3.setCurrentPosition(positions[2]);
        }
        client.println("OK");
        
    } else if (command.startsWith("M17")) {
        // M17 - Enable motors
        motor1.enableOutputs();
        motor2.enableOutputs();
        motor3.enableOutputs();
        motorConfigs[0].enabled = true;
        motorConfigs[1].enabled = true;
        motorConfigs[2].enabled = true;
        client.println("OK");
        
    } else if (command.startsWith("M18")) {
        // M18 - Disable motors
        motor1.disableOutputs();
        motor2.disableOutputs();
        motor3.disableOutputs();
        motorConfigs[0].enabled = false;
        motorConfigs[1].enabled = false;
        motorConfigs[2].enabled = false;
        client.println("OK");
        
    } else if (command.startsWith("M112")) {
        // M112 - Emergency stop
        motor1.stop();
        motor2.stop();
        motor3.stop();
        steppers.stop();
        client.println("OK");
        
    } else if (command.startsWith("M114")) {
        // M114 - Get current position
        client.print("X:");
        client.print(motor1.currentPosition());
        client.print(" Y:");
        client.print(motor2.currentPosition());
        client.print(" Z:");
        client.println(motor3.currentPosition());
        
    } else if (command.startsWith("M203")) {
        // M203 - Set speed
        // Format: M203 S<speed> T<motor>
        int motorIndex = parseValue(command, 'T', 0);
        float speed = parseValue(command, 'S', motorConfigs[motorIndex].maxSpeed);
        
        if (motorIndex == 0) motor1.setMaxSpeed(speed);
        else if (motorIndex == 1) motor2.setMaxSpeed(speed);
        else if (motorIndex == 2) motor3.setMaxSpeed(speed);
        
        if (motorIndex >= 0 && motorIndex < 3) {
            motorConfigs[motorIndex].maxSpeed = speed;
        }
        client.println("OK");
        
    } else if (command.startsWith("M204")) {
        // M204 - Set acceleration
        // Format: M204 A<accel> T<motor>
        int motorIndex = parseValue(command, 'T', 0);
        float accel = parseValue(command, 'A', motorConfigs[motorIndex].acceleration);
        
        if (motorIndex == 0) motor1.setAcceleration(accel);
        else if (motorIndex == 1) motor2.setAcceleration(accel);
        else if (motorIndex == 2) motor3.setAcceleration(accel);
        
        if (motorIndex >= 0 && motorIndex < 3) {
            motorConfigs[motorIndex].acceleration = accel;
        }
        client.println("OK");
        
    } else if (command.startsWith("M350")) {
        // M350 - Set microstepping (informational only, hardware dependent)
        // Format: M350 S<microsteps> T<motor>
        int motorIndex = parseValue(command, 'T', 0);
        int microsteps = parseValue(command, 'S', motorConfigs[motorIndex].microstepping);
        
        if (motorIndex >= 0 && motorIndex < 3) {
            motorConfigs[motorIndex].microstepping = microsteps;
        }
        client.println("OK");
        
    } else {
        client.println("ERR:Unknown command");
    }
}

long parseAxis(String command, char axis, long defaultValue) {
    int index = command.indexOf(axis);
    if (index < 0) return defaultValue;
    
    // Check if G91 (relative) is in command
    bool relative = command.indexOf("G91") >= 0;
    
    // Find the number after the axis letter
    index++;  // Skip the axis letter
    while (index < command.length() && (command[index] == ' ' || command[index] == '\t')) {
        index++;
    }
    
    String valueStr = "";
    bool negative = false;
    if (index < command.length() && command[index] == '-') {
        negative = true;
        index++;
    }
    
    while (index < command.length() && 
           (isdigit(command[index]) || command[index] == '.')) {
        valueStr += command[index];
        index++;
    }
    
    if (valueStr.length() == 0) return defaultValue;
    
    long value = valueStr.toInt();
    if (negative) value = -value;
    
    if (relative) {
        return defaultValue + value;
    } else {
        return value;
    }
}

float parseValue(String command, char axis, float defaultValue) {
    int index = command.indexOf(axis);
    if (index < 0) return defaultValue;
    
    index++;  // Skip the axis letter
    while (index < command.length() && (command[index] == ' ' || command[index] == '\t')) {
        index++;
    }
    
    String valueStr = "";
    bool negative = false;
    if (index < command.length() && command[index] == '-') {
        negative = true;
        index++;
    }
    
    while (index < command.length() && 
           (isdigit(command[index]) || command[index] == '.')) {
        valueStr += command[index];
        index++;
    }
    
    if (valueStr.length() == 0) return defaultValue;
    
    float value = valueStr.toFloat();
    if (negative) value = -value;
    
    return value;
}

int parseValue(String command, char axis, int defaultValue) {
    return (int)parseValue(command, axis, (float)defaultValue);
}

void homeMotors(EthernetClient& client) {
    // Simple homing: move until end-stop is hit
    // This is a basic implementation - you may want to add more sophisticated homing
    
    client.println("Homing...");
    
    // Home motor 1
    motor1.setSpeed(-500);  // Move backward
    while (digitalRead(ENDSTOP1_PIN) == HIGH) {
        motor1.runSpeed();
    }
    motor1.setCurrentPosition(0);
    positions[0] = 0;
    
    // Home motor 2
    motor2.setSpeed(-500);
    while (digitalRead(ENDSTOP2_PIN) == HIGH) {
        motor2.runSpeed();
    }
    motor2.setCurrentPosition(0);
    positions[1] = 0;
    
    // Home motor 3
    motor3.setSpeed(-500);
    while (digitalRead(ENDSTOP3_PIN) == HIGH) {
        motor3.runSpeed();
    }
    motor3.setCurrentPosition(0);
    positions[2] = 0;
    
    client.println("Homed");
}

