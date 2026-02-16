#include <Arduino.h>

// Pin Definitions
#define TRIG_FRONT 5
#define ECHO_FRONT 4
#define TRIG_LEFT 7
#define ECHO_LEFT 6
#define TRIG_RIGHT 3
#define ECHO_RIGHT 2

#define ENA 10
#define IN1 9
#define IN2 8
#define ENB 11
#define IN3 13
#define IN4 12

long previousFrontDistance = 0;
long previousLeftDistance = 0;
long previousRightDistance = 0;
int stableCount = 0;
const int stableThreshold = 5;

void setup() {
    pinMode(TRIG_FRONT, OUTPUT);
    pinMode(ECHO_FRONT, INPUT);
    pinMode(TRIG_LEFT, OUTPUT);
    pinMode(ECHO_LEFT, INPUT);
    pinMode(TRIG_RIGHT, OUTPUT);
    pinMode(ECHO_RIGHT, INPUT);

    pinMode(ENA, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(ENB, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);

    Serial.begin(9600);
}

long measureDistance(int trigPin, int echoPin) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    long duration = pulseIn(echoPin, HIGH, 30000);
    long distance = duration * 0.034 / 2;
    return distance;
}

void moveForward() {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENA, 90);
    analogWrite(ENB, 90);
}

void moveBackwardWithRandomAngle() {
    int randomSpeed = random(50, 100); 
    int randomDelay = random(500, 1000); 

    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENA, 90);        
    analogWrite(ENB, randomSpeed); 
    delay(randomDelay); 
}

void turnLeft() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENA, 90);
    analogWrite(ENB, 90);
}

void turnRight() {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENA, 90);
    analogWrite(ENB, 90);
}

void stopMotors() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
}

void loop() {
    long frontDistance = measureDistance(TRIG_FRONT, ECHO_FRONT);
    long leftDistance = measureDistance(TRIG_LEFT, ECHO_LEFT);
    long rightDistance = measureDistance(TRIG_RIGHT, ECHO_RIGHT);

    Serial.print("Front: ");
    Serial.print(frontDistance);
    Serial.print(" cm | Left: ");
    Serial.print(leftDistance);
    Serial.print(" cm | Right: ");
    Serial.println(rightDistance);

    if (frontDistance == previousFrontDistance && 
        leftDistance == previousLeftDistance && 
        rightDistance == previousRightDistance) {
        stableCount++;
    } else {
        stableCount = 0;
    }

    previousFrontDistance = frontDistance;
    previousLeftDistance = leftDistance;
    previousRightDistance = rightDistance;

    if (stableCount > stableThreshold) {
        Serial.println("Stuck detected! Moving Backward with Random Angle...");
        moveBackwardWithRandomAngle();
        delay(1000);  
        stopMotors();
        delay(500);  

        
        long newLeftDistance = measureDistance(TRIG_LEFT, ECHO_LEFT);
        long newRightDistance = measureDistance(TRIG_RIGHT, ECHO_RIGHT);
        long newFrontDistance = measureDistance(TRIG_FRONT, ECHO_FRONT);

        if (newFrontDistance > 20) {
            Serial.println("Space detected ahead, moving forward...");
            moveForward();
            delay(500);
        } else if (newLeftDistance > newRightDistance) {
            Serial.println("Turning Left...");
            turnLeft();
            delay(500);
        } else {
            Serial.println("Turning Right...");
            turnRight();
            delay(500);
        }
        stopMotors();
        delay(500);
    } else if (frontDistance < 20) {
        stopMotors();
        delay(500);
        if (leftDistance > rightDistance) {
            Serial.println("Turning Left...");
            turnLeft();
            delay(300);
        } else {
            Serial.println("Turning Right...");
            turnRight();
            delay(300);
        }
    } else {
        Serial.println("Moving Forward...");
        moveForward();
    }
}
