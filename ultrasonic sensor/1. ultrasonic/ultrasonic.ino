#include <PID_v1.h>



// Define pins for ultrasonic sensor
const int trigPin = 7;
const int echoPin = 6;

// Define variable for the duration and distance
long duration;
int distance;

void setup() {
  // Start the serial communication
  Serial.begin(9600);
  
  // Set the trigPin as an output and echoPin as an input
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
}

void loop() {
  // Clear the trigPin by setting it LOW
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  // Send a 10 microsecond HIGH pulse to trigger the ultrasonic sensor
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Read the pulse duration from echoPin
  duration = pulseIn(echoPin, HIGH);

  // Calculate the distance in centimeters
  distance = duration * 0.0344 / 2; // Speed of sound is 0.0344 cm/µs, divide by 2 for the round trip

  // Print the distance to the Serial Monitor
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  // Wait a short time before measuring again
  delay(500);
}
