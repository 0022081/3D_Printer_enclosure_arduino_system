#include <LiquidCrystal_I2C.h>
#include <DFRobot_DHT11.h>
#include <ezButton.h>

// Define pins
#define DHT11PIN 2        // DHT11/DHT22 data pin
const int potPin1 = A0;   // Potentiometer 1 for LCD selection
const int potPin2 = A1;   // Potentiometer 2 for fan speed adjustment
const int switchPin1 = 4; // Switch for PSU on/off
const int relayPin = 8;   // Relay pin
const int redLedPin = 9;  // Red LED
const int greenLedPin = 10; // Green LED
const int fanPin = 6;    // PWM pin for fan control
const int buzzerPin = 3;  // Buzzer pin for temp control

// Initialize objects
LiquidCrystal_I2C lcd(0x27, 16, 2); // 0x27 is the default I2C address for the LCD
DFRobot_DHT11 DHT;
ezButton toggleSwitch(switchPin1); // Initialize ezButton for the toggle switch

void setup() {
  // Initialize serial communication
  Serial.begin(9600);

  // Initialize LCD
  lcd.begin(16, 2); 
  lcd.backlight();

  // Initialize pins
  pinMode(relayPin, OUTPUT);
  pinMode(redLedPin, OUTPUT);
  pinMode(greenLedPin, OUTPUT);
  pinMode(fanPin, OUTPUT); // Set fan pin as output
  pinMode(buzzerPin, OUTPUT); // Set buzzer pin as output

  // Set initial states
  digitalWrite(relayPin, LOW); // PSU initially off
  digitalWrite(redLedPin, HIGH); // Red LED on (PSU off)
  digitalWrite(greenLedPin, LOW); // Green LED off (PSU off)
  digitalWrite(buzzerPin, LOW); // Buzzer off initially

  // Configure toggle switch
  toggleSwitch.setDebounceTime(50); // Set debounce time to 50 ms

  Serial.println("Setup complete. Waiting for switch input...");
}

void loop() {
  // Update the switch state
  toggleSwitch.loop();

  // Read temperature and humidity
  DHT.read(DHT11PIN);
  float tempC = DHT.temperature;
  float hum = DHT.humidity;

  // Check if any reads failed and exit early (to try again).
  if (isnan(tempC) || isnan(hum)) {
    Serial.println(F("Failed to read from DHT sensor!"));
    return;
  }

  // Check for dangerous temperature
  if (tempC >= 100) {
    Serial.println(F("Dangerous temperature detected! Turning off the printer and activating buzzer."));
    
    // Turn off PSU
    digitalWrite(relayPin, LOW); 
    digitalWrite(redLedPin, HIGH); // Red LED on
    digitalWrite(greenLedPin, LOW); // Green LED off
    
    // Activate buzzer with a warning tone (1 kHz frequency)
    tone(buzzerPin, 1000);
    
    // Display warning on LCD
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("OVERHEAT ALERT!");
    lcd.setCursor(0, 1);
    lcd.print("Temp: ");
    lcd.print(tempC);
    lcd.print(" C");

    delay(1000); // Small delay to stabilize reading
    return; // Skip the rest of the loop
  } else {
    // Turn off buzzer when temperature returns to safe levels
    noTone(buzzerPin);
  }

  // Directly map the switch state to the PSU state
  if (toggleSwitch.getState() == HIGH) { // Switch is open
    digitalWrite(relayPin, HIGH); // PSU on
    digitalWrite(redLedPin, LOW); // Red LED off
    digitalWrite(greenLedPin, HIGH); // Green LED on
    Serial.println("Switch OPEN: PSU ON");
  } else { // Switch is closed
    digitalWrite(relayPin, LOW); // PSU off
    digitalWrite(redLedPin, HIGH); // Red LED on
    digitalWrite(greenLedPin, LOW); // Green LED off
    Serial.println("Switch CLOSED: PSU OFF");
  }

  // Read potentiometer 1 value for LCD selection
  int potValue1 = analogRead(potPin1);

  // Read potentiometer 2 value for fan speed adjustment
  int potValue2 = analogRead(potPin2);

  // Control fan speed using potentiometer 2
  int fanSpeed = map(potValue2, 0, 1023, 0, 255); // Map potentiometer value to PWM range (0-255)
  analogWrite(fanPin, fanSpeed); // Set fan speed using PWM

  // Display data on LCD based on potentiometer 1 value
  static int lastPotValue1 = -1;
  if (potValue1 != lastPotValue1) {
    lastPotValue1 = potValue1;
    lcd.clear();
    if (potValue1 < 341) { // Display temperature
      lcd.setCursor(0, 0);
      lcd.print("Temp: ");
      lcd.print(tempC);
      lcd.print(" C");
    } else if (potValue1 < 682) { // Display humidity
      lcd.print("Hum: ");
      lcd.print(hum);
      lcd.print(" %");
    } else { // Display fan speed as percentage
      lcd.print("Fan Speed:");
      lcd.setCursor(0, 1);
      lcd.print(map(potValue2, 0, 1023, 0, 100));
      lcd.print(" %");
    }
  }

  delay(100); // Small delay for stability
}
