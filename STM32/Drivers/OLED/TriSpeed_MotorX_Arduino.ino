#define RED_PIN     5
#define GREEN_PIN   6
#define BLUE_PIN    9
#define WHITE_PIN   3
//#define BTN1_PIN    7

bool settingMode = false;
int speedMode = 0;  // 0=呼吸、1=綠、2=黃、3=紅

void setup() {
  pinMode(RED_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);
  pinMode(BLUE_PIN, OUTPUT);
  pinMode(WHITE_PIN, OUTPUT);
  //pinMode(BTN1_PIN, INPUT_PULLUP);

  Serial.begin(115200);
}

void loop() {
  handleSerial();
  //handleButton();

  if (settingMode) {
    blinkWhiteLED();
    showSettingRGB();
  } else {
    digitalWrite(WHITE_PIN, LOW);
    showNormalRGB();
  }
}

void handleSerial() {
  if (Serial.available()) {
    char ch = Serial.read();
    if (ch == 's') {
      settingMode = true;
      speedMode = 0;
    } else if (ch == 'e') {
      settingMode = false;
    } else if (ch >= '0' && ch <= '3') {
      speedMode = ch - '0';
    }
  }
}

/*void handleButton() {
  static unsigned long lastBtnTime = 0;
  if (!settingMode && digitalRead(BTN1_PIN) == LOW) {
    if (millis() - lastBtnTime > 300) {
      speedMode++;
      if (speedMode > 3) speedMode = 0;
      lastBtnTime = millis();
    }
  }
}*/

void blinkWhiteLED() {
  static unsigned long lastToggle = 0;
  static bool ledState = false;
  if (millis() - lastToggle > 500) {
    ledState = !ledState;
    digitalWrite(WHITE_PIN, ledState);
    lastToggle = millis();
  }
}

void showNormalRGB() {
  switch (speedMode) {
    case 0: warmBreathing(); break;
    case 1: rgb(0, 255, 0); break;      // 綠
    case 2: rgb(255, 255, 0); break;    // 黃
    case 3: rgb(255, 0, 0); break;      // 紅
  }
}

void showSettingRGB() {
  static unsigned long lastToggle = 0;
  static bool flashState = false;
  static int breath = 0;
  static int delta = 2;

  if (speedMode == 0) {
    if (millis() - lastToggle > 15) {
      breath += delta;
      if (breath > 255 || breath < 0) delta = -delta;
      breath = constrain(breath, 0, 255);
      rgb(breath / 2, 0, breath);  // 藍紫呼吸燈
      lastToggle = millis();
    }
  } else {
    if (millis() - lastToggle > 300) {
      flashState = !flashState;
      lastToggle = millis();
    }
    if (flashState) {
      switch (speedMode) {
        case 1: rgb(0, 255, 0); break;
        case 2: rgb(255, 255, 0); break;
        case 3: rgb(255, 0, 0); break;
      }
    } else {
      rgb(0, 0, 0);
    }
  }
}

void warmBreathing() {
  static unsigned long lastTime = 0;
  static int b = 0;
  static int delta = 2;

  if (millis() - lastTime > 15) {
    b += delta;
    if (b > 255 || b < 0) delta = -delta;
    b = constrain(b, 0, 255);
    rgb(b, b / 4, 0);  // 橘紅呼吸
    lastTime = millis();
  }
}

void rgb(int r, int g, int b) {
  analogWrite(RED_PIN, r);
  analogWrite(GREEN_PIN, g);
  analogWrite(BLUE_PIN, b);
}
