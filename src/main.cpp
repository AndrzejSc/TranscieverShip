#include <Arduino.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <RF24.h>


#include <HMC5983.h>
#include <Wire.h>
#include <Vcc.h>

#include <SPI.h>
//#include <nRF24L01.h>
//#include <MPU6050.h>
//#include <Adafruit_BMP085.h>
//#include <EEPROM.h>


/**************************************************************************
 *          FUNKCJE
 *************************************************************************/


void radioSetup();                  /// Ustawienia dla radia
void readOffsets();                 /// Funkcja do wczytania wartości offsetu z EEPROM
void mpuSetup();                    /// Uruchomienie żyroskopu
void hmcSetup();                    /// Uruchomienie magnetometru
void bmpSetup();                    /// Uruchomienie i ustawienie czujnika temp i cisnienia
void pinSetup();                    /// Funkcja ustawia odpowiednie porty jako we/wy
void calibrate();                   /// Kalibracja magnetometru
void doCalibrate();                 /// Funkcja uruchamia kalibrację oraz zapisuje nowe dane do EEPROM
int getAzimuth();                   /// Odczytanie i obliczenie azymutu
void getTempPress();                /// Odczytanie temperatury i cisnienia
void setToZero();                   /// Ustawienie korekcji aby azymut wskazywał północ
void getGPS();                      /// Pobranie, analiza i przypisanie danych z GPS
void displayInfo();                 /// Wyświetlenie info w SerialMonitor
void sendInfo();                    /// Wysyła info do odbiornika na brzegu
void getCommand();                  /// Odbiera komendy od odbiornika na brzegu
void parseCommand();                /// Odczytuje co takiego wysłano nam z brzegu
void doCommand();                   /// Sprawdza, czy zmieniły się PINy i ewentualnie dokonuje zmian
void doPortCommand();                   /// Sprawdza, czy zmieniły się PINy i ewentualnie dokonuje zmian



/**************************************************************************
 *          Zmienne globlane, struktury
 *************************************************************************/


// Struktura zawierająca dane do wysłania
struct geoloc {
    double lat;
    double lon;
    uint8_t sats;
    int timeHour;
    int timeMin;
    int timeSec;
    float vccShip;          //napięcie na baterii łodzi
    int azymut;
    //uint8_t temperature;
    //int32_t pressure;

};
geoloc currentLoc;                          // zmienna zawierająca aktualne współrzędne

// Struktura zawierająca ustawienia pinów i wartości stanów portów ABCD
struct PinMap {
    // Przypisanie do poszczególnych portów ABCD, odpowiednich pinów arduino
    const uint8_t pinA = A0;
    const uint8_t pinB = A1;
    const uint8_t pinC = A2;
    const uint8_t pinD = A3;
    const uint8_t pinPWM1 = 3;
    const uint8_t pinPWM2 = 5;
    const uint8_t pinPWM3 = 6;

    bool A, B, C, D = 0;                        // 5 zmiennych oznaczających stan aktualny pinów
    uint8_t PWM1, PWM2, PWM3;                   // 3 zmienne do wysyłania stanów od 0 do 255



    //TODO: Wolne piny: 2,3 (PWM) 5(PWM),6(PWM), A0, A1, A2, A3, A6, A7

// PWM: 3,5,6,9,10,11

};
PinMap pinMap;

struct Command{                              // Zmienne przechowujące dane odebrane
    bool A, B, C, D = 0;                     // 5 zmiennych do wysyłania stanów 1 lub 0
    uint8_t PWM_1, PWM_2, PWM_3 = 0;         // 3 zmienne do wysyłania stanów od 0 do 100
};
Command command;

//Zmienne do kalibracji
//int minX, maxX, minY, maxY, offX, offY = 0;
//int offXset;                                // Wartość offsetu X - przesunięcie względem osi X
//int offYset;                                // Wartość offsetu Y - przesunięcie względem osi y

// Zmiennne do ustalenia kąta
int previousDegree;                         // Zmienna przechowująca poprzedni azymut
int_fast8_t korekcja;                               // Korekcja do ustawienia północy
bool isGPSlocation = false;                 // flaga informująca czy mamy poprawne dane GPS

//Zmienne od obliczania napięcia baterii
const float vccMin = 3.3;                   // najmniejszy zakres napięcia
const float vccMax = 4;                     // największy zakres napięcia
const float vccCorrection = 4.0 / 4.0;        // Napięcie zmierzone multimetrem/ odczytane przez arduino

// Buffor do odebrania komend
const uint8_t bufLen = 3;                  // Ilość znaków w wiadomości do wysłania
char commandBuf[bufLen];                    // Bufor do wysłania wiadomości

//Zmienne inne
uint8_t calibrateLed = 4;                   //dioda do informowania o kalibracji


/**************************************************************************
 *          OBIEKTY
 *************************************************************************/
HMC5983 compass;                           // Obiekt do obsługi magnetometru (kompas)
//MPU6050 mpu;                                // Obiekt do obsługi żyroskopu i akcelerometru
RF24 radio(7, 8);                           // Obiekt do obsługi radia nRD (pin CE, pin CSE)
TinyGPSPlus gps;                            // Obiekt do obsługi GPS
SoftwareSerial serialGPS(9, 10);            // Obiekt do komunikacji z GPS (RX,TX)
//Adafruit_BMP085 bmp;                        // Obiekt do odczytu temperatury i cisnienia
Vcc vcc(vccCorrection);                     // Obiekt do odczytu napięcia baterii

/**************************************************************************
 *          AKCJA !!!
 *************************************************************************/
void setup() {

    Serial.begin(9600);                 // Uruchomienie Serial portu
    serialGPS.begin(9600);              // Uruchomienie komunikacji szeregowej dla GPS

    pinSetup();                         // Ustawienie pinów jako wejścia / wyjścia
    radioSetup();                       // Wywyołanie funkcji do ustawienia radia
    // readOffsets();                      // Wczytanie wartości offsetu z EEPROM
    // mpuSetup();                         // Uruchomienie zyroskopu
    hmcSetup();                         // Uruchomienie magnetometru
    //bmpSetup();                         // Uruchomienie czujnika temp i cisnienia

    korekcja = (-50);
}

void loop() {
    //Serial.println("Funkcja loop");

    //Serial.print("test GET AZIMUTH...");
    currentLoc.azymut = getAzimuth();

    //Serial.println(currentLoc.azymut);
    //Serial.println(" OK");

    //Serial.print("test GPS...");
    getGPS();
    //Serial.println(" OK");

    //getTempPress();

    //currentLoc.vccShip = floor((double) vcc.Read_Perc(vccMin,vccMax));
    currentLoc.vccShip = vcc.Read_Volts();
    //Serial.println("test vcc read OK");

    // displayInfo();
    // Serial.println("powrot z sendInfo");

    //Serial.print("test Send info... ");
    sendInfo();
    //Serial.println(" OK");

    //Serial.println("powrot z sendInfo");
    doCommand();
    delay(400);

}

/**************************************************************************
 *          radioSetup()
 *          Ustawienie opcji dla modułu nRF, uruchomienie modułu
 *************************************************************************/
void radioSetup() {
    //Ustawienia dla radia
    const uint64_t pipeOut = 0xf0f0f0f0e1;
    const uint64_t pipeIn = 0xf0f0f0f0d1;
    radio.begin();
    radio.setDataRate(RF24_250KBPS); //RF24_1MBPS,RF24_2MBPS,RF24_250KBPS
    radio.setPALevel(RF24_PA_HIGH);
    radio.setChannel(19);

    // radio.setAutoAck(true);
    radio.enableAckPayload();
    //radio.setRetries(2, 5);

    radio.openWritingPipe(pipeOut);
    radio.openReadingPipe(1, pipeIn);
    //radio.startListening();

    //memset(&data, 0, sizeof(MyData)); //wypełnienie zerami struktury data

    Serial.print("Radio check...");
    delay(100);
    Serial.println("done.");
    return;
}

/**************************************************************************
 * pinSetup()
 * Ustawienie pinów jako odpowienio wejścia i wyjścia
 *************************************************************************/
void pinSetup() {
//    pinMode(calibrateLed, OUTPUT);
//    digitalWrite(calibrateLed, LOW);

    // Ustawianie pinów A,B,C,D jako wyjścia i ustawienie stanów początkowych
    pinMode(pinMap.pinA, OUTPUT);
    digitalWrite(pinMap.pinA, (pinMap.A ? HIGH : LOW));

    pinMode(pinMap.pinB, OUTPUT);
    digitalWrite(pinMap.pinB, (pinMap.B ? HIGH : LOW));

    pinMode(pinMap.pinC, OUTPUT);
    digitalWrite(pinMap.pinC, (pinMap.C ? HIGH : LOW));

    pinMode(pinMap.pinD, OUTPUT);
    digitalWrite(pinMap.pinD, (pinMap.D ? HIGH : LOW));

    //pinMode(pinMap.pinPWM1,OUTPUT);
    //pinMode(pinMap.pinPWM2,OUTPUT);
    //pinMode(pinMap.pinPWM3,OUTPUT);
}

/**************************************************************************
 * readOffsets()
 * Wczytanie wartości offsetów z pamięci EEPROM
 *************************************************************************/
void readOffsets() {
//    EEPROM.get(0,offXset);
//    EEPROM.get(sizeof(int),offYset);
//    Serial.print("Wczytano offsetX: "); Serial.print(offXset);
//    Serial.print(" offsetY: "); Serial.println(offYset);

}

/**************************************************************************
 * mpuSetup() - Uruchomienie i ustawienie opcji dla modułu akcelerometru
 *************************************************************************/
//void mpuSetup() {
//    // Initialize MPU6050
//    Serial.print("Uruchamiam zyroskop...");
//    while (!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G)) {
//        delay(500);
//    }
//    Serial.println(" done.");
//
//    // Enable bypass mode dla zyroskopu
//    mpu.setI2CMasterModeEnabled(false);
//    mpu.setI2CBypassEnabled(true);
//    mpu.setSleepEnabled(false);
//}

/**************************************************************************
 * hmcSetup() - Ustawienie opcji dla modułu magnetometru, uruchomienie modułu
 *************************************************************************/
void hmcSetup() {

    // Dla HMC5983
    compass.begin();





    // Uruchamiam HMC588L - magnetometr
//    Serial.print("Uruchamiam magnetometr...");
//    while (!compass.begin()) {
//        Serial.println("Nie odnaleziono magnetometru. Sprawdz polacznie!");
//        delay(500);
//    }
//    Serial.println(" done.");

    //Ustawienia magnetometru HMC5883L
    //compass.setRange(HMC5883L_RANGE_1_3GA);   // Ustawienie zakresu pomiaroergo
    //compass.setMeasurementMode(HMC5883L_CONTINOUS); //Praca ciągła
    //compass.setDataRate(HMC5883L_DATARATE_30HZ); //Czestotliwosc pomiarow
    //compass.setSamples(HMC5883L_SAMPLES_8); //Liczba usrednionych probek
}

/**************************************************************************
 * bmpSetup() - Ustawienie opcji dla modułu pomiaru temperatury i cisnienia
 *************************************************************************/
void bmpSetup() {
//    Serial.print("Uruchamiam czujnik temperatury i cisnienia...");
//    if (!bmp.begin()) {
//        Serial.println("Nie odnaleziono czujnika BMP085 / BMP180");
//        return ;
//    }
//    Serial.println(" done.");
}

/**************************************************************************
 * displayInfo() - Wyświetlenie danych na Serial Monitor
 *************************************************************************/
void displayInfo() {
    // Wyświetlenie danych z czujnika temp i ciśnienia
    //Serial.print("T:");
    //Serial.print(currentLoc.temperature);
    // Serial.print(" P:");
    // Serial.print(currentLoc.pressure);

    // Wyświetlenie położenia GPS
    Serial.print(F("L:"));
    Serial.print(gps.location.lat());
    Serial.print(F(","));
    Serial.print(gps.location.lng());

    // Wyświetlenie liczby satelit
    Serial.print(F(" S:"));
    Serial.print(gps.satellites.value());

    // Wyświetlenie czasu
    Serial.print(F(" T:"));
    if (currentLoc.timeHour < 10) Serial.print(F("0"));
    Serial.print(currentLoc.timeHour);
    Serial.print(F(":"));
    if (currentLoc.timeMin < 10) Serial.print(F("0"));
    Serial.print(currentLoc.timeMin);
    Serial.print(F(":"));
    if (currentLoc.timeSec < 10) Serial.print(F("0"));
    Serial.print(currentLoc.timeSec);

    //Informacje z Magnetometru
    Serial.print(F(" A:"));
    Serial.print(currentLoc.azymut);

    //Informacje z stanie baterii
    Serial.print(F(" VS:"));
    Serial.println(currentLoc.vccShip);

}

/**************************************************************************
 * doCalibrate()
 * Funkcja uruchamia dwukrotnie proces kalibracji oraz zapisuje
 * obliczone offsety do pamięci EEPROM
 *************************************************************************/
void doCalibrate() {
//    Serial.print("Aktualny Xoffset: "); Serial.print(offXset);
//    Serial.print(" Yoffset: "); Serial.print(offYset);
//
//    calibrate();
//    // Dla zwiększenia dokładności, przeprowadzana jest powtórna kalibracja
//    calibrate();
//
//    // Zapis do EEPROM
//    EEPROM.put(0,offXset);
//    EEPROM.put(sizeof(int),offYset);


}

/**************************************************************************
 *          calibrate()
 * Funkcja oblicza offsety i kalibruje magnetometr. Uruchamia się na czas 10s.
 * W tym czasie należy obracać urządzeniem w zakresie 360 stopni
 *************************************************************************/
void calibrate() {

    /// Dla HMC5883L
//    digitalWrite(calibrateLed, LOW);
//    delay(700);
//
//    Serial.print("Uruchamiam kalibracje za: ");
//    Serial.print("3 ");
//    digitalWrite(calibrateLed, HIGH);
//    delay(100);
//    digitalWrite(calibrateLed, LOW);
//    delay(900);
//
//    Serial.print("2 ");
//    digitalWrite(calibrateLed, HIGH);
//    delay(100);
//    digitalWrite(calibrateLed, LOW);
//    delay(900);
//
//    Serial.println("1 ");
//    digitalWrite(calibrateLed, HIGH);
//    delay(100);
//    digitalWrite(calibrateLed, LOW);
//    delay(900);
//
//    minX = 0;
//    maxX = 0;
//    minY = 0;
//    maxY = 0;
//    offX = 0;
//    offY = 0;
//
//    unsigned long time;
//    time = millis();        //aktualny czas wywołania funkcji milis()
//    uint8_t i = 1;          //zmienna pomocnicza do migania
//    Serial.print("Kalibracja: ");
//
//    while (millis() - time < 10000) {
//
//        // jeśli aktualny czas - czas uruchomienia jest wielokrotnością 200ms, zmień stan diody na przeciwny oraz
//        //zwiększ licznik i
//        if ((millis() - time) > 50 * i) {
//            Serial.print(".");
//            //delay(100);
//            digitalWrite(calibrateLed, i%2);
//            i++;
//        }
//
//        Vector mag = compass.readRaw();
//        // Determine Min / Max values during 10s
//        if (mag.XAxis < minX) minX = (int) mag.XAxis;
//        if (mag.XAxis > maxX) maxX = (int) mag.XAxis;
//        if (mag.YAxis < minY) minY = (int) mag.YAxis;
//        if (mag.YAxis > maxY) maxY = (int) mag.YAxis;
//
//        // Calculate offsets
//        offX = (maxX + minX) / 2;
//        offY = (maxY + minY) / 2;
//
//        //Serial.print(offX);
//        //Serial.print(":");
//        //Serial.print(offY);
//        //Serial.print("\n");
//    }
//    digitalWrite(calibrateLed, LOW);        //zgaś diodę, jeśli była by włączona
//
//    //Obliczenie offsetów X i Y, z obu wywołań funkcji
//    offXset = offXset + offX;
//    offYset = offYset + offY;
//
//    //Ustawienie offsetu
//    compass.setOffset(offXset, offYset);
//    Serial.print("Ustawiam offset X:");
//    Serial.print(offXset);
//    Serial.print(" Y:");
//    Serial.println(offYset);
    return;
}

/**************************************************************************
 * getAzimuth()
 * Funkcja oblicza azymut, bez korekcji do północy
 * Zwraca int azymut
 *************************************************************************/
int getAzimuth() {


    /// Dla HMC5883L
//    float heading;
//    Vector mag = compass.readNormalize();
//    Serial.println("test1");
//    heading = atan2(mag.YAxis, mag.XAxis);
    //########################################################  Compensate Pitch & Roll
//    float roll;
//    float pitch;
//    Vector acc = mpu.readScaledAccel();
//    roll = asin(acc.YAxis);
//    pitch = asin(-acc.XAxis);
//
//    if (roll > 0.78 || roll < -0.78 || pitch > 0.78 || pitch < -0.78) {
//        heading = -1000;
//    } else {
//        // Some of these are used twice, so rather than computing them twice in the algorithem we precompute them before hand.
//        float cosRoll = cos(roll);
//        float sinRoll = sin(roll);
//        float cosPitch = cos(pitch);
//        float sinPitch = sin(pitch);
//
//        // Tilt compensation
//        float Xh = mag.XAxis * cosPitch + mag.ZAxis * sinPitch;
//        float Yh = mag.XAxis * sinRoll * sinPitch + mag.YAxis * cosRoll - mag.ZAxis * sinRoll * cosPitch;
//
//        heading = atan2(Yh, Xh);
//    }
//    if (heading == -1000) {
//        heading = atan2(mag.YAxis, mag.XAxis);
//    }



//
//
//    // Set declination angle on your location and fix heading
//    // You can find your declination on: http://magnetic-declination.com/
//    // (+) Positive or (-) for negative
//    // For Bytom / Poland declination angle is 4'26E (positive)
//    // Formula: (deg + (min / 60.0)) / (180 / M_PI);
//    float declinationAngle = (4.0 + (26.0 / 60.0)) / (180 / M_PI);
//    heading += declinationAngle;
//    heading += (korekcja * M_PI / 180);
//    // Correct for heading < 0deg and heading > 360deg
//    if (heading < 0) {
//        heading += 2 * PI;
//    }
//    if (heading > 2 * PI) {
//        heading -= 2 * PI;
//    }
//
//    Serial.println("test2");
//    /// #######################################         MAPOWANIE tutaj jeśli bedzie potrzebne
//
//    // Convert to degrees
//    heading = (int) (heading * 180 / M_PI);
////Serial.print(heading); Serial.print(" ");
//
//    if (heading < (previousDegree + 10) && heading > (previousDegree - 10)) {
//        heading = previousDegree;
//    } else {
//        previousDegree = (int) heading;
//    }

    int heading, temp;
    //Serial.print("..");
    temp = (int) floor(compass.read());
    //Serial.print(" OK ");
    if (temp != -999) {
        heading = temp;
    } else {
        heading = currentLoc.azymut;
        //Serial.println("INCORRECT HEADING!");
    }

    return heading;
}

/**************************************************************************
 * NIE ZAIMPLEMENTOWANO
 *
 *          setToZero()
 *          Funkcja koryguje wartość azymutu, tak aby wartość 0 = ~północ
 *
 * Zmienia wartosć 'azymut' w struct geoloc
 * Zmienna korekcja ustawiana narazie ręcznie
 *************************************************************************/
void setToZero() {
    // TODO: Korekcja ustawiana przez aplikacje
    int korekcja = (-80);
    currentLoc.azymut = +korekcja;
    return;
}

/**************************************************************************
 * getGPS()
 * Odczytanie, analiza i przypisanie zmiennych do struktury geoloc
 *************************************************************************/
void getGPS() {
    //Serial.println("test GPS 1");
    if (isGPSlocation) {
        digitalWrite(calibrateLed, HIGH);
    } else digitalWrite(calibrateLed, LOW);
    // Serial.println("test GPS 2");

    while (serialGPS.available() > 0) {

        //Serial.println(F("Port Dostepny"));
        // char inByte = serialGPS.read();
        //Serial.print(inByte);

        if (gps.encode(serialGPS.read())) {
            if (millis() > 5000 && gps.charsProcessed() < 10) {
                Serial.println(F("No GPS detected: check wiring."));
                return;
            }
        }
    }

    // Pobranie aktualnych współrzędnych
    if (gps.location.isValid()) {
        currentLoc.lat = gps.location.lat();
        currentLoc.lon = gps.location.lng();
        isGPSlocation = true;
    } else {
        currentLoc.lat = 0;
        currentLoc.lon = 0;
        isGPSlocation = false;
    }

    // Pobranie liczby satelit
    if (gps.satellites.isValid()) {
        currentLoc.sats = gps.satellites.value();
    } else currentLoc.sats = 0;

    // Pobranie czasu z satelit
    if (gps.time.isValid()) {
        currentLoc.timeHour = gps.time.hour();
        currentLoc.timeMin = gps.time.minute();
        currentLoc.timeSec = gps.time.second();
    }// else Serial.println(F("INVALID TIME"));
    return;

}

/**************************************************************************
 * getTempPress()
 * Odczytanie temperatury i cisnienia, zapisanie do struktury currentLoc
 *************************************************************************/
void getTempPress() {
//    currentLoc.temperature = (uint8_t)bmp.readTemperature();
//    currentLoc.pressure = bmp.readPressure();

}

/**************************************************************************
 * sendInfo()
 * Wysłanie danych na brzeg do odbiornika
 *************************************************************************/
void sendInfo() {

    radio.flush_tx();
    delay(20);
    radio.write(&currentLoc, sizeof(geoloc));
    //Serial.println("Dane wyslane.");
    delay(20);

    radio.startListening();
    delay(20);

    // Jeśli mamy coś w pakiecie odbierającym
    // TODO Sprawdzić czy działa radio.available() zamiast isAck...
    //if (radio.isAckPayloadAvailable()) {
    if(radio.available()){
        //Serial.println(radio.getPayloadSize());
        //memset(&commandBuf,0, sizeof(commandBuf));

        radio.read(&command, sizeof(command));

        Serial.print("Command: A"); Serial.print(command.A);
        Serial.print(" B"); Serial.print(command.B);
        Serial.print(" C"); Serial.print(command.C);
        Serial.print(" D"); Serial.print(command.D);
        Serial.print(" P1:"); Serial.print(command.PWM_1);
        Serial.print(" P2:"); Serial.print(command.PWM_2);
        Serial.print(" P3:"); Serial.println(command.PWM_3);

        //parseCommand();
    }
    radio.stopListening();

    delay(20);


    return;
}

/*************************************************************************
 * getCommand()
 * odebranie i analiza komend z brzegu
 *************************************************************************/
void getCommand() {
    //TODO: Odebranie komend z brzegu
    radio.startListening();
    delay(20);
    //Serial.println("Listening radio...");
    if (radio.available()) {
        radio.read(&commandBuf, sizeof(commandBuf));
        Serial.print("COMMAND: ");
        Serial.println(commandBuf);
    } else {
        //sendInfo();
    }
    Serial.println(radio.available());
    //Serial.println("Listening radio2...");
    //radio.stopListening();
    delay(20);
    return;
}

/**************************************************************************
 * parseCommand()
 * analiza komend z brzegu i zapis nowych wartości do zmiennych
 *************************************************************************/
/*
void parseCommand() {
/// A1|B0|C1|D1|E0|P1:111|P2:222|P3:333|KAL

    Serial.print("Parsuje: ");
    Serial.println(commandBuf);
    uint8_t value;
    char *command = strtok(commandBuf, "|");
    while (command != NULL) {
        //Serial.println(command);
        switch (command[0]) {
            case 'A':
                value = (uint8_t) atoi(command + 1);
                pinMap._A = (bool) value;

                Serial.print("PIN A:");
                Serial.print(value);
                break;
            case 'B':
                value = (uint8_t) atoi(command + 1);
                pinMap._B = (bool) value;

                Serial.print("PIN B:");
                Serial.print(value);
                break;
            case 'C':
                value = (uint8_t) atoi(command + 1);
                pinMap._C = (bool) value;

                Serial.print("PIN C:");
                Serial.print(value);
                break;
            case 'D':
                value = (uint8_t) atoi(command + 1);
                pinMap._D = (bool) value;

                Serial.print("PIN D:");
                Serial.print(value);
                break;

            default:
                if (!strncmp(command, "P1", 2)) {
                    //Serial.print( atoi(strchr(command, ':') + 1));
                    value = (uint8_t) atoi(strchr(command, ':') + 1);
                    pinMap._PWM1 = value;

                    Serial.print("PWM 1:");
                    Serial.print((int) value); //Serial.print(command);
                    break;
                } else if (!strncmp(command, "P2", 2)) {
                    value = (uint8_t) atoi(strchr(command, ':') + 1);
                    pinMap._PWM2 = value;

                    Serial.print("PWM 2:");
                    Serial.print(value);
                    break;
                } else if (!strncmp(command, "P3", 2)) {
                    value = (uint8_t) atoi(strchr(command, ':') + 1);
                    pinMap._PWM3 = value;

                    Serial.print("PWM 3:");
                    Serial.print(value); //Serial.print(command);
                    break;
                } else if (!strncmp(command, "KAL", 3)) {
                    //value = atoi(strchr(command,':')+1);
                    Serial.print("Kalibracja");
                    doCalibrate();
                    break;
                } else {
                    Serial.print("COMMAND INCORRECT:");
                    Serial.print(command);
                }
                break;

        }
        Serial.println();
        command = strtok(NULL, "|");
    }
    doCommand();
}
*/

/**************************************************************************
 * doCommand()
 * sprawdza, czy zmieniły sie stany pinów po otrzymaniu komendy
 *************************************************************************/
void doCommand() {
    // Sprawdzić czy się zmieniły piny i doda
    if (command.A != pinMap.A) {
        pinMap.A = command.A;

        // warunek ? true : false
        pinMap.A ? digitalWrite(pinMap.pinA, HIGH) : digitalWrite(pinMap.pinA, LOW);        // Zmiana stanu pinu A
    }

    if (command.B != pinMap.B) {
        pinMap.B = command.B;
        pinMap.B ? digitalWrite(pinMap.pinB, HIGH) : digitalWrite(pinMap.pinB, LOW);        // Zmiana stanu pinu B
    }

    if (command.C != pinMap.C) {
        pinMap.C = command.C;
        pinMap.C ? digitalWrite(pinMap.pinC, HIGH) : digitalWrite(pinMap.pinC, LOW);        // Zmiana stanu pinu C
    }

    if (command.D != pinMap.D) {
        pinMap.D = command.D;
        pinMap.D ? digitalWrite(pinMap.pinD, HIGH) : digitalWrite(pinMap.pinD, LOW);        // Zmiana stanu pinu D
    }

    if (command.PWM_1 != pinMap.PWM1) {
        pinMap.PWM1 = command.PWM_1;
        // PWM1 otrzymujemy z zakresu 0-100, dlatego musimy pomnożyć aby było z zakresu 0-255
        analogWrite(pinMap.pinPWM1, (int) (pinMap.PWM1 * 2.55));                                            // Zmiana wartośći PWM1
    }

    if (command.PWM_2 != pinMap.PWM2) {
        pinMap.PWM2 = command.PWM_2;
        analogWrite(pinMap.pinPWM2, (int) (pinMap.PWM2 * 2.55));                                            // Zmiana wartośći PWM2
    }

    if (command.PWM_3 != pinMap.PWM3) {
        pinMap.PWM3 = command.PWM_3;
        analogWrite(pinMap.pinPWM3, (int) (pinMap.PWM3 * 2.55));                                            // Zmiana wartośći PWM3
    }
}

/**************************************************************************
 * doCommand()
 * sprawdza, czy zmieniły sie stany pinów po otrzymaniu komendy
 *************************************************************************/
void doPortCommand() {

}
