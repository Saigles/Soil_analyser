#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "Adafruit_BME680.h"
#include <Kionix_KX023.h>
#include <MicroNMEA.h>

// ------------------------------
// définitions des pins et adresses
// ------------------------------
#define PIN_ACTIVATION_CAPTEURS   PA13
#define PIN_I2C_SDA               PB7
#define PIN_I2C_SCL               PB8

// adresses i2c des capteurs
#define ADRESSE_BME280            0x76
#define ADRESSE_BME680            0x77
#define ADRESSE_KX023             0x1F

// ports série
#define SERIAL_USB                Serial
#define SERIAL_GNSS               Serial4
#define SERIAL_LORA               Serial1

// autres constantes
#define PRESSION_NIVEAU_MER       (1013.25)
#define INTERVAL_ENVOI            60000  // 60 secondes entre chaque envoi

// ------------------------------
// variables globales
// ------------------------------
unsigned long dernier_envoi = 0;
bool bme680_disponible = false;  // indique si le bme680 est présent

// ------------------------------
// instances des capteurs
// ------------------------------
Adafruit_BME280 bme280;
Adafruit_BME680 bme680;
KX023 kx023(Wire, ADRESSE_KX023);
char tampon_nmea[100];
MicroNMEA nmea(tampon_nmea, sizeof(tampon_nmea));

// ------------------------------
// fonctions utilitaires
// ------------------------------

// fonction pour convertir un tableau d'octets en chaîne hexadécimale
void tableau_vers_chaine(byte tableau[], unsigned int longueur, char chaine[]) {
    for (unsigned int i = 0; i < longueur; i++) {
        byte nib1 = (tableau[i] >> 4) & 0x0F;
        byte nib2 = (tableau[i] >> 0) & 0x0F;
        chaine[i*2+0] = nib1 < 0xA ? '0' + nib1 : 'A' + nib1 - 0xA;
        chaine[i*2+1] = nib2 < 0xA ? '0' + nib2 : 'A' + nib2 - 0xA;
    }
    chaine[longueur * 2] = '\0';
}

// fonction pour afficher les valeurs des capteurs sur le moniteur série
void affiche_valeurs_capteurs() {
    // bme280
    SERIAL_USB.println(F("bme280 readings:"));
    SERIAL_USB.print(F("temperature = "));
    SERIAL_USB.print(bme280.readTemperature());
    SERIAL_USB.println(F(" °c"));
    SERIAL_USB.print(F("pression = "));
    SERIAL_USB.print(bme280.readPressure() / 100.0F);
    SERIAL_USB.println(F(" hpa"));
    SERIAL_USB.print(F("humidite = "));
    SERIAL_USB.print(bme280.readHumidity());
    SERIAL_USB.println(F(" %"));

    // bme680
    if (bme680_disponible && bme680.performReading()) {
        SERIAL_USB.println(F("\nbme680 readings:"));
        SERIAL_USB.print(F("temperature = "));
        SERIAL_USB.print(bme680.temperature);
        SERIAL_USB.println(F(" °c"));
        SERIAL_USB.print(F("gaz = "));
        SERIAL_USB.print(bme680.gas_resistance / 1000.0);
        SERIAL_USB.println(F(" kohms"));
    } else {
        SERIAL_USB.println(F("\nbme680 non disponible - utilisation de valeurs par defaut (0)"));
    }

    // kx023
    float acc_x, acc_y, acc_z;
    kx023.readAsynchronousReadBackAccelerationData(&acc_x, &acc_y, &acc_z);
    SERIAL_USB.println(F("\nkx023 readings:"));
    SERIAL_USB.print(F("accel: x="));
    SERIAL_USB.print(acc_x);
    SERIAL_USB.print(F(" y="));
    SERIAL_USB.print(acc_y);
    SERIAL_USB.print(F(" z="));
    SERIAL_USB.println(acc_z);

    SERIAL_USB.println();
}

// fonction pour envoyer les données via lora
bool envoi_lora() {
    unsigned char donnees[32];
    int index = 0;

    // bme280 : lecture de la temperature, de l'humidite et de la pression
    int16_t temperature = (int16_t)(bme280.readTemperature() * 100);
    uint8_t humidite = (uint8_t)(bme280.readHumidity());
    uint16_t pression = (uint16_t)(bme280.readPressure() / 100.0F);

    donnees[index++] = temperature >> 8;
    donnees[index++] = temperature & 0xFF;
    donnees[index++] = humidite;
    donnees[index++] = pression >> 8;
    donnees[index++] = pression & 0xFF;

    // bme680 : envoi de 0 si non disponible
    if (bme680_disponible && bme680.performReading()) {
        uint16_t gaz = (uint16_t)(bme680.gas_resistance / 1000.0);
        donnees[index++] = gaz >> 8;
        donnees[index++] = gaz & 0xFF;
    } else {
        donnees[index++] = 0;
        donnees[index++] = 0;
    }

    // kx023 : lecture des accelerations et conversion
    float ax, ay, az;
    kx023.readAsynchronousReadBackAccelerationData(&ax, &ay, &az);
    int8_t x = (int8_t)(50 * ax);
    int8_t y = (int8_t)(50 * ay);
    int8_t z = (int8_t)(50 * az);
    donnees[index++] = x;
    donnees[index++] = y;
    donnees[index++] = z;

    // conversion du tableau en chaine hexadécimale
    char chaine[64];
    tableau_vers_chaine(donnees, index, chaine);

    // construction de la commande AT pour lora
    String commande = "AT+SEND=1,0,8,0," + String(chaine) + "\r\n";
    
    // nettoyage du buffer série de lora
    while (SERIAL_LORA.available()) {
        SERIAL_LORA.read();
    }

    SERIAL_LORA.print(commande);
    SERIAL_USB.println(F("envoi de la commande: "));
    SERIAL_USB.println(commande);

    delay(1000);
    
    // lecture et affichage des réponses du module lora
    while (SERIAL_LORA.available()) {
        String reponse = SERIAL_LORA.readStringUntil('\n');
        SERIAL_USB.println("reponse: " + reponse);
    }

    return true;
}

void setup() {
    // configuration des pins
    pinMode(PIN_ACTIVATION_CAPTEURS, OUTPUT);
    digitalWrite(PIN_ACTIVATION_CAPTEURS, HIGH);

    // initialisation de l'i2c
    Wire.setSDA(PIN_I2C_SDA);
    Wire.setSCL(PIN_I2C_SCL);
    Wire.begin();
    delay(200);

    // initialisation du port serie usb
    SERIAL_USB.begin(115200);
    while (!SERIAL_USB);
    SERIAL_USB.println(F("\ndemarrage du terminal echostar..."));

    // initialisation du bme280
    unsigned status = bme280.begin(ADRESSE_BME280);
    if (!status) {
        SERIAL_USB.println(F("bme280 non trouve!"));
        while (1) delay(10);
    }
    SERIAL_USB.println(F("bme280 initialise"));

    // initialisation du bme680
    if (!bme680.begin(ADRESSE_BME680)) {
        SERIAL_USB.println(F("bme680 non trouve - poursuite sans lui"));
        bme680_disponible = false;
    } else {
        bme680_disponible = true;
        bme680.setTemperatureOversampling(BME680_OS_8X);
        bme680.setHumidityOversampling(BME680_OS_2X);
        bme680.setPressureOversampling(BME680_OS_4X);
        bme680.setIIRFilterSize(BME680_FILTER_SIZE_3);
        bme680.setGasHeater(320, 150);
        SERIAL_USB.println(F("bme680 initialise"));
    }

    // initialisation du kx023
    if (!kx023.begin()) {
        SERIAL_USB.println(F("kx023 non trouve!"));
        while (1) delay(10);
    }
    kx023.configAsynchronousReadBackAccelerationData(KX023_ACCLERATION_RANGE_2G, KX023_ODR_25HZ);
    kx023.setOperatingMode();
    SERIAL_USB.println(F("kx023 initialise"));

    // initialisation de la liaison lora
    SERIAL_LORA.begin(115200);
    // reset du module
    ECHOSTAR_SERIAL.println("AT+RST");
    delay(1000); // pause pour s'assurer que le module se réinitialise

    // définir la région (ex : europe)
    ECHOSTAR_SERIAL.println("AT+REGION=EU868");
    delay(200);

    // rejoindre le réseau
    ECHOSTAR_SERIAL.println("AT+JOIN");
    delay(1000); // pause pour laisser le temps de joindre

    SERIAL_USB.println(F("lora initialise"));
}

void loop() {
    unsigned long temps_courant = millis();

    if (temps_courant - dernier_envoi >= INTERVAL_ENVOI) {
        affiche_valeurs_capteurs();
        
        if (envoi_lora()) {
            SERIAL_USB.println(F("donnees envoyees avec succes"));
        } else {
            SERIAL_USB.println(F("echec de l'envoi des donnees"));
        }
        
        dernier_envoi = temps_courant;
    }

    delay(100);
}
