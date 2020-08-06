
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <TinyGPS++.h>
#include <axp20x.h>

double latu, lon, alt;
TinyGPSPlus gps;
HardwareSerial GPS(1);
AXP20X_Class axp;
uint8_t txBuffer[9];
uint32_t LatitudeBinary, LongitudeBinary;
uint16_t altitudeGps;
uint8_t hdopGps;



static const u1_t PROGMEM APPEUI[8]= { 0xB2, 0x5F, 0x02, 0xD0, 0x7E, 0xD5, 0xB3, 0x70 };  // APP EUI fornecido pela TTN, colocar em formato de LSB
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}

static const u1_t PROGMEM DEVEUI[8]= { 0x75, 0x3B, 0xB6, 0x79, 0xDE, 0xF1, 0xD0, 0x00 };// DEV EUI fornecido pela TTN, colocar em formato de LSB
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}


static const u1_t PROGMEM APPKEY[16] ={ 0xA3, 0x2D, 0xA6, 0x78, 0x92, 0x2B, 0x03, 0xA8, 0xA3, 0x20, 0x6F, 0x90, 0xFE, 0x0B, 0xD8, 0x30 }; // APP KEY FORNECIDO PELA TTN EM MSB
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}

static osjob_t sendjob;

const unsigned TX_INTERVAL = 10; 

// Pinmap ESP32
const lmic_pinmap lmic_pins = {
    .nss = 18,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 14,
    .dio = {26, 33,32},
};

void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
            {
              u4_t netid = 0;
              devaddr_t devaddr = 0;
              u1_t nwkKey[16];
              u1_t artKey[16];
              LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey); // FUNÇÃO PARA OBTER AS KEYS DE SESSÃO.
              Serial.print("netid: ");
              Serial.println(netid, DEC);
              Serial.print("devaddr: ");
              Serial.println(devaddr, HEX);
              Serial.print("artKey: ");
              for (int i=0; i<sizeof(artKey); ++i) {
                Serial.print(artKey[i], HEX);
              }
              Serial.println("");
              Serial.print("nwkKey: ");
              for (int i=0; i<sizeof(nwkKey); ++i) {
                Serial.print(nwkKey[i], HEX);
              }
              Serial.println("");
            }
      Serial.println(F("Successful OTAA Join..."));

            LMIC_setLinkCheckMode(0); // Desativa a verificação de validade do link com a rede, só ativar no inicio de uma sessão
            break;

        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
              Serial.print(F("Received "));
              Serial.print(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
            }
            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            break;

        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;

        case EV_TXSTART:
            Serial.println(F("EV_TXSTART"));
            break;
        default:
            Serial.print(F("Unknown event: "));
            Serial.println((unsigned) ev);
            break;
    }
}

void do_send(osjob_t* j){
    // Verifica se não está ocorrendo uma transmissão no momento TX/RX
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {

if (gps.location.isUpdated())
  {
      latu = gps.location.lat();
      lon=gps.location.lng();
  }
if (gps.altitude.isUpdated())
    LatitudeBinary = ((gps.location.lat() + 90) / 180) * 16777215;
    LongitudeBinary = ((gps.location.lng () + 180) / 360) * 16777215;

    txBuffer[0] = ( LatitudeBinary >> 16 ) & 0xFF;
    txBuffer[1] = ( LatitudeBinary >> 8 ) & 0xFF;
    txBuffer[2] = LatitudeBinary & 0xFF;

    txBuffer[3] = ( LongitudeBinary >> 16 ) & 0xFF;
    txBuffer[4] = ( LongitudeBinary >> 8 ) & 0xFF;
    txBuffer[5] = LongitudeBinary & 0xFF;

    altitudeGps = gps.altitude.meters();
    txBuffer[6] = ( altitudeGps >> 8 ) & 0xFF;
    txBuffer[7] = altitudeGps & 0xFF;

    hdopGps = gps.hdop.hdop() * 10;
    txBuffer[8] = hdopGps & 0xFF;


    LMIC_setTxData2(1, txBuffer, sizeof(txBuffer), 0);
    txBuffer[8] = hdopGps & 0xFF;
  
  Serial.println(gps.location.lat(), 5);
  Serial.println(gps.location.lat(), 5);
  Serial.print("Longitude : ");
  Serial.println(gps.location.lng(), 4);
  Serial.print("Satellites: ");
  Serial.println(gps.satellites.value());
  Serial.print("Altitude  : ");
  Serial.print(gps.altitude.feet() / 3.2808);
  Serial.println(F("Packet queued"));
  Serial.print(F("Sending packet on frequency: "));
  Serial.println(LMIC.freq);
    }
}
 int i = 3;
void setup() {
Serial.begin(115200);
  Wire.begin(21, 22); // configurado a comunicação com o axp
  if (!axp.begin(Wire, AXP192_SLAVE_ADDRESS)) {
    Serial.println("AXP192 Begin PASS");
  } else {
    Serial.println("AXP192 Begin FAIL");
  }
  axp.setPowerOutPut(AXP192_LDO2, AXP202_ON);
  axp.setPowerOutPut(AXP192_LDO3, AXP202_ON);
  axp.setPowerOutPut(AXP192_DCDC2, AXP202_ON);
  axp.setPowerOutPut(AXP192_EXTEN, AXP202_ON);
  axp.setPowerOutPut(AXP192_DCDC1, AXP202_ON);
  GPS.begin(9600, SERIAL_8N1, 34, 12);  // configurando comunicação com o NEO6.
  Serial.println(F("Starting"));

    #ifdef VCC_ENABLE
    // For Pinoccio Scout boards
    pinMode(VCC_ENABLE, OUTPUT);
    digitalWrite(VCC_ENABLE, HIGH);
    delay(1000);
    #endif

    #if defined(CFG_au921)
    Serial.println(F("Loading AU915/AU921 Configuration..."));
    #endif

    os_init();
    LMIC_reset(); // Redefine o estado do MAC
  
    LMIC_setDrTxpow(DR_SF7, 14); //Definir a potencia de envio e o SF inicial.
    LMIC_selectSubBand(1);
    LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100); // algoritimo de correção de clock.
    
     // Inicia o OTAA caso não tenha uma sessão ativa e caso tenha envia a msg.
    do_send(&sendjob);
}

void loop() {

     os_runloop_once();
     while (GPS.available())
      gps.encode(GPS.read());
}
