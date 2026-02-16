#include <SPI.h>
#include <RF24.h>

enum HubCmd : uint8_t {
    HC_NONE = 0,
    HC_SEQ_ALL,
    HC_SEQ_LASER,
    HC_SEQ_SERVOS,
    HC_SEQ_LINK,
    HC_SEQ_MCU,
    HC_SWEEP_H,
    HC_SWEEP_V,
    HC_SWEEP_D1,
    HC_SWEEP_D2,
    HC_HOME,
    HC_SWEEP_FULL
};

enum HubState : uint8_t {
    HS_IDLE = 0,
    HS_WAIT,
    HS_SWEEP_H,
    HS_SWEEP_V,
    HS_SWEEP_D1,
    HS_SWEEP_D2,
    HS_DIAG
};

enum HubCheck : uint8_t {
    HCHECK_NONE = 0,
    HCHECK_LASER,
    HCHECK_SERVOS,
    HCHECK_RADIO,
    HCHECK_MCU,
    HCHECK_ALL
};

struct HubCmdFrame {
    uint8_t nodeId;
    HubCmd  command;
};

struct HubTelemFrame {
    uint8_t nodeId;
    int8_t  tiltDeg;
    int8_t  panDeg;
    HubState state;
};

struct HubCheckFrame {
    uint8_t   nodeId;
    HubCheck  item;
    bool      phaseStart;
    bool      phaseEnd;
    bool      success;
};

const uint8_t HUB_RF_CE  = 7;
const uint8_t HUB_RF_CSN = 8;

RF24 hubRadio(HUB_RF_CE, HUB_RF_CSN);

const byte HUB_PIPE_HOST[6] = "HBASE";
const byte HUB_PIPE_NODE[6] = "HNODE";

const uint8_t HUB_TARGET_ID = 1;

void radioSendCmd(HubCmd c) {
    HubCmdFrame frame;
    frame.nodeId = HUB_TARGET_ID;
    frame.command = c;

    hubRadio.stopListening();
    hubRadio.openWritingPipe(HUB_PIPE_NODE);
    bool ok = hubRadio.write(&frame, sizeof(frame));
    hubRadio.startListening();

    Serial.print("send ");
    Serial.print((int)c);
    Serial.print(" status ");
    Serial.println(ok ? "OK" : "FAIL");
}

const char* stateToStr(HubState s) {
    switch (s) {
        case HS_IDLE:     return "IDLE";
        case HS_WAIT:     return "WAIT";
        case HS_SWEEP_H:  return "H";
        case HS_SWEEP_V:  return "V";
        case HS_SWEEP_D1: return "D1";
        case HS_SWEEP_D2: return "D2";
        case HS_DIAG:     return "DIAG";
        default:          return "?";
    }
}

const char* checkToStr(HubCheck c) {
    switch (c) {
        case HCHECK_LASER:  return "LASER";
        case HCHECK_SERVOS: return "SERVOS";
        case HCHECK_RADIO:  return "RADIO";
        case HCHECK_MCU:    return "MCU";
        case HCHECK_ALL:    return "ALL";
        default:            return "NONE";
    }
}

void showHelp() {
    Serial.println("TX console:");
    Serial.println(" 1 - run all diag");
    Serial.println(" 2 - test laser");
    Serial.println(" 3 - test servos");
    Serial.println(" 4 - test radio");
    Serial.println(" 5 - test mcu");
    Serial.println(" h - sweep H");
    Serial.println(" v - sweep V");
    Serial.println(" d - sweep diag1");
    Serial.println(" f - sweep diag2");
    Serial.println(" s - full sweep");
    Serial.println(" 0 - go home");
}

void handleConsoleKey(char c) {
    switch (c) {
        case '1': radioSendCmd(HC_SEQ_ALL);   break;
        case '2': radioSendCmd(HC_SEQ_LASER); break;
        case '3': radioSendCmd(HC_SEQ_SERVOS);break;
        case '4': radioSendCmd(HC_SEQ_LINK);  break;
        case '5': radioSendCmd(HC_SEQ_MCU);   break;
        case 'h': radioSendCmd(HC_SWEEP_H);   break;
        case 'v': radioSendCmd(HC_SWEEP_V);   break;
        case 'd': radioSendCmd(HC_SWEEP_D1);  break;
        case 'f': radioSendCmd(HC_SWEEP_D2);  break;
        case 's': radioSendCmd(HC_SWEEP_FULL);break;
        case '0': radioSendCmd(HC_HOME);      break;
        default: break;
    }
}

void printCheckFrame(const HubCheckFrame* f) {
    Serial.print("[CHK] id=");
    Serial.print(f->nodeId);
    Serial.print(" item=");
    Serial.print(checkToStr(f->item));
    Serial.print(" st=");
    Serial.print(f->phaseStart ? '1' : '0');
    Serial.print(" end=");
    Serial.print(f->phaseEnd ? '1' : '0');
    Serial.print(" ok=");
    Serial.println(f->success ? '1' : '0');
}

void printTelemFrame(const HubTelemFrame* f) {
    Serial.print("[TEL] id=");
    Serial.print(f->nodeId);
    Serial.print(" mode=");
    Serial.print(stateToStr(f->state));
    Serial.print(" tilt=");
    Serial.print(f->tiltDeg);
    Serial.print(" pan=");
    Serial.println(f->panDeg);
}

void rxProcess() {
    uint8_t raw[32];
    hubRadio.read(&raw, sizeof(raw));

    HubCheckFrame* cf = (HubCheckFrame*)raw;
    bool looksCheck =
        cf->nodeId == HUB_TARGET_ID &&
        cf->item >= HCHECK_LASER &&
        cf->item <= HCHECK_ALL;

    if (looksCheck) {
        printCheckFrame(cf);
    } else {
        HubTelemFrame* tf = (HubTelemFrame*)raw;
        printTelemFrame(tf);
    }
}

void setup() {
    Serial.begin(115200);
    Serial.println();
    showHelp();

    hubRadio.begin();
    hubRadio.setChannel(90);
    hubRadio.setDataRate(RF24_250KBPS);
    hubRadio.setPALevel(RF24_PA_LOW);
    hubRadio.openReadingPipe(1, HUB_PIPE_HOST);
    hubRadio.startListening();
}

void loop() {
    if (Serial.available()) {
        char c = Serial.read();
        handleConsoleKey(c);
    }

    if (hubRadio.available()) {
        rxProcess();
    }
}
