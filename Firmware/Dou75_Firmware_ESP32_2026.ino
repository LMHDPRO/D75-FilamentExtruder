// ═══════════════════════════════════════════════════════════════════
//  EXTRUSORA PP — ESP32 DualCore  ★  ESP32_Experimental
//  Convierte filamento PP 2.4 mm → 1.75 mm
//
//  Termistor: NTC 100K genérico Ender3 V1 (β = 3950)
//  Pantalla:  LCD I2C 20×4 @ 0x27
//  Encoder:   KY-040  (OUTA=18, OUTB=19, SW=5)
//
//  MAPA DE PINES FINAL
//  ┌──────────┬───────┬─────────────────────────────────────────┐
//  │ Señal    │ GPIO  │ Nota                                    │
//  ├──────────┼───────┼─────────────────────────────────────────┤
//  │ ENC_CLK  │  18   │ OUTA — interrupt                       │
//  │ ENC_DT   │  19   │ OUTB — interrupt                       │
//  │ ENC_SW   │   5   │ Botón encoder — INPUT_PULLUP            │
//  │ NTC      │  34   │ ADC1_CH6 — input-only, máx 3.3 V       │
//  │ HEATER   │  25   │ MOSFET calefactor — LEDC ch0            │
//  │ FAN      │  23   │ MOSFET ventilador — LEDC ch1            │
//  │ STEP1    │  26   │ Motor reductor STEP                     │
//  │ DIR1     │  27   │ Motor reductor DIR                      │
//  │ STEP2    │   4   │ Motor arrastre STEP                     │
//  │ DIR2     │   2   │ Motor arrastre DIR (TB6600 sin pull-up) │
//  │ EN_DRV   │  13   │ Enable ambos TB6600 (LOW=activo)        │
//  │ SDA      │  21   │ I2C default                             │
//  │ SCL      │  22   │ I2C default                             │
//  └──────────┴───────┴─────────────────────────────────────────┘
//
//  PROTOCOLO SERIAL (115200 baud)
//  ESP32 → PC  (cada 100 ms):
//    "T:160.5,S:160,D1:100,D2:100,P:127,F:64,RUN:1\n"
//  PC → ESP32:
//    "ST:165\n"   set temperatura
//    "SD1:100\n"  set RPM driver 1
//    "SD2:120\n"  set RPM driver 2
//    "SF:1\n"     fan AUTO=1 / OFF=0
//    "RUN\n"      iniciar extrusión
//    "STOP\n"     detener
//
//  LIBRERÍAS (Library Manager):
//    · RotaryEncoder     by Matthias Hertel
//    · PID_v1            by Brett Beauregard
//    · AccelStepper      by Mike McCauley
//    · LiquidCrystal_I2C by Frank de Brabander
// ═══════════════════════════════════════════════════════════════════

#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <PID_v1.h>
#include <AccelStepper.h>
#include <RotaryEncoder.h>

// ───────────────────────────────────────────────────────────────────
//  PINES
// ───────────────────────────────────────────────────────────────────
#define PIN_ENC_CLK   18
#define PIN_ENC_DT    19
#define PIN_ENC_SW     5
#define PIN_NTC       34
#define PIN_HEATER    25
#define PIN_FAN       23
#define PIN_STEP1     26
#define PIN_DIR1      27
#define PIN_STEP2      4
#define PIN_DIR2       2
#define PIN_EN_DRV    13

// ───────────────────────────────────────────────────────────────────
//  LEDC
// ───────────────────────────────────────────────────────────────────
#define LEDC_HEAT_CH  0
#define LEDC_FAN_CH   1
#define LEDC_FREQ  5000
#define LEDC_RES      8

// ───────────────────────────────────────────────────────────────────
//  NTC 100K — Ender3 V1 genérico  β = 3950
//  Divisor: 3.3 V → 4.7 KΩ → GPIO34 → NTC100K → GND
// ───────────────────────────────────────────────────────────────────
#define NTC_SERIES_R  4700.0f
#define NTC_R0      100000.0f
#define NTC_T0          25.0f
#define NTC_BETA      3950.0f
#define ADC_VREF         3.3f
#define ADC_MAX       4095.0f

// ───────────────────────────────────────────────────────────────────
//  MOTORES — Full step, 200 pasos/rev
//  pasos/seg = RPM × 200 / 60  →  1 RPM ≈ 3.333 pasos/seg
// ───────────────────────────────────────────────────────────────────
#define STEPS_PER_REV  200
#define RPM_TO_SPS     (STEPS_PER_REV / 60.0f)
#define MOTOR_DIR      -1      // -1 = invertir sentido; cambiar a 1 si gira al revés
#define RPM_MAX_LIM    300
#define RPM_DEFAULT    100

// ───────────────────────────────────────────────────────────────────
//  TEMPERATURA
// ───────────────────────────────────────────────────────────────────
#define TEMP_MIN        20
#define TEMP_MAX       260
#define TEMP_DEFAULT   160
#define TEMP_PREHEAT   150     // temperatura de pre-calentamiento
#define TEMP_READY_TOL   3.0f  // ±°C para considerar "temperatura lista"

// ───────────────────────────────────────────────────────────────────
//  MENÚ — estados de la máquina de estados HMI
// ───────────────────────────────────────────────────────────────────
enum MenuState : uint8_t {
  ST_MAIN,        // menú principal: PRE-CALENTAR / CONFIGURAR
  ST_PREHEAT,     // calentando hasta 150°C, espera temperatura
  ST_CONFIG,      // submenú configuración
  ST_EDIT_D1,     // editando RPM Driver 1
  ST_EDIT_D2,     // editando RPM Driver 2
  ST_EDIT_TEMP,   // editando temperatura setpoint
  ST_EDIT_FAN,    // editando Fan AUTO/OFF
  ST_RUNNING      // extrusión activa — pantalla de operación
};

// ───────────────────────────────────────────────────────────────────
//  OBJETOS
// ───────────────────────────────────────────────────────────────────
RotaryEncoder    encoder(PIN_ENC_DT, PIN_ENC_CLK, RotaryEncoder::LatchMode::FOUR3);
LiquidCrystal_I2C lcd(0x27, 20, 4);
AccelStepper     motor1(AccelStepper::DRIVER, PIN_STEP1, PIN_DIR1);
AccelStepper     motor2(AccelStepper::DRIVER, PIN_STEP2, PIN_DIR2);

// ───────────────────────────────────────────────────────────────────
//  VARIABLES COMPARTIDAS — protegidas con portMUX
// ───────────────────────────────────────────────────────────────────
portMUX_TYPE gMux = portMUX_INITIALIZER_UNLOCKED;

volatile double g_temp      = 0.0;
volatile int    g_setpoint  = TEMP_DEFAULT;
volatile int    g_rpmD1     = RPM_DEFAULT;
volatile int    g_rpmD2     = RPM_DEFAULT;
volatile int    g_heatPwm   = 0;
volatile int    g_fanPwm    = 0;
volatile bool   g_fanAuto   = true;     // true=AUTO, false=OFF
volatile bool   g_running   = false;
volatile bool   g_sensorOK  = true;
volatile bool   g_tempReady = false;    // true cuando T dentro de tolerancia

// ───────────────────────────────────────────────────────────────────
//  PID — solo Core 0
// ───────────────────────────────────────────────────────────────────
double pidIn = 0, pidOut = 0, pidSP = TEMP_DEFAULT;
PID myPID(&pidIn, &pidOut, &pidSP, 35.0, 0.5, 0.0, DIRECT);

// ═══════════════════════════════════════════════════════════════════
//  LECTURA NTC — promedio 8 muestras
// ═══════════════════════════════════════════════════════════════════
float readNTC() {
  long sum = 0;
  for (int i = 0; i < 8; i++) {
    sum += analogRead(PIN_NTC);
    delayMicroseconds(150);
  }
  float raw = sum / 8.0f;
  if (raw < 20 || raw > 4075) return -999.0f;
  float v    = raw * (ADC_VREF / ADC_MAX);
  float rNTC = NTC_SERIES_R * v / (ADC_VREF - v);
  float lnR  = logf(rNTC / NTC_R0);
  float invT = (1.0f / (NTC_T0 + 273.15f)) + (lnR / NTC_BETA);
  return (1.0f / invT) - 273.15f;
}

// ═══════════════════════════════════════════════════════════════════
//  TAREA CORE 0 — NTC + PID + Calefactor + Fan + Serial
// ═══════════════════════════════════════════════════════════════════
void taskPID(void* pv) {
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, 255);
  TickType_t wake = xTaskGetTickCount();

  for (;;) {
    float t = readNTC();
    bool  ok = (t > -100.0f);

    portENTER_CRITICAL(&gMux);
    pidSP = (double)g_setpoint;
    portEXIT_CRITICAL(&gMux);

    // ── Calefactor ──
    if (ok) {
      pidIn = (double)t;
      myPID.Compute();
      ledcWrite(LEDC_HEAT_CH, (uint32_t)pidOut);
    } else {
      pidOut = 0;
      ledcWrite(LEDC_HEAT_CH, 0);  // fallo sensor → apagar
    }

    // ── Ventilador AUTO: proporcional sobre 80°C ──
    int fanVal = 0;
    bool fanAuto;
    portENTER_CRITICAL(&gMux); fanAuto = g_fanAuto; portEXIT_CRITICAL(&gMux);
    if (fanAuto && ok && pidIn > 80.0) {
      fanVal = (int)constrain((pidIn - 80.0) * 2.55f, 0, 255);
    }
    ledcWrite(LEDC_FAN_CH, fanVal);

    // ── Temperatura lista ──
    bool ready = ok && fabsf(t - (float)pidSP) <= TEMP_READY_TOL;

    portENTER_CRITICAL(&gMux);
    g_temp      = ok ? (double)t : g_temp;
    g_heatPwm   = (int)pidOut;
    g_fanPwm    = fanVal;
    g_sensorOK  = ok;
    g_tempReady = ready;
    portEXIT_CRITICAL(&gMux);

    // ── Telemetría Serial ──
    if (ok) {
      Serial.printf("T:%.1f,S:%d,D1:%d,D2:%d,P:%d,F:%d,RUN:%d\n",
                    (float)pidIn, g_setpoint, g_rpmD1, g_rpmD2,
                    (int)pidOut, fanVal, (int)g_running);
    } else {
      Serial.println("T:ERR,S:0,D1:0,D2:0,P:0,F:0,RUN:0");
    }

    vTaskDelayUntil(&wake, pdMS_TO_TICKS(100));
  }
}

// ═══════════════════════════════════════════════════════════════════
//  HELPERS LCD — 20 caracteres exactos con relleno de espacios
// ═══════════════════════════════════════════════════════════════════
void lcdRow(uint8_t row, const char* fmt, ...) {
  char buf[21];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  // Rellenar con espacios hasta 20 caracteres
  int len = strlen(buf);
  while (len < 20) buf[len++] = ' ';
  buf[20] = '\0';
  lcd.setCursor(0, row);
  lcd.print(buf);
}

void lcdClear4() {
  for (int i = 0; i < 4; i++) lcdRow(i, "");
}

// ═══════════════════════════════════════════════════════════════════
//  TAREA CORE 1 — HMI: encoder, menú, motores, comandos serial
// ═══════════════════════════════════════════════════════════════════
void taskHMI(void* pv) {

  // ── Estado HMI ──
  MenuState state     = ST_MAIN;
  int       menuSel   = 0;       // selección en menú actual
  int       cfgSel    = 0;       // selección en submenú config
  int       editVal   = 0;       // valor siendo editado
  int       lastEncPos= 0;
  bool      lastBtn   = HIGH;
  ulong     btnDown   = 0;
  bool      longFired = false;
  ulong     lastLcd   = 0;
  bool      dirty     = true;

  // ── Config items ──
  const char* cfgLabels[] = {
    "VELOCIDAD DRV 1",
    "VELOCIDAD DRV 2",
    "TEMPERATURA",
    "VENTILADOR"
  };
  const int CFG_N = 4;
  // cfgSel 0..3: primera visible en pantalla (scroll con 3 filas útiles)
  int cfgScroll = 0;

  // ── Motores ──
  motor1.setSpeed(0);
  motor2.setSpeed(0);
  digitalWrite(PIN_EN_DRV, HIGH);   // deshabilitado al inicio

  for (;;) {
    ulong now = millis();

    // ── Motores (siempre, no perder pasos) ──
    if (g_running) {
      motor1.runSpeed();
      motor2.runSpeed();
    }

    // ── Encoder tick ──
    encoder.tick();
    int encPos = (int)encoder.getPosition();
    int encDelta = encPos - lastEncPos;
    if (encDelta != 0) { lastEncPos = encPos; dirty = true; }

    // ── Botón: short press / long press ──
    bool btn = digitalRead(PIN_ENC_SW);
    bool shortPress = false, longPress = false;
    if (btn == LOW && lastBtn == HIGH) {          // flanco bajada
      btnDown = now; longFired = false;
    }
    if (btn == LOW && !longFired && (now - btnDown > 700)) {
      longPress = true; longFired = true; dirty = true;
    }
    if (btn == HIGH && lastBtn == LOW && !longFired) {
      shortPress = true; dirty = true;
    }
    lastBtn = btn;

    // ── Comandos desde Tkinter ──
    if (Serial.available()) {
      String cmd = Serial.readStringUntil('\n');
      cmd.trim();
      portENTER_CRITICAL(&gMux);
      if      (cmd.startsWith("ST:"))  g_setpoint = constrain(cmd.substring(3).toInt(), TEMP_MIN, TEMP_MAX);
      else if (cmd.startsWith("SD1:")) g_rpmD1 = constrain(cmd.substring(4).toInt(), 0, RPM_MAX_LIM);
      else if (cmd.startsWith("SD2:")) g_rpmD2 = constrain(cmd.substring(4).toInt(), 0, RPM_MAX_LIM);
      else if (cmd.startsWith("SF:"))  g_fanAuto = (cmd.substring(3).toInt() == 1);
      else if (cmd.startsWith("JOG1:")) {
        // Movimiento manual motor 1 (sin calentar)
        portEXIT_CRITICAL(&gMux);
        bool on = (cmd.substring(5).toInt() == 1);
        if (on) { motor1.setSpeed(MOTOR_DIR * g_rpmD1 * RPM_TO_SPS); digitalWrite(PIN_EN_DRV, LOW); }
        else    { motor1.setSpeed(0); if (!g_running && motor2.speed() == 0) digitalWrite(PIN_EN_DRV, HIGH); }
        goto skipMux;
      } else if (cmd.startsWith("JOG2:")) {
        portEXIT_CRITICAL(&gMux);
        bool on = (cmd.substring(5).toInt() == 1);
        if (on) { motor2.setSpeed(MOTOR_DIR * g_rpmD2 * RPM_TO_SPS); digitalWrite(PIN_EN_DRV, LOW); }
        else    { motor2.setSpeed(0); if (!g_running && motor1.speed() == 0) digitalWrite(PIN_EN_DRV, HIGH); }
        goto skipMux;
      } else if (cmd.startsWith("EN:")) {
        // Habilitar/deshabilitar AMBOS TB6600 (pin EN_DRV compartido)
        portEXIT_CRITICAL(&gMux);
        bool en = (cmd.substring(3).toInt() == 1);
        digitalWrite(PIN_EN_DRV, en ? LOW : HIGH);
        if (!en) { motor1.setSpeed(0); motor2.setSpeed(0); }
        else if (g_running) {
          motor1.setSpeed(MOTOR_DIR * g_rpmD1 * RPM_TO_SPS);
          motor2.setSpeed(MOTOR_DIR * g_rpmD2 * RPM_TO_SPS);
        }
        goto skipMux;
      }
      else if (cmd == "RUN") {
        g_running = true;
        portEXIT_CRITICAL(&gMux);
        float s1 = g_rpmD1 * RPM_TO_SPS, s2 = g_rpmD2 * RPM_TO_SPS;
        motor1.setSpeed(MOTOR_DIR * s1);
        motor2.setSpeed(MOTOR_DIR * s2);
        digitalWrite(PIN_EN_DRV, LOW);
        state = ST_RUNNING; dirty = true;
        goto skipMux;
      } else if (cmd == "STOP") {
        g_running = false;
        portEXIT_CRITICAL(&gMux);
        motor1.setSpeed(0); motor2.setSpeed(0);
        digitalWrite(PIN_EN_DRV, HIGH);
        state = ST_MAIN; menuSel = 0; dirty = true;
        goto skipMux;
      }
      portEXIT_CRITICAL(&gMux);
      dirty = true;
      skipMux:;
    }

    // ══════════════════════════════════════════════════════════
    //  MÁQUINA DE ESTADOS HMI
    // ══════════════════════════════════════════════════════════
    switch (state) {

      // ──────────────────────────────────────────────────────
      case ST_MAIN: {
        // Opciones: 0=PRE-CALENTAR, 1=CONFIGURAR
        if (encDelta) menuSel = constrain(menuSel + (encDelta > 0 ? 1 : -1), 0, 1);
        if (shortPress) {
          if (menuSel == 0) {
            // PRE-CALENTAR → 150°C
            portENTER_CRITICAL(&gMux); g_setpoint = TEMP_PREHEAT; portEXIT_CRITICAL(&gMux);
            state = ST_PREHEAT;
          } else {
            cfgSel = 0; cfgScroll = 0;
            state = ST_CONFIG;
          }
          dirty = true;
        }
        if (dirty && (now - lastLcd > 250)) {
          double t; bool sok;
          portENTER_CRITICAL(&gMux); t = g_temp; sok = g_sensorOK; portEXIT_CRITICAL(&gMux);
          lcdRow(0, "==== EXTRUSORA PP ===");
          lcdRow(1, "%s PRE-CALENTAR 150C", menuSel==0 ? ">" : " ");
          lcdRow(2, "%s CONFIGURAR",         menuSel==1 ? ">" : " ");
          if (sok) lcdRow(3, "T:%.1fC  SP:%dC", t, g_setpoint);
          else     lcdRow(3, "T: ERR  SP:%dC",  g_setpoint);
          lastLcd = now; dirty = false;
        }
        break;
      }

      // ──────────────────────────────────────────────────────
      case ST_PREHEAT: {
        // Esperar que llegue a TEMP_PREHEAT ± TOL
        bool ready;
        double t; bool sok;
        portENTER_CRITICAL(&gMux); t=g_temp; sok=g_sensorOK; ready=g_tempReady; portEXIT_CRITICAL(&gMux);

        if (!sok) {
          lcdRow(0, "!! ERROR SENSOR NTC !"); lcdRow(1, "Revisa conexion NTC ");
          lcdRow(2, "Calefactor apagado  "); lcdRow(3, "Long press = volver ");
        } else if (ready) {
          // Temperatura lista → auto-arrancar
          portENTER_CRITICAL(&gMux); g_running = true; portEXIT_CRITICAL(&gMux);
          float s1 = g_rpmD1*RPM_TO_SPS, s2 = g_rpmD2*RPM_TO_SPS;
          motor1.setSpeed(MOTOR_DIR*s1); motor2.setSpeed(MOTOR_DIR*s2);
          digitalWrite(PIN_EN_DRV, LOW);
          state = ST_RUNNING; dirty = true;
        } else {
          // Mostrando progreso de calentamiento
          if (dirty && (now - lastLcd > 500)) {
            int pct = (int)constrain((t / TEMP_PREHEAT) * 100.0, 0, 100);
            int bars = pct / 5;  // 0-20 barras en 20 chars
            char bar[21]; memset(bar, 0, 21);
            for (int i=0;i<20;i++) bar[i] = (i < bars) ? '#' : '-';
            lcdRow(0, "  PRE-CALENTANDO... ");
            lcdRow(1, "T:%.1fC  → %dC", t, TEMP_PREHEAT);
            lcdRow(2, "%s", bar);
            lcdRow(3, "%d%%  Long=Cancelar", pct);
            lastLcd = now; dirty = false;
          } else {
            dirty = true;  // forzar refresh periódico
          }
        }
        if (longPress) {
          // Cancelar
          portENTER_CRITICAL(&gMux); g_setpoint = TEMP_DEFAULT; portEXIT_CRITICAL(&gMux);
          state = ST_MAIN; menuSel = 0; lcdClear4(); dirty = true;
        }
        break;
      }

      // ──────────────────────────────────────────────────────
      case ST_CONFIG: {
        // Scroll: 3 filas útiles (fila 0 = header)
        if (encDelta) {
          cfgSel = constrain(cfgSel + (encDelta > 0 ? 1 : -1), 0, CFG_N-1);
          // Ajustar scroll para que cfgSel sea visible
          if (cfgSel < cfgScroll) cfgScroll = cfgSel;
          if (cfgSel >= cfgScroll + 3) cfgScroll = cfgSel - 2;
        }
        if (shortPress) {
          // Preparar valor de edición
          portENTER_CRITICAL(&gMux);
          if      (cfgSel==0) editVal = g_rpmD1;
          else if (cfgSel==1) editVal = g_rpmD2;
          else if (cfgSel==2) editVal = g_setpoint;
          else                editVal = g_fanAuto ? 1 : 0;
          portEXIT_CRITICAL(&gMux);
          state = (MenuState)(ST_EDIT_D1 + cfgSel);
          lcdClear4(); dirty = true;
        }
        if (longPress) { state = ST_MAIN; menuSel = 1; lcdClear4(); dirty = true; }

        if (dirty && (now - lastLcd > 200)) {
          lcdRow(0, "==== CONFIGURACION ==");
          for (int r=0; r<3; r++) {
            int idx = cfgScroll + r;
            if (idx < CFG_N) lcdRow(r+1, "%s%-16s", (idx==cfgSel ? ">" : " "), cfgLabels[idx]);
            else              lcdRow(r+1, "");
          }
          lastLcd = now; dirty = false;
        }
        break;
      }

      // ──────────────────────────────────────────────────────
      case ST_EDIT_D1:
      case ST_EDIT_D2: {
        bool isD1 = (state == ST_EDIT_D1);
        if (encDelta) editVal = constrain(editVal + encDelta*10, 0, RPM_MAX_LIM);
        if (shortPress) {
          portENTER_CRITICAL(&gMux);
          if (isD1) g_rpmD1 = editVal; else g_rpmD2 = editVal;
          portEXIT_CRITICAL(&gMux);
          if (g_running) {
            float sps = editVal * RPM_TO_SPS;
            if (isD1) motor1.setSpeed(MOTOR_DIR*sps);
            else      motor2.setSpeed(MOTOR_DIR*sps);
          }
          state = ST_CONFIG; lcdClear4(); dirty = true;
        }
        if (longPress) { state = ST_CONFIG; lcdClear4(); dirty = true; }
        if (dirty && (now - lastLcd > 100)) {
          lcdRow(0, isD1 ? "  VELOCIDAD DRV 1   " : "  VELOCIDAD DRV 2   ");
          lcdRow(1, "");
          lcdRow(2, "    < %d RPM >", editVal);
          lcdRow(3, " [OK]=Guardar Lng=X ");
          lastLcd = now; dirty = false;
        }
        break;
      }

      // ──────────────────────────────────────────────────────
      case ST_EDIT_TEMP: {
        if (encDelta) editVal = constrain(editVal + encDelta, TEMP_MIN, TEMP_MAX);
        if (shortPress) {
          portENTER_CRITICAL(&gMux); g_setpoint = editVal; portEXIT_CRITICAL(&gMux);
          state = ST_CONFIG; lcdClear4(); dirty = true;
        }
        if (longPress) { state = ST_CONFIG; lcdClear4(); dirty = true; }
        if (dirty && (now - lastLcd > 100)) {
          lcdRow(0, "    TEMPERATURA     ");
          lcdRow(1, "");
          lcdRow(2, "     < %d C >", editVal);
          lcdRow(3, " [OK]=Guardar Lng=X ");
          lastLcd = now; dirty = false;
        }
        break;
      }

      // ──────────────────────────────────────────────────────
      case ST_EDIT_FAN: {
        if (encDelta) editVal = (editVal == 0) ? 1 : 0;
        if (shortPress) {
          portENTER_CRITICAL(&gMux); g_fanAuto = (editVal == 1); portEXIT_CRITICAL(&gMux);
          state = ST_CONFIG; lcdClear4(); dirty = true;
        }
        if (longPress) { state = ST_CONFIG; lcdClear4(); dirty = true; }
        if (dirty && (now - lastLcd > 100)) {
          lcdRow(0, "    VENTILADOR      ");
          lcdRow(1, "");
          lcdRow(2, "   < %s >", editVal ? " AUTO " : " OFF  ");
          lcdRow(3, " [OK]=Guardar Lng=X ");
          lastLcd = now; dirty = false;
        }
        break;
      }

      // ──────────────────────────────────────────────────────
      case ST_RUNNING: {
        // Encoder ajusta setpoint de temperatura
        if (encDelta) {
          portENTER_CRITICAL(&gMux);
          g_setpoint = constrain(g_setpoint + encDelta, TEMP_MIN, TEMP_MAX);
          portEXIT_CRITICAL(&gMux);
        }
        // Long press = STOP
        if (longPress) {
          portENTER_CRITICAL(&gMux); g_running = false; portEXIT_CRITICAL(&gMux);
          motor1.setSpeed(0); motor2.setSpeed(0);
          digitalWrite(PIN_EN_DRV, HIGH);
          state = ST_MAIN; menuSel = 0; lcdClear4(); dirty = true;
        }
        if (dirty && (now - lastLcd > 500)) {
          double t; int sp, d1, d2, pwm, fpwm; bool sok, fan;
          portENTER_CRITICAL(&gMux);
          t=g_temp; sp=g_setpoint; d1=g_rpmD1; d2=g_rpmD2;
          pwm=g_heatPwm; fpwm=g_fanPwm; sok=g_sensorOK; fan=g_fanAuto;
          portEXIT_CRITICAL(&gMux);

          if (sok) lcdRow(0, "T:%.1fC   SP:%dC", t, sp);
          else     lcdRow(0, "T: ERR   SP:%dC",  sp);
          lcdRow(1, "D1:%dRPM  D2:%dRPM", d1, d2);
          lcdRow(2, "PWM:%-3d  FAN:%s", pwm, fan ? "AUTO" : "OFF ");
          lcdRow(3, "CORRIENDO Lng=STOP  ");
          lastLcd = now; dirty = false;
        } else {
          dirty = true;  // refrescar periódicamente
        }
        break;
      }
    } // switch

    taskYIELD();
  }
}

// ═══════════════════════════════════════════════════════════════════
//  SETUP
// ═══════════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);

  Wire.begin(21, 22);
  Wire.setClock(400000);

  // LEDC calefactor
  ledcSetup(LEDC_HEAT_CH, LEDC_FREQ, LEDC_RES);
  ledcAttachPin(PIN_HEATER, LEDC_HEAT_CH);
  ledcWrite(LEDC_HEAT_CH, 0);

  // LEDC ventilador
  ledcSetup(LEDC_FAN_CH, LEDC_FREQ, LEDC_RES);
  ledcAttachPin(PIN_FAN, LEDC_FAN_CH);
  ledcWrite(LEDC_FAN_CH, 0);

  // Pines digitales
  pinMode(PIN_ENC_SW, INPUT_PULLUP);
  pinMode(PIN_EN_DRV, OUTPUT);
  digitalWrite(PIN_EN_DRV, HIGH);   // motores deshabilitados al inicio

  // Interrupciones encoder
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_CLK), []{ encoder.tick(); }, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_DT),  []{ encoder.tick(); }, CHANGE);

  // Motores
  motor1.setMaxSpeed(RPM_MAX_LIM * RPM_TO_SPS + 100);
  motor2.setMaxSpeed(RPM_MAX_LIM * RPM_TO_SPS + 100);

  // LCD splash
  lcd.init();
  lcd.backlight();
  lcdRow(0, "   EXTRUSORA PP v2  ");
  lcdRow(1, "   ESP32 DualCore   ");
  lcdRow(2, " Filamento PP 1.75  ");
  lcdRow(3, "    Iniciando...    ");
  delay(2000);
  lcdRow(3, "                    ");

  // Lanzar tareas
  xTaskCreatePinnedToCore(taskPID, "PID", 4096, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(taskHMI, "HMI", 8192, NULL, 1, NULL, 1);
}

void loop() { vTaskDelete(NULL); }
