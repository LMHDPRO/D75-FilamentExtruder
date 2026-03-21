<div align="center">

```
██████╗  ██████╗ ██╗   ██╗ ██████╗ ██╗      █████╗ ███████╗███╗███████╗
██╔══██╗██╔═══██╗██║   ██║██╔════╝ ██║     ██╔══██╗██╔════╝ ██║██╔════╝
██║  ██║██║   ██║██║   ██║██║  ███╗██║     ███████║███████╗╚██║███████╗
██║  ██║██║   ██║██║   ██║██║   ██║██║     ██╔══██║╚════██║ ██║╚════██║
██████╔╝╚██████╔╝╚██████╔╝╚██████╔╝███████╗██║  ██║███████║ ██║███████║
╚═════╝  ╚═════╝  ╚═════╝  ╚═════╝ ╚══════╝╚═╝  ╚═╝╚══════╝ ╚═╝╚══════╝
```

# Filament Extruder Controller

**ESP32 DualCore · FreeRTOS · PID Temperature · Dual TB6600**

*Reduce PP filament — 2.4 mm → 1.75 mm for 3D printing*

---

[![ESP32](https://img.shields.io/badge/ESP32-DevKitC-blue?style=flat-square&logo=espressif)](https://www.espressif.com/)
[![Arduino](https://img.shields.io/badge/Arduino-Framework-00979D?style=flat-square&logo=arduino&logoColor=white)](https://www.arduino.cc/)
[![Python](https://img.shields.io/badge/Python-3.10+-3776AB?style=flat-square&logo=python&logoColor=white)](https://www.python.org/)
[![FreeRTOS](https://img.shields.io/badge/FreeRTOS-DualCore-brightgreen?style=flat-square)](https://www.freertos.org/)
[![License](https://img.shields.io/badge/License-MIT-yellow?style=flat-square)](LICENSE)

</div>

---

## ¿Qué es esto?

**Douglas75** es un controlador open-source para una extrusora de filamento, desarrollado originalmente en el marco de un **Practicum** universitario para reducir el diámetro de filamento fabricado artesanalmente. El diseño nació con polipropileno (PP) de **2.4 mm → 1.75 mm**, pero el sistema es compatible con cualquier tipo de filamento termoplástico que requiera un proceso de reducción controlada de diámetro.

El sistema corre en un **ESP32 con FreeRTOS dual-core** — un núcleo dedicado al control PID de temperatura y otro a la interfaz de usuario. Incluye un **panel de control Tkinter** para operación desde PC vía USB serial.

---

## Características

| | Función | Detalle |
|---|---|---|
| 🌡️ | Control temperatura | PID con NTC 100K genérico Ender3 (β = 3950) |
| 🔥 | Calefactor | MOSFET IRLZ44N, PWM 5 kHz vía LEDC |
| 💨 | Ventilador | MOSFET IRLZ44N, PWM proporcional AUTO o manual OFF |
| ⚙️ | Motores | 2× NEMA17 full-step, drivers TB6600, enable compartido |
| 🖥️ | LCD | I2C 20×4 @ 0x27, menú completo con encoder KY-040 |
| 💻 | GUI PC | Tkinter dark mode + gráfica temperatura en tiempo real |
| 🧵 | Estimación | Contador de filamento procesado (mm y gramos) |
| ⚡ | Arquitectura | FreeRTOS — Core 0: PID · Core 1: HMI |

---

## Hardware

### Bill of Materials (BOM)

| # | Categoría | Componente | Modelo / Especificación | Qty |
|:---:|---|---|---|:---:|
| 1 | 🧠 MCU | Microcontrolador | ESP32 Development Board | 1 |
| 2 | 🔥 Calefactor | Hotend completo | 40W 12V Ceramic Cartridge Heater + NTC 100K Thermistor | 1 |
| 3 | 🔩 Boquilla | Nozzle | MK8 0.4 mm Nozzle | 1 |
| 4 | ⚙️ Extrusión | Extrusor | Creality MK8 Extruder | 1 |
| 5 | ⚙️ Motor | Motor paso a paso | NEMA17 Stepper Motor | 2 |
| 6 | ⚙️ Driver | Driver motor | TB6600 Stepper Motor Driver | 2 |
| 7 | 🔧 Soporte | Montura motor | NEMA Motor Mount | 2 |
| 8 | 🖥️ Pantalla | Pantalla LCD | 20×4 LCD Screen I2C | 1 |
| 9 | 🎛️ Entrada | Encoder rotatorio | KY-040 Rotary Encoder | 1 |
| 10 | 🧵 Sensor | Sensor de filamento | Filament Runout Sensor | 1 |
| 11 | ⚡ Regulador | Regulador voltaje | L7805CV 5V Voltage Regulator (TO-220) | 1 |
| 12 | ⚡ MOSFET | MOSFET N-Channel | IRFL44N N-Channel MOSFET | 2 |
| 13 | 🔩 Rodamientos | Rodamiento | 608ZZ Bearing | 4 |
| 14 | 💨 Ventilador | Fan | 4010 Fan 12V | 1 |
| 15 | 🟫 PCB | Placa base | 15×15 cm Single-Sided Phenolic Board | 1 |
| 16 | 🔌 Conectores | Pin header | 36-pin Single Row Pin Header (2.54 mm) | 1 |
| 17 | 🔌 Conectores | Female header | 10-pin Female Header (2.54 mm) | 2 |
| 18 | 🔌 Conectores | Terminales | 2-Pin Green Screw Terminals | 6 |
| 19 | 🔋 Capacitores | Condensador electrolítico | 100 µF | 2 |
| 20 | 🔋 Capacitores | Condensador cerámico | 100 nF | 2 |
| 21 | 🔩 Tornillería | Tornillos Allen | M2, M3 & M4 Socket Head Screws (surtido) | — |

### Mapa de pines ESP32

| GPIO | Señal | Tipo |
|---|---|---|
| `18` | OUTA encoder (CLK) | Input interrupt |
| `19` | OUTB encoder (DT) | Input interrupt |
| `5` | SW encoder | Input PULLUP |
| `34` | Termistor NTC | ADC1_CH6 |
| `25` | MOSFET calefactor | PWM LEDC ch0 |
| `23` | MOSFET ventilador | PWM LEDC ch1 |
| `26` | STEP Motor 1 | Output |
| `27` | DIR Motor 1 | Output |
| `4` | STEP Motor 2 | Output |
| `2` | DIR Motor 2 | Output |
| `13` | EN_Driver | Output |
| `21` | SDA LCD | I2C |
| `22` | SCL LCD | I2C |

> ⚠️ **Pines prohibidos:** GPIO 0, 6–11 (flash interno), 12, 15

### Circuito divisor NTC

```
3.3V
 │
[4.7 KΩ]  ← R7 pull-up
 │
 ├──── GPIO34 ──────────────── net: Thermistor_GPIO34
 │
 ├──── 100 nF cerámico  ┐ en paralelo
 ├──── 10 µF electrolítico ┘
 │
[NTC 100K]  ← cable vía J3 terminal block
 │
GND
```

### Circuito calefactor

```
GPIO25 ──[10 Ω]──→ Gate IRLZ44N
                    Drain  → Calefactor DC 12V (−)
                    Source → GND

[10 KΩ] entre Gate y GND   ← pull-down, seguro durante boot
LED + [1.8 KΩ] en paralelo ← indicador visual de activación
```

### Alimentación dual USB + 12V

```
12V (fuente principal)
 ├── Buck LM2596 → 5V → VIN ESP32
 ├── TB6600 VMOT  (+ condensador 100 µF entre VMOT y GND)
 └── Calefactor DC (vía MOSFET)

USB-C ESP32 → PC  (serial Tkinter GUI)
✓ Coexistencia segura — diodo Schottky en DevKit aísla USB de VIN
```

---

## Menú LCD

```
ARRANQUE
  └── Splash 2s
        └── MENÚ PRINCIPAL
              ├── ► PRE-CALENTAR ──→ Calienta a 150 °C
              │                       Barra de progreso
              │                       Auto-arranca al llegar
              └── ► CONFIGURAR
                    ├── VELOCIDAD DRV 1   (0–300 RPM, paso 10)
                    ├── VELOCIDAD DRV 2   (0–300 RPM, paso 10)
                    ├── TEMPERATURA       (20–260 °C, paso 1)
                    └── VENTILADOR        (AUTO / OFF)

PANTALLA CORRIENDO:
  ┌────────────────────┐
  │ T:160.5C  SP:160C  │
  │ D1:100RPM D2:100RP │
  │ PWM:127   FAN:AUTO │
  │ CORRIENDO Lng=STOP │
  └────────────────────┘
```

**Controles encoder KY-040:**

| Acción | Función |
|---|---|
| Girar | Navegar menú / cambiar valor |
| Presión corta | Seleccionar / confirmar |
| Presión larga >700 ms | Volver / STOP de emergencia |

---

## Protocolo Serial

**115200 baud, 8N1**

### ESP32 → PC (cada 100 ms)

```
T:160.5,S:160,D1:100,D2:100,P:127,F:64,RUN:1
```

| Campo | Descripción |
|---|---|
| `T` | Temperatura real (°C) — o `ERR` si falla el sensor |
| `S` | Setpoint temperatura |
| `D1` / `D2` | RPM motores 1 y 2 |
| `P` | PWM calefactor (0–255) |
| `F` | PWM ventilador (0–255) |
| `RUN` | Estado extrusión `0`/`1` |

### PC → ESP32

| Comando | Descripción |
|---|---|
| `ST:165` | Cambiar setpoint temperatura |
| `SD1:100` | Cambiar RPM Motor 1 |
| `SD2:120` | Cambiar RPM Motor 2 |
| `SF:1` / `SF:0` | Ventilador AUTO / OFF |
| `RUN` | Iniciar extrusión |
| `STOP` | Detener extrusión |
| `JOG1:1` / `JOG1:0` | Motor 1 manual ON/OFF |
| `JOG2:1` / `JOG2:0` | Motor 2 manual ON/OFF |
| `EN:1` / `EN:0` | Habilitar / deshabilitar ambos TB6600 |

---

## Instalación

### Firmware ESP32

**1. Instalar PlatformIO:**

Instala la extensión [PlatformIO IDE](https://platformio.org/install/ide?install=vscode) para VS Code.

**2. Clonar y abrir el proyecto:**

```bash
git clone https://github.com/LMHDPRO/D75-FilamentExtruder.git
cd D75-FilamentExtruder
code .   # abrir en VS Code con PlatformIO
```

**3. Las dependencias se instalan automáticamente** desde `platformio.ini`:

```ini
[env:esp32dev]
platform  = espressif32
board     = esp32dev
framework = arduino
lib_deps  =
    mathertel/RotaryEncoder
    br3ttb/PID
    waspinator/AccelStepper
    marcoschwartz/LiquidCrystal_I2C
```

**4. Subir el firmware:**

Botón **Upload** (→) en la barra de PlatformIO, o:

```bash
pio run --target upload
```

> Si no entra en modo flash automáticamente, mantén presionado el botón **BOOT** del DevKit mientras sube.

**5. Verificar en Serial Monitor (115200 baud):**

```
T:25.3,S:160,D1:100,D2:100,P:0,F:0,RUN:0   ✓ Sensor OK
T:ERR,S:0,D1:0,D2:0,P:0,F:0,RUN:0          ✗ Revisar NTC o divisor
```

### GUI Python

```bash
pip install pyserial matplotlib
python extrusora_gui.py
```

---

## Arquitectura de software

```
ESP32 — FreeRTOS DualCore
│
├── Core 0  [prioridad 2]  taskPID
│    ├── Leer NTC (promedio 8 muestras)
│    ├── Calcular PID
│    ├── Escribir LEDC calefactor (0–255)
│    ├── Calcular PWM ventilador proporcional
│    └── Enviar telemetría serial cada 100 ms
│
└── Core 1  [prioridad 1]  taskHMI
     ├── Encoder KY-040 via interrupciones
     ├── Máquina de estados LCD (6 estados)
     ├── AccelStepper Motor 1 + Motor 2
     └── Parser comandos serial desde PC

Variables compartidas → protegidas con portMUX_TYPE
```

---

## Calibración

### Temperatura no coincide con la real

Cambiar `NTC_BETA` en `extrusora_esp32.ino`:

| Termistor | β |
|---|---|
| Genérico impresoras 3D (este proyecto) | **3950** |
| Semitec 104GT-2 | 4267 |
| EPCOS B57560G | 3988 |
| Marlin tabla 1 | 4388 |

### Motor gira al revés

```cpp
#define MOTOR_DIR  -1   →   #define MOTOR_DIR  1
```

### Estimación de filamento

Medir el diámetro del rodillo de arrastre con un vernier y actualizar en `extrusora_gui.py`:

```python
DRIVE_DIAM_MM = 11.0   # ← reemplazar con medición real
```

---

## Estructura del repositorio

```
D75-FilamentExtruder/
 ├── firmware/
 │    └── extrusora_esp32.ino      ← código ESP32
 ├── software/
 │    └── extrusora_gui.py         ← panel de control PC
 ├── hardware/
 │    ├── CircuitoESP32.kicad_sch  ← esquemático KiCad
 │    └── CircuitoESP32.kicad_pcb  ← PCB KiCad
 └── README.md
```

---

## Créditos

- [RotaryEncoder](https://github.com/mathertel/RotaryEncoder) — Matthias Hertel
- [Arduino PID Library](https://github.com/br3ttb/Arduino-PID-Library) — Brett Beauregard
- [AccelStepper](https://www.airspayce.com/mikem/arduino/AccelStepper/) — Mike McCauley
- [LiquidCrystal_I2C](https://github.com/marcoschwartz/LiquidCrystal_I2C) — Frank de Brabander

---

## Equipo

<div align="center">

*Proyecto desarrollado en la* **Universidad Anáhuac Mayab** *— Mérida, Yucatán*

<br>

| | Nombre | |
|:---:|:---|:---:|
| 🧑‍💻 | **Skyler Schenck** | [![LinkedIn](https://img.shields.io/badge/LinkedIn-0A66C2?style=flat-square&logo=linkedin&logoColor=white)](https://www.linkedin.com/in/skyler-schenck/) |
| 🧑‍💻 | **Darian Cuellar** | [![LinkedIn](https://img.shields.io/badge/LinkedIn-0A66C2?style=flat-square&logo=linkedin&logoColor=white)](https://www.linkedin.com/in/darian-cuellar-jimenez-6a144630b/) |
| 🧑‍💻 | **José Pardiñaz** | [![LinkedIn](https://img.shields.io/badge/LinkedIn-0A66C2?style=flat-square&logo=linkedin&logoColor=white)](https://www.linkedin.com/in/josepardinaz/) |
| 🧑‍💻 | **Douglas Fuentes** | [![LinkedIn](https://img.shields.io/badge/LinkedIn-0A66C2?style=flat-square&logo=linkedin&logoColor=white)](https://www.linkedin.com/in/douglas-fuentes-de-la-o/) |
| 🧑‍🔬 | **Gerardo Alonzo** | [![Research](https://img.shields.io/badge/Research-Profile-6B46C1?style=flat-square)](https://research.rciueducation.org/es/persons/gerardo-manuel-alonzo-medina/) |
| 🧑‍🔬 | **Andrés Oliva** | [![Research](https://img.shields.io/badge/Research-Profile-6B46C1?style=flat-square)](https://research.rciueducation.org/es/persons/andr%C3%A9s-iv%C3%A1n-oliva-avil%C3%A9s/) |

<br>

[![Anáhuac Mayab](https://img.shields.io/badge/Universidad_Anáhuac_Mayab-Mérida%2C_MX-D4111A?style=for-the-badge)](https://merida.anahuac.mx/)

</div>

---

## Apoya el proyecto

Este proyecto es **open-source bajo licencia MIT** — puedes usarlo, modificarlo y distribuirlo libremente. Si te fue útil y quieres apoyar al creador, puedes conseguir los archivos 3D del modelo físico de la extrusora en Cults3D a precio libre:

<div align="center">

[![Cults3D](https://img.shields.io/badge/Cults3D-Extrusora_DOU175-FF6B35?style=for-the-badge&logo=data:image/svg+xml;base64,PHN2ZyB4bWxucz0iaHR0cDovL3d3dy53My5vcmcvMjAwMC9zdmciIHZpZXdCb3g9IjAgMCAyNCAyNCI+PHBhdGggZmlsbD0id2hpdGUiIGQ9Ik0xMiAyQzYuNDggMiAyIDYuNDggMiAxMnM0LjQ4IDEwIDEwIDEwIDEwLTQuNDggMTAtMTBTMTcuNTIgMiAxMiAyem0tMiAxNHYtNGgtNGw2LTZjMCAyLjIxIDEuNzkgNCA0IDRsLTYgNnoiLz48L3N2Zz4=)](https://cults3d.com/es/modelo-3d/herramientas/extrusora-de-filamento-dou175)

*Los archivos de código son y serán siempre gratuitos. El modelo 3D está disponible en Cults a precio libre — cualquier contribución es bienvenida y apoya el desarrollo continuo.*

</div>

---

## Licencia

MIT License — úsalo, modifícalo y distribúyelo libremente.

---

<div align="center">
<sub>Douglas75 Project · ESP32_Experimental · Made with ☕ in Mérida, MX</sub>
</div>
