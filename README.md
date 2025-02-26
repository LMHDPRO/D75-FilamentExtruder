Control de Temperatura con PID y Motor Paso a Paso
Este proyecto implementa un sistema de control de temperatura utilizando un algoritmo PID (Proporcional-Integral-Derivativo) para mantener una temperatura deseada. Además, incluye el control de un motor paso a paso mediante un potenciómetro. El sistema está diseñado para funcionar en un ESP32 con un módulo CP2102.

Características principales
Control de temperatura: Utiliza un termistor para medir la temperatura y un algoritmo PID para ajustar la salida PWM y mantener la temperatura deseada.

Control de motor paso a paso: Permite controlar la velocidad de un motor paso a paso mediante un potenciómetro.

Interfaz LCD: Muestra la temperatura actual, la velocidad del motor y el valor del PID en una pantalla LCD I2C.

Botón de control: Un botón permite activar o desactivar el motor paso a paso.

Componentes utilizados
ESP32 (con módulo CP2102 para comunicación serial).

Termistor (sensor de temperatura).

Motor paso a paso con driver.

Pantalla LCD I2C (16x2 caracteres).

Potenciómetro (para controlar la velocidad del motor).

Botón (para activar/desactivar el motor).

MOSFET (para controlar la salida de potencia basada en el PID).

Librerías utilizadas
LiquidCrystal_I2C: Para controlar la pantalla LCD I2C.

AccelStepper: Para controlar el motor paso a paso.

Thermistor: Para leer la temperatura desde el termistor. Esta librería fue desarrollada por panStamp y se puede encontrar en su repositorio oficial: thermistor v1.0.3.

Instalación
Clona este repositorio en tu computadora: 
git clone https://github.com/LMHDPRO/D75-FilamentExtruder.git

Abre el proyecto en el IDE de Arduino.

Instala las librerías necesarias:

LiquidCrystal_I2C: Disponible en el Library Manager del IDE de Arduino.

AccelStepper: Disponible en el Library Manager del IDE de Arduino.

Thermistor: Descarga la versión 1.0.3 https://github.com/panStamp/thermistor

Conecta los componentes según el esquema proporcionado (ver sección Esquema de conexiones).

Sube el código a tu ESP32.

Esquema de conexiones
Componente	Pin ESP32
Termistor	GPIO 34 (A0)
Potenciómetro	GPIO 35 (A1)
Botón	GPIO 23
Enable Stepper	GPIO 22
Step Stepper	GPIO 18
Dir Stepper	GPIO 19
LED	GPIO 2
PWM (MOSFET)	GPIO 5
SDA LCD	GPIO 21
SCL LCD	GPIO 22

Uso
Enciende el sistema.

La pantalla LCD mostrará la temperatura actual, la velocidad del motor y el valor del PID.

Gira el potenciómetro para ajustar la velocidad del motor paso a paso.

Presiona el botón para activar o desactivar el motor.

El sistema ajustará automáticamente la salida PWM para mantener la temperatura deseada.

Créditos
Este proyecto utiliza la librería Thermistor desarrollada por panStamp. Agradecemos a los contribuidores de esta librería por su trabajo. Puedes encontrar más información y descargar la librería desde su repositorio oficial: thermistor v1.0.3.

Licencia
Este proyecto está bajo la licencia MIT. Siéntete libre de usarlo, modificarlo y distribuirlo según tus necesidades.
