#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <DHT.h>
#include <Adafruit_BMP085.h>
#include "BluetoothSerial.h"

BluetoothSerial SerialBT;
WebServer server(80);

// CONFIGURA TU RED
const char* ssid = "moto g84 5G";
const char* password = "12345678";

// LCD I2C
#define LCD_ADDR 0x27
#define LCD_BACKLIGHT 0x08
#define ENABLE 0x04

// Pines
#define DHT22_PIN 4
#define DHT11_PIN 5
#define LED_BLANCO 18
#define LED_ROJO   19
#define LED_VERDE  2
#define BUZZER_PIN 23

DHT dht22(DHT22_PIN, DHT22);
DHT dht11(DHT11_PIN, DHT11);
Adafruit_BMP085 bmp;

// Estado
bool pausado = false;
bool buzzerMute = false;
float temp, hum, pres;
float lastTemp = -999, lastHum = -999, lastPres = -999;

const char html[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head><meta charset='UTF-8'><title>ESP32</title><style>
body{text-align:center;font-family:sans-serif;background:#f0f0f0;}
.card{margin:auto;padding:10px;background:#fff;width:280px;border-radius:10px;box-shadow:0 0 5px #aaa;}
button{margin:5px;padding:8px;font-size:15px;border:none;border-radius:5px;background:#007BFF;color:#fff;}
button:hover{background:#0056b3}
</style></head><body>
<div class='card'><h3>Estación Meteorologica</h3>
<p id='temp'>Temp: -- °C</p>
<p id='hum'>Humedad: -- %</p>
<p id='pres'>Presión: ---- hPa</p>
<p id='estado'>Estado: --</p>
<p id='sonido'>Sonido: --</p>
<button onclick="cmd('p')">Pausar</button>
<button onclick="cmd('c')">Continuar</button>
<button onclick="cmd('m')">Mute</button>
<button onclick="cmd('u')">Unmute</button>
</div><script>
function cmd(c){fetch('/cmd?comando='+c).then(()=>upd())}
function upd(){fetch('/datos').then(r=>r.json()).then(d=>{
  document.getElementById('temp').innerText='Temp: '+d.temp+' °C';
  document.getElementById('hum').innerText='Humedad: '+d.hum+' %';
  document.getElementById('pres').innerText='Presión: '+d.pres+' hPa';
  document.getElementById('estado').innerText='Estado: '+d.estado;
  document.getElementById('sonido').innerText='Sonido: '+d.sonido;
})}
setInterval(upd, 3000);upd();
</script></body></html>
)rawliteral";
void lcd_send_nibble(uint8_t nibble, uint8_t mode);
void lcd_send_byte(uint8_t byte, uint8_t mode);
void lcd_command(uint8_t cmd);
void lcd_write_char(char chr);
void lcd_init();
void lcd_set_cursor(uint8_t col, uint8_t row);
void lcd_print(const char* str);
void lcd_clear();

// Web handlers
void handleRoot() { server.send_P(200, "text/html", html); }
void handleDatos() {
  String json = "{";
  json += "\"temp\":" + String(temp, 1) + ",";
  json += "\"hum\":" + String(hum, 1) + ",";
  json += "\"pres\":" + String(pres, 1) + ",";
  json += "\"estado\":\"" + String(pausado ? "pausado" : "midiendo") + "\",";
  json += "\"sonido\":\"" + String(buzzerMute ? "muteado" : "activo") + "\"}";
  server.send(200, "application/json", json);
}
void handleCmd() {
  if (server.hasArg("comando")) {
    char c = server.arg("comando")[0];
    if (c == 'p') {
      pausado = true;
      lcd_clear();
      lcd_set_cursor(0, 0); lcd_print("Mediciones");
      lcd_set_cursor(0, 1); lcd_print("en pausa");
      delay(500);
    }
    if (c == 'c') {
      pausado = false;
      lcd_clear();
      lcd_set_cursor(0, 0); lcd_print("Reanudando...");
      delay(500);
    }
    if (c == 'm') {
      buzzerMute = true;
      lcd_clear();
      lcd_set_cursor(0, 0); lcd_print("Sonido:");
      lcd_set_cursor(0, 1); lcd_print("muteado");
      delay(500);
    }
    if (c == 'u') {
      buzzerMute = false;
      lcd_clear();
      lcd_set_cursor(0, 0); lcd_print("Sonido:");
      lcd_set_cursor(0, 1); lcd_print("activo");
      delay(500);
    }
  }
  server.send(200, "text/plain", "ok");
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Iniciando ESP32...");

  // Inicializar sensores
  Wire.begin(21, 22);      // SDA, SCL
  dht22.begin();
  dht11.begin();
  bmp.begin();
  lcd_init();

  // Pines de salida
  pinMode(LED_BLANCO, OUTPUT);
  pinMode(LED_ROJO, OUTPUT);
  pinMode(LED_VERDE, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  // Iniciar Bluetooth
  SerialBT.begin("EstacionESP32");
  Serial.println("Bluetooth iniciado como 'EstacionESP32'");

  // Conexión WiFi
  Serial.print("Conectando a WiFi: ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nWiFi conectado");
  Serial.print("Dirección IP: ");
  Serial.println(WiFi.localIP()); 

  // Iniciar servidor web
  server.on("/", handleRoot);
  server.on("/datos", handleDatos);
  server.on("/cmd", handleCmd);
  server.begin();
  Serial.println("Servidor web iniciado");
}


void loop() {
  server.handleClient();

  // Bluetooth comandos (procesados en tiempo real)
  if (SerialBT.available()) {
    char c = SerialBT.read();
    if (c == 'p') {
      pausado = true;
      lcd_clear();
      lcd_set_cursor(0, 0); lcd_print("Mediciones");
      lcd_set_cursor(0, 1); lcd_print("en pausa");
    }
    if (c == 'c') {
      pausado = false;
      lcd_clear();
      lcd_set_cursor(0, 0); lcd_print("Reanudando...");
    }
    if (c == 'm') {
      buzzerMute = true;
      lcd_clear();
      lcd_set_cursor(0, 0); lcd_print("Sonido:");
      lcd_set_cursor(0, 1); lcd_print("muteado");
    }
    if (c == 'u') {
      buzzerMute = false;
      lcd_clear();
      lcd_set_cursor(0, 0); lcd_print("Sonido:");
      lcd_set_cursor(0, 1); lcd_print("activo");
    }
  }

  static unsigned long previousMillis = 0;
  static int pantalla = 0;
  const unsigned long intervalo = 3000;

  // Control del buzzer breve y LED blanco (cada ciclo de lectura)
  static bool medicionEnCurso = false;
  static unsigned long buzzerStart = 0;

  unsigned long currentMillis = millis();

  // Ejecutar cada 3 segundos solo si no está pausado
  if (!pausado && currentMillis - previousMillis >= intervalo) {
    previousMillis = currentMillis;
    medicionEnCurso = true;
    buzzerStart = currentMillis;

    // Activar LED blanco y buzzer (breve)
    digitalWrite(LED_BLANCO, HIGH);
    if (!buzzerMute) tone(BUZZER_PIN, 1000);

    // Leer sensores
    temp = dht22.readTemperature();
    hum = dht11.readHumidity();
    pres = bmp.readPressure() / 100.0;

    // Bluetooth si hay cambios
    if (temp != lastTemp || hum != lastHum || pres != lastPres) {
      SerialBT.print("Temp: "); SerialBT.print(temp, 1);
      SerialBT.print(" C | Hum: "); SerialBT.print(hum, 1);
      SerialBT.print(" % | Pres: "); SerialBT.print(pres, 1);
      SerialBT.println(" hPa");
      lastTemp = temp; lastHum = hum; lastPres = pres;
    }

    // LEDs según temperatura
    if (temp > 27.5) {
      digitalWrite(LED_ROJO, HIGH); digitalWrite(LED_VERDE, LOW);
    } else if (temp < 23.0) {
      digitalWrite(LED_VERDE, HIGH); digitalWrite(LED_ROJO, LOW);
    } else {
      digitalWrite(LED_ROJO, LOW); digitalWrite(LED_VERDE, LOW);
    }

    // Mostrar en LCD (pantalla alterna)
    lcd_clear();
    if (pantalla == 0) {
      lcd_set_cursor(0, 0); lcd_print("Temp:");
      char tStr[6]; dtostrf(temp, 4, 1, tStr);
      lcd_print(tStr); lcd_write_char(223); lcd_print("C");
      lcd_set_cursor(0, 1); lcd_print("Hum: ");
      char hStr[6]; dtostrf(hum, 4, 1, hStr);
      lcd_print(hStr); lcd_print("%");
    } else {
      lcd_set_cursor(0, 0); lcd_print("Presion:");
      char pStr[10]; dtostrf(pres, 7, 1, pStr);
      lcd_set_cursor(0, 1); lcd_print(pStr); lcd_print(" hPa");
    }
    pantalla = 1 - pantalla;
  }

  // Apagar buzzer y LED después de 500 ms
  if (medicionEnCurso && (millis() - buzzerStart >= 500)) {
    digitalWrite(LED_BLANCO, LOW);
    noTone(BUZZER_PIN);
    medicionEnCurso = false;
  }
}

// FUNCIONES LCD
void lcd_send_nibble(uint8_t nibble, uint8_t mode) {
  uint8_t data = (nibble & 0xF0) | LCD_BACKLIGHT | mode;
  Wire.beginTransmission(LCD_ADDR);
  Wire.write(data | ENABLE); Wire.write(data);
  Wire.endTransmission(); delayMicroseconds(50);
}
void lcd_send_byte(uint8_t byte, uint8_t mode) {
  lcd_send_nibble(byte & 0xF0, mode);
  lcd_send_nibble((byte << 4) & 0xF0, mode);
}
void lcd_command(uint8_t cmd) { lcd_send_byte(cmd, 0x00); }
void lcd_write_char(char chr) { lcd_send_byte(chr, 0x01); }
void lcd_set_cursor(uint8_t col, uint8_t row) {
  const uint8_t row_offsets[] = {0x00, 0x40, 0x14, 0x54};
  lcd_command(0x80 | (col + row_offsets[row]));
}
void lcd_print(const char* str) { while (*str) lcd_write_char(*str++); }
void lcd_clear() { lcd_command(0x01); delay(2); }
void lcd_init() {
  delay(50); lcd_send_nibble(0x30, 0x00); delay(5);
  lcd_send_nibble(0x30, 0x00); delay(5);
  lcd_send_nibble(0x30, 0x00); delay(5);
  lcd_send_nibble(0x20, 0x00);
  lcd_command(0x28); lcd_command(0x08);
  lcd_command(0x01); delay(2);
  lcd_command(0x06); lcd_command(0x0C);
}