/*
 * =============================================================
 * PROJETO: AquaOS v14.0 (Open Source Edition)
 * AUTOR: [gaewnt/GitHub]
 * =============================================================
 * DESCRIÇÃO:
 * Sistema de automação para aquários utilizando ESP32 e plataforma
 * ESP RainMaker. Controla luzes, temperatura, filtragem e alimentação.
 * * HARDWARE NECESSÁRIO:
 * - ESP32 DevKit v1
 * - Módulo Relé 4 canais
 * - Drivers de Motor de Passo (ULN2003)
 * - Sensor de Temperatura DS18B20
 * * NOTAS DE VERSÃO:
 * - Otimizado para fuso horário UTC-4 (Configurável).
 * - Calibração de pH definida como 7.0 (Placeholder).
*/

#include "RMaker.h"
#include "WiFi.h"
#include "WiFiProv.h"
#include <nvs_flash.h>
#include <AccelStepper.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "time.h"
#include <Wire.h>

// Importa credenciais (Crie um arquivo secrets.h se não existir)
#if __has_include("secrets.h")
  #include "secrets.h"
#else
  const char *PROV_POP = "12345678"; // Senha padrão caso não haja arquivo secrets
  const char *PROV_NAME = "PROV_NOMEDOPROJETO"; //Substituia pelo que você configurar no RainMaker
#endif

// =============================================================
//               CONFIGURAÇÕES DO USUÁRIO (EDITÁVEIS)
// =============================================================

// --- Fuso Horário (Padrão: Cuiabá/AMT -4h) ---
// Para Brasília (UTC-3), mude gmtOffset_sec para -10800
const long  gmtOffset_sec = -14400; 
const int   daylightOffset_sec = 0;
const char* ntpServer = "pool.ntp.org";

// --- Definição de Pinos (GPIOs) ---
// Relés
#define PIN_RELAY_FILTER   5   // Filtro
#define PIN_RELAY_HEATER   17  // Aquecedor
#define PIN_RELAY_LIGHT    23  // Luzes LED

// Sensores e Atuadores Extras
#define PIN_COOLER_PWM     19  // Ventoinhas (Coolers)
#define PIN_SENSOR_PH      34  // Entrada Analógica Sensor pH
#define PIN_SENSOR_TEMP    4   // Barramento OneWire (DS18B20)

// Motores de Passo (Alimentador)
// Pinos: IN1, IN3, IN2, IN4 (Sequência para AccelStepper)
#define STEPPER_HALFSTEP 8
// Motor 1
#define M1_PIN1 14
#define M1_PIN2 26
#define M1_PIN3 27
#define M1_PIN4 13
// Motor 2
#define M2_PIN1 32
#define M2_PIN2 25
#define M2_PIN3 33
#define M2_PIN4 18

// --- Configurações de Lógica ---
#define LIGADO  LOW   // Mude para HIGH se seu relé for acionado em nível alto
#define DESLIGADO HIGH

// Temperaturas Padrão
float target_temp = 26.5; 
float temp_hysteresis = 0.5;

// =============================================================
//               FIM DAS CONFIGURAÇÕES
// =============================================================

// Objetos Globais
AccelStepper stepper1(STEPPER_HALFSTEP, M1_PIN1, M1_PIN3, M1_PIN2, M1_PIN4);
AccelStepper stepper2(STEPPER_HALFSTEP, M2_PIN1, M2_PIN3, M2_PIN2, M2_PIN4);
OneWire oneWire(PIN_SENSOR_TEMP);
DallasTemperature sensors(&oneWire);

// Variáveis de Estado
bool filter_state = true;
bool light_state = false;
bool maintenance_mode = false;
bool thermostat_auto = true; 
int  food_counter = 100;
float current_temp = 0.0;
float current_ph = 7.0;

// Variáveis de Controle de Tempo
bool last_heater_state = false;
bool last_cooler_state = false;
int light_on_hour = 8;
int light_off_hour = 18;
unsigned long last_temp_check = 0;
unsigned long last_schedule_check = 0;
const long CHECK_INTERVAL = 5000; 

String history_log = "Sistema Iniciado...\n";

// Declaração dos Dispositivos RainMaker
static Device my_filter("Filtro", "esp.device.outlet", NULL);
static Device my_cooler("Coolers", "esp.device.fan", NULL);
static Device my_heater("Aquecedor", "esp.device.switch", NULL);
static Device my_maintenance("Modo TPA", "esp.device.lock", NULL);
static Device my_auto_mode("Termostato Auto", "esp.device.switch", NULL);
static Device my_light("Luzes", "esp.device.lightbulb", NULL);
static Device my_feeder("Alimentar", "esp.device.switch", NULL);
static Device my_reset_food("Reset Racao", "esp.device.switch", NULL);
static Device my_logger("Historico", "esp.device.param", NULL);
static TemperatureSensor my_temp("Temperatura", NULL);
static TemperatureSensor my_ph("Sensor pH", NULL);

// Ponteiros de Parâmetros
static Param *filter_p, *cooler_p, *heater_p, *light_p, *feeder_p, *maint_p, *auto_p, *reset_p, *log_p;
static Param *status_msg_param, *light_on_param, *light_off_param;

// Protótipo do Callback
void write_callback(Device *device, Param *param, const param_val_t val, void *priv_data, write_ctx_t *ctx);

// --- SISTEMA DE LOGS ---
void addLog(String msg) {
  struct tm timeinfo;
  char timeStr[10];
  
  if(getLocalTime(&timeinfo)){
    sprintf(timeStr, "[%02d:%02d] ", timeinfo.tm_hour, timeinfo.tm_min);
  } else {
    sprintf(timeStr, "[--:--] ");
  }
  
  String newEntry = String(timeStr) + msg;
  history_log = newEntry + "\n" + history_log;
  
  // Limita o log para economizar memória (max 10 linhas)
  int lineCount = 0;
  int index = 0;
  while (index < history_log.length()) {
    if (history_log.charAt(index) == '\n') lineCount++;
    if (lineCount >= 10) {
      history_log = history_log.substring(0, index);
      break;
    }
    index++;
  }
  if(log_p) log_p->updateAndReport(esp_rmaker_str(history_log.c_str()));
}

// --- MENSAGENS DE STATUS (UI) ---
void updateStatusMsg() {
  char buffer[60];
  if (maintenance_mode) {
    sprintf(buffer, "⚠️ TPA: MANUTENÇÃO ATIVA");
  } else if (food_counter <= 0) {
    sprintf(buffer, "⚠️ ALERTA: RAÇÃO VAZIA!");
  } else {
    char mode[5];
    strcpy(mode, thermostat_auto ? "Aut" : "Man");
    sprintf(buffer, "Ração:%d%% | T:%.1f (%s) | pH:%.1f", food_counter, current_temp, mode, current_ph);
  }
  if(status_msg_param) status_msg_param->updateAndReport(esp_rmaker_str(buffer));
}

// --- LEITURA DE SENSORES ---
float readPH() {
    // TODO: Implementar lógica real de conversão de voltagem para pH
    // Atualmente retorna valor fixo para evitar leituras flutuantes sem sensor conectado
    return 7.0; 
}

void checkSensors() {
  sensors.requestTemperatures();
  float t = sensors.getTempCByIndex(0);
  current_ph = readPH();
  my_ph.updateAndReportParam("Temperature", current_ph); // Rainmaker usa param "Temperature" para floats genéricos às vezes

  // Proteção contra leitura inválida ou desconexão
  if (t == DEVICE_DISCONNECTED_C || t < -40 || t > 85) {
    digitalWrite(PIN_RELAY_HEATER, DESLIGADO);
    digitalWrite(PIN_COOLER_PWM, LOW);
    if(status_msg_param) status_msg_param->updateAndReport(esp_rmaker_str("ERRO SENSOR TEMP!"));
    addLog("⚠️ ERRO CRÍTICO: Sensor Temp!");
    return;
  }

  current_temp = t;
  my_temp.updateAndReportParam("Temperature", current_temp);
  updateStatusMsg();

  if (maintenance_mode) return; // Se estiver em TPA, não controla temperatura automaticamente

  // Controle do Termostato (Histerese)
  if (thermostat_auto) {
      if (current_temp > (target_temp + temp_hysteresis)) {
        // Precisa Esfriar
        digitalWrite(PIN_COOLER_PWM, HIGH);
        digitalWrite(PIN_RELAY_HEATER, DESLIGADO);
        if(cooler_p) cooler_p->updateAndReport(esp_rmaker_bool(true));
        if(heater_p) heater_p->updateAndReport(esp_rmaker_bool(false));
        
        if(!last_cooler_state) addLog("Termostato: Resfriando ❄️");
        last_cooler_state = true; last_heater_state = false;
      } 
      else if (current_temp < (target_temp - temp_hysteresis)) {
        // Precisa Aquecer
        digitalWrite(PIN_COOLER_PWM, LOW);
        digitalWrite(PIN_RELAY_HEATER, LIGADO);
        if(cooler_p) cooler_p->updateAndReport(esp_rmaker_bool(false));
        if(heater_p) heater_p->updateAndReport(esp_rmaker_bool(true));
        
        if(!last_heater_state) addLog("Termostato: Aquecendo 🔥");
        last_heater_state = true; last_cooler_state = false;
      } 
      else {
        // Zona Morta (Temperatura Ideal)
        digitalWrite(PIN_COOLER_PWM, LOW);
        digitalWrite(PIN_RELAY_HEATER, DESLIGADO);
        if(cooler_p) cooler_p->updateAndReport(esp_rmaker_bool(false));
        if(heater_p) heater_p->updateAndReport(esp_rmaker_bool(false));
        
        if(last_heater_state || last_cooler_state) addLog("Termostato: Temp Ideal ✅");
        last_heater_state = false; last_cooler_state = false;
      }
  }
}

// --- ALIMENTADOR AUTOMÁTICO ---
void feedFish() {
  Serial.println(">>> Iniciando ciclo de alimentação...");
  addLog("Alimentando Peixes... 🐟");
  
  // Movimento de "Shake" (v13.1) para evitar que a ração encrave
  // Move para trás
  stepper1.moveTo(-1700); stepper2.moveTo(-1700); 
  while (stepper1.distanceToGo() != 0 || stepper2.distanceToGo() != 0) {
    stepper1.run(); stepper2.run();
  }
  delay(500);
  
  // Move para o ponto zero
  stepper1.moveTo(0); stepper2.moveTo(0);
  while (stepper1.distanceToGo() != 0 || stepper2.distanceToGo() != 0) {
    stepper1.run(); stepper2.run();
  }

  // Trava Positiva Final
  stepper1.setCurrentPosition(0); stepper1.moveTo(50);          
  stepper2.setCurrentPosition(0); stepper2.moveTo(50);
  while (stepper1.distanceToGo() != 0 || stepper2.distanceToGo() != 0) {
    stepper1.run(); stepper2.run();
  }
  
  // Desliga bobinas para economizar energia e não esquentar o motor
  digitalWrite(M1_PIN1, LOW); digitalWrite(M1_PIN2, LOW); digitalWrite(M1_PIN3, LOW); digitalWrite(M1_PIN4, LOW);
  digitalWrite(M2_PIN1, LOW); digitalWrite(M2_PIN2, LOW); digitalWrite(M2_PIN3, LOW); digitalWrite(M2_PIN4, LOW);
  
  stepper1.setCurrentPosition(0);
  stepper2.setCurrentPosition(0);
  
  food_counter--;
  if(food_counter < 0) {
    food_counter = 0;
    addLog("⚠️ ALERTA: Ração acabou!");
  }
  
  // Desliga o botão no app automaticamente
  if(feeder_p) feeder_p->updateAndReport(esp_rmaker_bool(false));
  updateStatusMsg();
}

// --- AGENDAMENTO (LUZES) ---
void checkSchedule() {
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)) return; // Aguarda sincronia NTP

  if (maintenance_mode) return; // Não altera luzes durante manutenção

  bool should_be_on = false;
  int current_hour = timeinfo.tm_hour;

  // Lógica para ligar/desligar considerando virada de dia (ex: 22h as 06h)
  if (light_on_hour < light_off_hour) {
    if (current_hour >= light_on_hour && current_hour < light_off_hour) should_be_on = true;
  } else {
    if (current_hour >= light_on_hour || current_hour < light_off_hour) should_be_on = true;
  }

  // Só atua se o estado atual for diferente do desejado
  if (should_be_on && !light_state) {
      digitalWrite(PIN_RELAY_LIGHT, LIGADO);
      light_state = true;
      if(light_p) light_p->updateAndReport(esp_rmaker_bool(true));
      addLog("Timer: Luzes Ligadas 💡");
  } else if (!should_be_on && light_state && digitalRead(PIN_RELAY_LIGHT) == LIGADO) {
      digitalWrite(PIN_RELAY_LIGHT, DESLIGADO);
      light_state = false;
      if(light_p) light_p->updateAndReport(esp_rmaker_bool(false));
      addLog("Timer: Luzes Desligadas 🌙");
  }
}

// --- CALLBACK DO RAINMAKER (COMANDOS DO APP) ---
void write_callback(Device *device, Param *param, const param_val_t val, void *priv_data, write_ctx_t *ctx) {
  const char *device_name = device->getDeviceName();
  const char *param_name = param->getParamName();

  // Comandos de Liga/Desliga (Power)
  if (strcmp(param_name, "Power") == 0 || strcmp(param_name, "State") == 0) {
      
      if (strcmp(device_name, "Filtro") == 0) {
          filter_state = val.val.b;
          digitalWrite(PIN_RELAY_FILTER, filter_state ? LIGADO : DESLIGADO);
          addLog(filter_state ? "Filtro Ligado" : "Filtro Desligado");
      } 
      else if (strcmp(device_name, "Luzes") == 0) {
          light_state = val.val.b;
          digitalWrite(PIN_RELAY_LIGHT, light_state ? LIGADO : DESLIGADO);
          addLog(light_state ? "Luz Manual ON" : "Luz Manual OFF");
      }
      else if (strcmp(device_name, "Coolers") == 0) {
          // Ao acionar manual, desativa o automático
          if (thermostat_auto) {
             thermostat_auto = false;
             if(auto_p) auto_p->updateAndReport(esp_rmaker_bool(false));
          }
          digitalWrite(PIN_COOLER_PWM, val.val.b ? HIGH : LOW);
      }
      else if (strcmp(device_name, "Aquecedor") == 0) {
          if (thermostat_auto) {
             thermostat_auto = false;
             if(auto_p) auto_p->updateAndReport(esp_rmaker_bool(false));
          }
          digitalWrite(PIN_RELAY_HEATER, val.val.b ? LIGADO : DESLIGADO);
      }
      else if (strcmp(device_name, "Termostato Auto") == 0) {
          thermostat_auto = val.val.b;
          addLog(thermostat_auto ? "Termostato: Auto" : "Termostato: Manual");
          updateStatusMsg();
      }
      else if (strcmp(device_name, "Alimentar") == 0) {
          if (val.val.b == true) feedFish();
      }
      else if (strcmp(device_name, "Modo TPA") == 0) {
          maintenance_mode = val.val.b;
          if (maintenance_mode) {
             // Desliga tudo para segurança durante limpeza
             digitalWrite(PIN_RELAY_FILTER, DESLIGADO);
             digitalWrite(PIN_RELAY_HEATER, DESLIGADO);
             digitalWrite(PIN_COOLER_PWM, LOW);
             
             // Atualiza UI
             if(filter_p) filter_p->updateAndReport(esp_rmaker_bool(false));
             if(heater_p) heater_p->updateAndReport(esp_rmaker_bool(false));
             if(cooler_p) cooler_p->updateAndReport(esp_rmaker_bool(false));
             addLog("🔒 MODO TPA ATIVADO");
          } else {
             // Retorna filtro ao normal
             digitalWrite(PIN_RELAY_FILTER, LIGADO);
             if(filter_p) filter_p->updateAndReport(esp_rmaker_bool(true));
             addLog("🔓 TPA Finalizada");
          }
          updateStatusMsg();
      }
      else if (strcmp(device_name, "Reset Racao") == 0) {
          if (val.val.b == true) {
             food_counter = 100;
             updateStatusMsg();
             addLog("Ração Reabastecida (100%)");
             param->updateAndReport(esp_rmaker_bool(false)); // Botão de pulso
          }
      }
  }
  // Configuração de Horários
  else if (strcmp(param_name, "Hora Liga") == 0) {
      light_on_hour = val.val.i;
  }
  else if (strcmp(param_name, "Hora Desliga") == 0) {
      light_off_hour = val.val.i;
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  // 1. Configura Hora
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

  // 2. Inicializa NVS (Non-Volatile Storage) para WiFiProv
  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      err = nvs_flash_init();
  }
  ESP_ERROR_CHECK(err);
  
  // 3. Inicializa Sensores e Pinos
  Wire.begin(21, 22);
  sensors.begin();

  pinMode(PIN_RELAY_FILTER, OUTPUT);
  pinMode(PIN_RELAY_HEATER, OUTPUT);
  pinMode(PIN_RELAY_LIGHT, OUTPUT);
  pinMode(PIN_COOLER_PWM, OUTPUT);
  pinMode(PIN_SENSOR_PH, INPUT);

  // Estados Iniciais (Segurança)
  digitalWrite(PIN_RELAY_FILTER, LIGADO); // Filtro começa ligado
  digitalWrite(PIN_RELAY_HEATER, DESLIGADO);
  digitalWrite(PIN_RELAY_LIGHT, DESLIGADO);
  digitalWrite(PIN_COOLER_PWM, LOW);

  // Configuração Motores
  stepper1.setMaxSpeed(1000.0); stepper1.setAcceleration(500.0);
  stepper2.setMaxSpeed(1000.0); stepper2.setAcceleration(500.0);

  // --- TRAVAMENTO INICIAL MOTOR (Positivo) ---
  // Garante que o alimentador esteja travado/fechado ao ligar
  Serial.println("Calibrando alimentador...");
  stepper1.moveTo(1000); stepper2.moveTo(1000);
  unsigned long start = millis();
  // Time-out de 2 segundos para não travar o boot se o motor falhar
  while ((stepper1.distanceToGo() != 0 || stepper2.distanceToGo() != 0) && (millis() - start < 2000)) {
    stepper1.run(); stepper2.run();
  }
  stepper1.setCurrentPosition(0); stepper2.setCurrentPosition(0);
  
  // Desliga bobinas
  digitalWrite(M1_PIN1, LOW); digitalWrite(M1_PIN2, LOW); digitalWrite(M1_PIN3, LOW); digitalWrite(M1_PIN4, LOW);
  digitalWrite(M2_PIN1, LOW); digitalWrite(M2_PIN2, LOW); digitalWrite(M2_PIN3, LOW); digitalWrite(M2_PIN4, LOW);

  // 4. Inicializa RainMaker
  Node my_node;
  my_node = RMaker.initNode("AquaOS_Node");

  // Criação dos Dispositivos e Parâmetros
  // Filtro
  my_filter.addCb(write_callback);
  filter_p = new Param("Power", ESP_RMAKER_PARAM_POWER, value(true), PROP_FLAG_READ | PROP_FLAG_WRITE);
  filter_p->addUIType(ESP_RMAKER_UI_TOGGLE);
  my_filter.addParam(*filter_p);
  my_filter.assignPrimaryParam((param_handle_t*)filter_p->getParamHandle());
  my_node.addDevice(my_filter);

  // Coolers
  my_cooler.addCb(write_callback);
  cooler_p = new Param("Power", ESP_RMAKER_PARAM_POWER, value(false), PROP_FLAG_READ | PROP_FLAG_WRITE);
  cooler_p->addUIType(ESP_RMAKER_UI_TOGGLE);
  my_cooler.addParam(*cooler_p);
  my_cooler.assignPrimaryParam((param_handle_t*)cooler_p->getParamHandle());
  my_node.addDevice(my_cooler);

  // Aquecedor
  my_heater.addCb(write_callback);
  heater_p = new Param("Power", ESP_RMAKER_PARAM_POWER, value(false), PROP_FLAG_READ | PROP_FLAG_WRITE);
  heater_p->addUIType(ESP_RMAKER_UI_TOGGLE);
  my_heater.addParam(*heater_p);
  my_heater.assignPrimaryParam((param_handle_t*)heater_p->getParamHandle());
  my_node.addDevice(my_heater);

  // Termostato Auto
  my_auto_mode.addCb(write_callback);
  auto_p = new Param("Power", ESP_RMAKER_PARAM_POWER, value(true), PROP_FLAG_READ | PROP_FLAG_WRITE);
  auto_p->addUIType(ESP_RMAKER_UI_TOGGLE);
  my_auto_mode.addParam(*auto_p);
  my_auto_mode.assignPrimaryParam((param_handle_t*)auto_p->getParamHandle());
  my_node.addDevice(my_auto_mode);

  // Luzes com Sliders de Tempo
  my_light.addCb(write_callback);
  light_p = new Param("Power", ESP_RMAKER_PARAM_POWER, value(false), PROP_FLAG_READ | PROP_FLAG_WRITE);
  light_p->addUIType(ESP_RMAKER_UI_TOGGLE);
  my_light.addParam(*light_p);
  my_light.assignPrimaryParam((param_handle_t*)light_p->getParamHandle());
  
  light_on_param = new Param("Hora Liga", ESP_RMAKER_PARAM_RANGE, value(8), PROP_FLAG_READ | PROP_FLAG_WRITE);
  light_on_param->addUIType(ESP_RMAKER_UI_SLIDER);
  light_on_param->addBounds(value(0), value(23), value(1));
  my_light.addParam(*light_on_param);

  light_off_param = new Param("Hora Desliga", ESP_RMAKER_PARAM_RANGE, value(18), PROP_FLAG_READ | PROP_FLAG_WRITE);
  light_off_param->addUIType(ESP_RMAKER_UI_SLIDER);
  light_off_param->addBounds(value(0), value(23), value(1));
  my_light.addParam(*light_off_param);
  my_node.addDevice(my_light);

  // Modo Manutenção (TPA)
  my_maintenance.addCb(write_callback);
  maint_p = new Param("Power", ESP_RMAKER_PARAM_POWER, value(false), PROP_FLAG_READ | PROP_FLAG_WRITE);
  maint_p->addUIType(ESP_RMAKER_UI_TOGGLE);
  my_maintenance.addParam(*maint_p);
  my_maintenance.assignPrimaryParam((param_handle_t*)maint_p->getParamHandle());
  my_node.addDevice(my_maintenance);

  // Alimentador
  my_feeder.addCb(write_callback);
  feeder_p = new Param("Power", ESP_RMAKER_PARAM_POWER, value(false), PROP_FLAG_READ | PROP_FLAG_WRITE);
  feeder_p->addUIType(ESP_RMAKER_UI_TOGGLE);
  my_feeder.addParam(*feeder_p);
  my_feeder.assignPrimaryParam((param_handle_t*)feeder_p->getParamHandle());
  my_node.addDevice(my_feeder);

  // Reset de Ração
  my_reset_food.addCb(write_callback);
  reset_p = new Param("Power", ESP_RMAKER_PARAM_POWER, value(false), PROP_FLAG_READ | PROP_FLAG_WRITE);
  reset_p->addUIType(ESP_RMAKER_UI_TOGGLE);
  my_reset_food.addParam(*reset_p);
  my_reset_food.assignPrimaryParam((param_handle_t*)reset_p->getParamHandle());
  my_node.addDevice(my_reset_food);

  // Sensores (Visualização)
  status_msg_param = new Param("Info", ESP_RMAKER_PARAM_NAME, value("Iniciando..."), PROP_FLAG_READ);
  my_temp.addParam(*status_msg_param);
  my_node.addDevice(my_temp);

  my_node.addDevice(my_ph);

  // Histórico de Logs
  log_p = new Param("Logs", ESP_RMAKER_PARAM_NAME, value("Sem registros"), PROP_FLAG_READ);
  my_logger.addParam(*log_p);
  my_node.addDevice(my_logger);

  // Inicia Serviço
  RMaker.start();
  WiFiProv.beginProvision(WIFI_PROV_SCHEME_BLE, WIFI_PROV_SCHEME_HANDLER_FREE_BTDM, WIFI_PROV_SECURITY_1, PROV_POP, PROV_NAME);
}

void loop() {
  // Motores devem rodar constantemente se houver alvo definido
  stepper1.run();
  stepper2.run();

  // Verifica Temperatura a cada 5 segundos
  if (millis() - last_temp_check > CHECK_INTERVAL) {
    last_temp_check = millis();
    checkSensors();
  }

  // Verifica Agendamentos a cada 60 segundos
  if (millis() - last_schedule_check > 60000) {
    last_schedule_check = millis();
    checkSchedule();
  }
}