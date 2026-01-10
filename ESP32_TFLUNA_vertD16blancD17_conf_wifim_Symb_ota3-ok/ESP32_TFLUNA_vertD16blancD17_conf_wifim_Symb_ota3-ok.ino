/* ESP32 + TF-Luna (UART2 D16 RX / D17 TX)
   Firmware ESP32 — Capteur cuve — v1.2.1

   Objectif v1.1.9 (simple & robuste, sans casser OTA) :
   - Désynchronisation AU DÉMARRAGE (anti tempête Freebox / DHCP / TLS)
   - Wi-Fi plus stable (auto-reconnect + pas d’écriture flash)
   - Petit jitter sur les ENVOIS uniquement (évite re-synchronisation dans le temps)
   - OTA / Config : inchangés dans leur logique

   IMPORTANT :
   - OTA continue de fonctionner via ota_check.php
   - Le JSON envoyé conserve : correction, hauteurPlein, hauteurCuve
   - (Optionnel) fw est envoyé au serveur si ton api_cuve.php le gère
*/

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>
#include <WiFiManager.h>  // https://github.com/tzapu/WiFiManager
#include <HTTPClient.h>
#include <Update.h>

// --- WiFiManager global ---
WiFiManager wm;

// --- Identifiant matériel unique ---
String idCapteurStr;
const char* idCapteur = nullptr;

// --- VERSION FIRMWARE ---
const char* FIRMWARE_VERSION = "1.1.9";

// --- SERVEUR ---
const char* server    = "prod.lamothe-despujols.com";
const int   httpsPort = 443;

// --- CHEMINS OTA ---
const char* otaCheckPath = "/cuves/ota_check.php"; // renvoie JSON version + url

// --- BROCHE DU BOUTON BOOT ---
#define BOOT_PIN 0  // sur ESP32 classique, bouton BOOT = GPIO0

// --- VARIABLES DE CUVE ---
String nomCuve = "";   // pas de valeur par défaut
float hauteurCapteurFond = 200.0;
float hauteurMaxLiquide  = 50.0;
float diametreCuve       = 70.0;
float AjustementHL       = 0.00;

// --- TEMPO ---
unsigned long lastNotifyMillis   = 0;
const unsigned long intervalMs   = 8000UL; // mesure luna toutes les 8 secondes
unsigned long lastConfigCheck    = 0;
const unsigned long configCheckInterval = 60000UL;

// --- OTA TEMPO ---
unsigned long lastOtaCheck       = 0;
const unsigned long otaCheckInterval = 180000UL; // 3 minutes

int lastDistance = -1;

// --- Watchdog d’envoi ---
unsigned long lastSuccessfulSend      = 0;
const unsigned long sendWatchdogDelay = 5UL * 60UL * 1000UL; // 5 minutes

// --- TF-LUNA ---
HardwareSerial tfSerial(2); // UART2 (D16 RX / D17 TX)
uint8_t frameBuf[9];
int frameIdx = 0;

// --- Anti tempête Freebox : désynchro + jitter (simple) ---
static const unsigned long START_DELAY_MAX_MS = 20000UL; // 0..20s au boot
static const unsigned long SEND_JITTER_MAX_MS = 2000UL;  // 0..2s après chaque envoi

// =====================================================
// === CONNEXION WIFI + WiFiManager ROBUSTE          ===
// =====================================================
void setupWiFi() {
  pinMode(BOOT_PIN, INPUT_PULLUP);
  delay(200);

  Serial.println("\n🔌 Initialisation du Wi-Fi...");
  bool forceConfig = false;

  // Si le bouton BOOT est maintenu au démarrage → mode config Wi-Fi forcé
  unsigned long start = millis();
  while (millis() - start < 3000) {
    if (digitalRead(BOOT_PIN) == LOW) {
      forceConfig = true;
      break;
    }
  }

  // Configuration WiFiManager
  wm.setDebugOutput(false);
  wm.setConnectTimeout(20);
  wm.setConfigPortalTimeout(180);
  wm.setBreakAfterConfig(true);

  WiFi.mode(WIFI_STA);

  // ✅ v1.1.9 : Wi-Fi plus stable / évite écritures flash inutiles
  WiFi.setAutoReconnect(true);
  WiFi.persistent(false);

  bool connected = false;

  if (forceConfig) {
    Serial.println("⚙️ BOOT maintenu → reset des identifiants Wi-Fi + portail forcé.");
    wm.resetSettings();
    connected = wm.startConfigPortal("Cuve_Config_AP");
  } else {
    connected = wm.autoConnect("Cuve_Config_AP");
  }

  if (!connected) {
    Serial.println("❌ Impossible de se connecter au Wi-Fi (ni via portail). Redémarrage dans 5 s...");
    delay(5000);
    ESP.restart();
  }

  Serial.print("✅ Connecté au Wi-Fi : ");
  Serial.println(WiFi.SSID());
  Serial.print("Adresse IP : ");
  Serial.println(WiFi.localIP());

  Serial.print("Version firmware actuelle : ");
  Serial.println(FIRMWARE_VERSION);

  lastSuccessfulSend = millis();
}

// =====================================================
// === ENVOI DES DONNÉES (retourne true si OK)       ===
// =====================================================
bool sendDataToServer(const String &jsonPayload) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("⚠️ Wi-Fi non connecté, envoi annulé");
    return false;
  }

  WiFiClientSecure client;
  client.setInsecure();

  if (!client.connect(server, httpsPort)) {
    Serial.println("⚠️ Connexion HTTPS échouée (send)");
    return false;
  }

  String url = "/cuves/api_cuve.php";
  client.println("POST " + url + " HTTP/1.1");
  client.println("Host: " + String(server));
  client.println("Content-Type: application/json");
  client.print("Content-Length: ");
  client.println(jsonPayload.length());
  client.println("Connection: close");
  client.println();
  client.print(jsonPayload);

  // Lecture minimale de la réponse HTTP
  while (client.connected()) {
    String line = client.readStringUntil('\n');
    if (line == "\r") break;
  }
  String response = client.readString();
  Serial.println("Réponse serveur : " + response);
  client.stop();

  return true;
}

// =====================================================
// === RÉCUPÉRATION CONFIG SERVEUR                  ===
// =====================================================
void checkConfigUpdate() {
  if (WiFi.status() != WL_CONNECTED) return;

  WiFiClientSecure client;
  client.setInsecure();

  if (!client.connect(server, httpsPort)) return;

  String url = "/cuves/get_config.php?id=" + String(idCapteur);
  client.println("GET " + url + " HTTP/1.1");
  client.println("Host: " + String(server));
  client.println("Connection: close");
  client.println();

  String payload;
  while (client.connected()) {
    String line = client.readStringUntil('\n');
    if (line == "\r") break;
  }
  while (client.available()) payload += client.readString();
  client.stop();

  int start = payload.indexOf('{');
  int end   = payload.lastIndexOf('}');
  if (start < 0 || end <= start) return;

  String jsonStr = payload.substring(start, end + 1);

  StaticJsonDocument<512> doc;
  if (deserializeJson(doc, jsonStr)) return;

  if (doc.containsKey("error")) {
    Serial.println("Config serveur absente → valeurs par défaut conservées.");
    return;
  }

  nomCuve            = doc["nomCuve"].as<String>();
  hauteurCapteurFond = doc["hauteurCapteurFond"].as<float>();
  hauteurMaxLiquide  = doc["hauteurMaxLiquide"].as<float>();
  diametreCuve       = doc["diametreCuve"].as<float>();
  AjustementHL       = doc["AjustementHL"].as<float>();

  Serial.println("Config appliquée depuis serveur.");
}

// =====================================================
// === OTA : vérification et mise à jour             ===
// =====================================================
void checkForOTAUpdate() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("OTA: Wi-Fi non connecté, skip");
    return;
  }

  Serial.println("\n🔎 OTA: Vérification de nouvelle version...");

  // 1) Récupérer JSON de ota_check.php
  WiFiClientSecure client;
  client.setInsecure();

  if (!client.connect(server, httpsPort)) {
    Serial.println("⚠️ OTA: connexion HTTPS échouée (ota_check)");
    return;
  }

  client.println(String("GET ") + otaCheckPath + " HTTP/1.1");
  client.println("Host: " + String(server));
  client.println("Connection: close");
  client.println();

  String payload;
  while (client.connected()) {
    String line = client.readStringUntil('\n');
    if (line == "\r") break;
  }
  while (client.available()) {
    payload += client.readString();
  }
  client.stop();

  int start = payload.indexOf('{');
  int end   = payload.lastIndexOf('}');
  if (start < 0 || end <= start) {
    Serial.println("⚠️ OTA: JSON version introuvable");
    Serial.println(payload);
    return;
  }
  String jsonStr = payload.substring(start, end + 1);

  StaticJsonDocument<512> doc;
  DeserializationError err = deserializeJson(doc, jsonStr);
  if (err) {
    Serial.println("⚠️ OTA: erreur parsing JSON version");
    Serial.println(jsonStr);
    return;
  }

  const char* newVersion = doc["version"] | "";
  String fwUrl           = doc["url"] | "";

  if (strlen(newVersion) == 0 || fwUrl.length() == 0) {
    Serial.println("⚠️ OTA: champs 'version' ou 'url' manquants");
    return;
  }

  Serial.print("OTA: version distante = ");
  Serial.println(newVersion);
  Serial.print("OTA: version locale   = ");
  Serial.println(FIRMWARE_VERSION);

  if (String(newVersion) == String(FIRMWARE_VERSION)) {
    Serial.println("OTA: firmware déjà à jour.");
    return;
  }

  Serial.println("✅ Nouvelle version détectée, lancement OTA...");
  Serial.print("URL firmware: ");
  Serial.println(fwUrl);

  // 2) Téléchargement et flash
  HTTPClient https;
  WiFiClientSecure fwClient;
  fwClient.setInsecure();

  if (!https.begin(fwClient, fwUrl)) {
    Serial.println("⚠️ OTA: impossible d'initialiser la requête HTTP");
    return;
  }

  int httpCode = https.GET();
  if (httpCode != HTTP_CODE_OK) {
    Serial.print("⚠️ OTA: code HTTP inattendu: ");
    Serial.println(httpCode);
    https.end();
    return;
  }

  int contentLength = https.getSize();
  if (contentLength <= 0) {
    Serial.println("⚠️ OTA: taille de firmware inconnue ou nulle");
    https.end();
    return;
  }

  WiFiClient *stream = https.getStreamPtr();

  Serial.printf("OTA: taille firmware = %d octets\n", contentLength);

  if (!Update.begin(contentLength)) {
    Serial.println("⚠️ OTA: échec Update.begin()");
    https.end();
    return;
  }

  size_t written = Update.writeStream(*stream);
  if (written != (size_t)contentLength) {
    Serial.printf("⚠️ OTA: écrit %u / %d octets\n", (unsigned)written, contentLength);
    Update.end();
    https.end();
    return;
  }

  if (!Update.end()) {
    Serial.println("⚠️ OTA: Update.end() a échoué");
    https.end();
    return;
  }

  if (!Update.isFinished()) {
    Serial.println("⚠️ OTA: mise à jour incomplète");
    https.end();
    return;
  }

  Serial.println("✅ OTA: mise à jour réussie, redémarrage...");
  https.end();
  delay(500);
  ESP.restart();
}

// =====================================================
// === TF-LUNA                                       ===
// =====================================================
bool validFrame(uint8_t *buf) {
  if (buf[0] != 0x59 || buf[1] != 0x59) return false;
  uint16_t sum = 0;
  for (int i = 0; i < 8; ++i) sum += buf[i];
  return (uint8_t)(sum & 0xFF) == buf[8];
}

void processFrame(uint8_t *buf) {
  int dist = buf[2] + (buf[3] << 8);
  lastDistance = dist;
}

// =====================================================
// === CALCUL DU VOLUME                               ===
// =====================================================
String buildStatusString(int distance, float &volumeCuveHL, float &capaciteCuveHL,
                         float &pourcentage, float &hauteurPlein, float &hauteurCuve) {

  hauteurCuve = hauteurCapteurFond - hauteurMaxLiquide;
  if (hauteurCuve <= 0) hauteurCuve = 1.0;

  hauteurPlein = hauteurCuve - (distance - hauteurMaxLiquide);
  if (hauteurPlein < 0) hauteurPlein = 0;
  if (hauteurPlein > hauteurCuve) hauteurPlein = hauteurCuve;

  pourcentage = (hauteurPlein / hauteurCuve) * 100.0;
  capaciteCuveHL = (3.14159265 * pow((diametreCuve / 2.0), 2) * hauteurCuve) / 100000.0;
  volumeCuveHL = (pourcentage / 100.0) * capaciteCuveHL + AjustementHL;

  if (isnan(pourcentage)) pourcentage = 0;
  if (isnan(capaciteCuveHL)) capaciteCuveHL = 0;
  if (isnan(volumeCuveHL)) volumeCuveHL = 0;

  char buf[200];
  snprintf(buf, sizeof(buf),
           "%s | Distance: %d cm | %.2f%% | %.2f / %.2f HL (+%.2f HL) | %.1f / %.1f cm | RSSI: %d dBm",
           nomCuve.c_str(),
           distance,
           pourcentage,
           volumeCuveHL,
           capaciteCuveHL,
           AjustementHL,
           hauteurPlein,
           hauteurCuve,
           WiFi.RSSI());

  return String(buf);
}

// =====================================================
// === SETUP                                          ===
// =====================================================
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n--- TF-Luna + WiFiManager + AutoConfig + RSSI + OTA ---");

  // === ID MATÉRIEL UNIQUE (44 bits → 11 caractères hex) ===
  uint64_t chipid  = ESP.getEfuseMac();
  uint64_t shortId = (chipid & 0xFFFFFFFFFFFULL);

  char buf[16];
  snprintf(buf, sizeof(buf), "%011llX", (unsigned long long)shortId);

  idCapteurStr = String(buf);
  idCapteur    = idCapteurStr.c_str();

  Serial.print("ID capteur matériel : ");
  Serial.println(idCapteur);

  // IMPORTANT : nomCuve doit être vide au démarrage
  nomCuve = "";

  // UART TF-Luna
  tfSerial.begin(115200, SERIAL_8N1, 16, 17);

  // ✅ v1.1.9 : Désynchronisation au démarrage (anti tempête Freebox)
  randomSeed((unsigned long)ESP.getEfuseMac());
  unsigned long startDelay = random(0UL, START_DELAY_MAX_MS + 1UL);
  Serial.print("⏳ Délai de démarrage aléatoire: ");
  Serial.print(startDelay);
  Serial.println(" ms");
  delay(startDelay);

  // Wi-Fi
  setupWiFi();

  // Config serveur
  checkConfigUpdate();

  // OTA au démarrage
  checkForOTAUpdate();

  // Timers
  lastNotifyMillis = millis();
  lastConfigCheck  = millis();
  lastOtaCheck     = millis();
}

// =====================================================
// === LOOP                                           ===
// =====================================================
void loop() {

  // --- Lecture du TF-Luna ---
  while (tfSerial.available()) {
    uint8_t b = tfSerial.read();
    if (frameIdx == 0) {
      if (b == 0x59) frameBuf[frameIdx++] = b;
    } else if (frameIdx == 1) {
      if (b == 0x59) frameBuf[frameIdx++] = b;
      else {
        frameIdx = 0;
        if (b == 0x59) { frameBuf[0] = b; frameIdx = 1; }
      }
    } else {
      frameBuf[frameIdx++] = b;
      if (frameIdx == 9) {
        if (validFrame(frameBuf)) processFrame(frameBuf);
        frameIdx = 0;
      }
    }
  }

  unsigned long now = millis();

  // --- Envoi périodique ---
  if (now - lastNotifyMillis >= intervalMs) {
    // ✅ v1.1.9 : petit jitter après chaque cycle d’envoi (anti re-synchro)
    lastNotifyMillis = now + random(0UL, SEND_JITTER_MAX_MS + 1UL);

    if (lastDistance > 0) {
      float volumeCuveHL, capaciteCuveHL, pourcentage, hauteurPlein, hauteurCuve;
      String payloadText = buildStatusString(lastDistance, volumeCuveHL,
                                             capaciteCuveHL, pourcentage,
                                             hauteurPlein, hauteurCuve);

      Serial.println(payloadText);

      int rssi = WiFi.RSSI();

      // JSON attendu par api_cuve.php (+ fw optionnel)
      String json = String("{\"id\":\"") + idCapteur +
                    "\",\"cuve\":\"" + nomCuve +
                    "\",\"distance\":" + String(lastDistance) +
                    ",\"volume\":" + String(volumeCuveHL, 2) +
                    ",\"capacite\":" + String(capaciteCuveHL, 2) +
                    ",\"pourcentage\":" + String(pourcentage, 2) +
                    ",\"hauteurPlein\":" + String(hauteurPlein, 1) +
                    ",\"hauteurCuve\":" + String(hauteurCuve, 1) +
                    ",\"correction\":" + String(AjustementHL, 2) +
                    ",\"rssi\":" + String(rssi) +
                    ",\"fw\":\"" + String(FIRMWARE_VERSION) + "\"}";

      bool ok = sendDataToServer(json);
      if (ok) {
        lastSuccessfulSend = now;
      }
    }
  }

  // --- Vérifie la config toutes les 60 secondes ---
  if (now - lastConfigCheck >= configCheckInterval) {
    lastConfigCheck = now;
    checkConfigUpdate();
  }

  // --- Vérifie OTA toutes les 10 minutes ---
  if (now - lastOtaCheck >= otaCheckInterval) {
    lastOtaCheck = now;
    checkForOTAUpdate();
  }

  // --- WATCHDOG : si plus d'envoi réussi pendant 5 minutes ---
  if (millis() - lastSuccessfulSend > sendWatchdogDelay) {
    Serial.println("⏱️ Plus de 5 minutes sans envoi réussi → redémarrage pour forcer reconnexion Wi-Fi...");
    delay(500);
    ESP.restart();
  }
}