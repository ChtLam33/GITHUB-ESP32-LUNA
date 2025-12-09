/* ESP32 + TF-Luna (UART2 D16 RX / D17 TX)
   Lecture robuste, calcul volume, envoi JSON HTTPS,
   récupération auto de la config (get_config.php) toutes les 60s,
   configuration Wi-Fi locale via WiFiManager (appui long sur BOOT),
   sauvegarde du dernier Wi-Fi connu (persistant après coupure),
   indicateur de qualité Wi-Fi (RSSI),
   ET OTA HTTPS centralisé via ota_check.php.
*/

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>
#include <WiFiManager.h>  // https://github.com/tzapu/WiFiManager
#include <HTTPClient.h>
#include <Update.h>

// --- Identifiant matériel unique (généré à partir du chip ID) ---
String idCapteurStr;      // contiendra l'ID sous forme de String
const char* idCapteur = nullptr;  // pointeur vers les données de idCapteurStr

// --- VERSION FIRMWARE (à incrémenter à chaque nouvelle release) ---
const char* FIRMWARE_VERSION = "1.0.7";

// --- SERVEUR ---
const char* server   = "prod.lamothe-despujols.com";
const int   httpsPort = 443;

// --- CHEMINS OTA ---
const char* otaCheckPath = "/cuves/ota_check.php"; // renvoie JSON version + url

// --- BROCHE DU BOUTON BOOT ---
#define BOOT_PIN 0  // sur les ESP32 classiques, le bouton BOOT = GPIO0

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

// --- OTA TEMPO (par ex. toutes les 10 minutes) ---
unsigned long lastOtaCheck       = 0;
const unsigned long otaCheckInterval = 600000UL; // 10 minutes

int lastDistance = -1;

// --- Retry auto Wi-Fi ---
unsigned long lastWifiRetry       = 0;
const unsigned long wifiRetryInterval = 60000UL; // tentative toutes les 60 s si déconnecté


// --- TF-LUNA ---
HardwareSerial tfSerial(2); // UART2 (D16 RX / D17 TX)
uint8_t frameBuf[9];
int frameIdx = 0;


// === CONNEXION WIFI + WiFiManager ===
void setupWiFi() {
  pinMode(BOOT_PIN, INPUT_PULLUP);
  delay(200);

  Serial.println("\n🔌 Initialisation du Wi-Fi...");
  bool forceConfig = false;

  // Si le bouton BOOT est maintenu au démarrage → mode config Wi-Fi
  unsigned long start = millis();
  while (millis() - start < 3000) {
    if (digitalRead(BOOT_PIN) == LOW) {
      forceConfig = true;
      break;
    }
  }

  WiFiManager wm;
  wm.setDebugOutput(false);
  wm.setTimeout(180);  // portail actif max 3 minutes (au-delà, il rend la main)

  // --- Reconnexion automatique si Wi-Fi déjà connu ---
  WiFi.persistent(true);
  WiFi.mode(WIFI_STA);
  WiFi.begin();

  int tries = 0;
  // On passe de 20 tentatives (~10 s) à 60 tentatives (~30 s)
  while (WiFi.status() != WL_CONNECTED && tries < 60) {
    delay(500);
    Serial.print(".");
    tries++;
  }

  if (WiFi.status() == WL_CONNECTED && !forceConfig) {
    Serial.println("\n✅ Reconnexion Wi-Fi réussie sans portail !");
  } else {
    Serial.println("\n⚙️ Démarrage du portail Wi-FiManager...");
    if (forceConfig) {
      // BOOT maintenu → on force le portail de config
      if (!wm.startConfigPortal("Cuve_Config_AP")) {
        Serial.println("❌ Échec de configuration Wi-Fi (portail forcé)");
      }
    } else {
      // Tentative auto → si échec, on lance autoConnect puis portail si besoin
      if (!wm.autoConnect("Cuve_Config_AP")) {
        Serial.println("⚠️ Connexion Wi-Fi échouée, lancement AP config...");
        wm.startConfigPortal("Cuve_Config_AP");
      }
    }
  }

  // À ce stade, soit WiFiManager a trouvé un réseau, soit non
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("✅ Connecté au Wi-Fi : ");
    Serial.println(WiFi.SSID());
    Serial.print("Adresse IP : ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("❌ Toujours pas connecté au Wi-Fi après WiFiManager.");
  }

  Serial.print("Version firmware actuelle : ");
  Serial.println(FIRMWARE_VERSION);
}



// === ENVOI DES DONNÉES ===
void sendDataToServer(String jsonPayload) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("⚠️ Wi-Fi non connecté, envoi annulé");
    return;
  }

  WiFiClientSecure client;
  client.setInsecure();

  if (!client.connect(server, httpsPort)) {
    Serial.println("⚠️ Connexion HTTPS échouée (send)");
    return;
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

  Serial.println("➡️ Données envoyées au serveur :");
  Serial.println(jsonPayload);

  while (client.connected()) {
    String line = client.readStringUntil('\n');
    if (line == "\r") break;
  }
  String response = client.readString();
  Serial.println("Réponse serveur : " + response);
  client.stop();
}

// === RÉCUPÉRATION CONFIG SERVEUR ===
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


// === OTA : vérification et mise à jour ===
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

// === TF-LUNA ===
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

// === CALCUL DU VOLUME ===
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


// === SETUP ===
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n--- TF-Luna + WiFiManager + AutoConfig + RSSI + OTA ---");

  // === ID MATÉRIEL UNIQUE (44 bits → 11 caractères hex) ===
  uint64_t chipid  = ESP.getEfuseMac();                  // 48 bits uniques par ESP32
  uint64_t shortId = (chipid & 0xFFFFFFFFFFFULL);        // On garde 44 bits → 11 hex

  char buf[16];  // 11 caractères hex + null terminator + marge
  snprintf(buf, sizeof(buf), "%011llX", (unsigned long long)shortId);  // Exemple : "03FA91C2B7D"

  idCapteurStr = String(buf);
  idCapteur    = idCapteurStr.c_str();

  Serial.print("ID capteur matériel : ");
  Serial.println(idCapteur);
  // =======================================================

  // IMPORTANT : nomCuve doit être vide au démarrage
  nomCuve = "";

  // UART TF-Luna
  tfSerial.begin(115200, SERIAL_8N1, 16, 17);

  // Wi-Fi
  setupWiFi();

  // Récupération de la configuration serveur
  checkConfigUpdate();

  // Vérification OTA au démarrage
  checkForOTAUpdate();
}





// === LOOP ===
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
    lastNotifyMillis = now;

    if (lastDistance > 0) {
      float volumeCuveHL, capaciteCuveHL, pourcentage, hauteurPlein, hauteurCuve;
      String payloadText = buildStatusString(lastDistance, volumeCuveHL,
                                             capaciteCuveHL, pourcentage,
                                             hauteurPlein, hauteurCuve);

      Serial.println(payloadText);

      int rssi = WiFi.RSSI();
      String json = String("{\"id\":\"") + idCapteur +
                    "\",\"cuve\":\"" + nomCuve +
                    "\",\"distance\":" + String(lastDistance) +
                    ",\"volume\":" + String(volumeCuveHL, 2) +
                    ",\"capacite\":" + String(capaciteCuveHL, 2) +
                    ",\"pourcentage\":" + String(pourcentage, 2) +
                    ",\"hauteurPlein\":" + String(hauteurPlein) +
                    ",\"hauteurCuve\":" + String(hauteurCuve) +
                    ",\"correction\":" + String(AjustementHL, 2) +
                    ",\"rssi\":" + String(rssi) + "}";

      sendDataToServer(json);
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

  // ===================================================================
  // === RETRY AUTOMATIQUE DU WIFI SI NON CONNECTÉ (toutes les 60 s) ===
  // ===================================================================
  if (WiFi.status() != WL_CONNECTED) {
    if (now - lastWifiRetry >= wifiRetryInterval) {
      lastWifiRetry = now;

      Serial.println("🔁 Wi-Fi non connecté → tentative automatique...");

      // Reset propre du Wi-Fi (important si routeur refuse temporairement)
      WiFi.disconnect(true, true);
      delay(200);

      WiFi.mode(WIFI_STA);

      // Relance avec les derniers identifiants sauvegardés par WiFiManager
      WiFi.begin();
    }
  }
}

