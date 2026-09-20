/* ESP32 + TF-Luna (UART2 D16 RX / D17 TX)
   Firmware ESP32 — Capteur cuve — v1.3.1

   Changements v1.3.1 (TEST, a confirmer sur le terrain) :
   - Petit delai (300ms) ajoute juste apres WiFi.mode(WIFI_STA), avant
     la tentative de connexion - piste pour expliquer un rejet rapide
     observe au boot ("wifi:Association refused too many times, max
     allowed 1"). Si ca ne resout pas le probleme, a retirer en 1.3.2
     (strictement identique au 1.3.0 sinon).

   Changements v1.3.0 :
   - SIMPLIFICATION : le calcul du volume/%/hauteurs est desormais fait
     cote serveur (interpretCuve() dans cuves_lib.php) - le firmware
     n'envoie plus que la distance brute. Suppression de buildStatusString()
     et du polling de config toutes les 60s (checkConfigUpdate()), devenu
     inutile (plus rien a en tirer cote firmware).
   - CLE API : premiere inscription automatique aupres de /cuves/register.php
     avec un secret partage grave dans le firmware, cle recue stockee en
     NVS (Preferences) et reutilisee a chaque redemarrage - envoyee dans
     l'en-tete X-Api-Key de chaque requete.
   - CERTIFICAT HTTPS : les connexions verifient desormais le certificat
     du serveur (ISRG Root X1, verifie empiriquement contre la chaine
     reelle du serveur le 20/09/2026) au lieu de client.setInsecure().
     Necessite une synchronisation d'horloge (NTP) prealable. IMPORTANT :
     si la synchro NTP ou la validation echoue, repli automatique en mode
     non verifie (comme avant) plutot que de rester muet - priorite
     absolue a ne jamais perdre la capacite de recevoir un correctif OTA
     a distance (capteurs au plafond, difficiles d'acces).

   Objectif version (historique v1.2.3) :
   - Désynchronisation AU DÉMARRAGE (anti tempête Freebox / DHCP / TLS)
   - Wi-Fi plus stable (auto-reconnect + pas d’écriture flash)
   - Petit jitter sur les ENVOIS uniquement (évite re-synchronisation dans le temps)
   - FIX IMPORTANT : OTA uniquement si version distante STRICTEMENT supérieure à la version locale
*/

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>
#include <WiFiManager.h>  // https://github.com/tzapu/WiFiManager
#include <HTTPClient.h>
#include <Update.h>
#include <Preferences.h>
#include <time.h>
#include "secrets.h" // definit CUVE_PROVISIONING_SECRET - jamais commite (voir .gitignore + secrets.h.example)

// --- WiFiManager global ---
WiFiManager wm;

// --- Stockage persistant (cle API) ---
Preferences prefs;

// --- Identifiant matériel unique ---
String idCapteurStr;
const char* idCapteur = nullptr;

// --- VERSION FIRMWARE ---
const char* FIRMWARE_VERSION = "1.3.1";

// --- SERVEUR ---
const char* server    = "prod.lamothe-despujols.com";
const int   httpsPort = 443;

// --- CHEMINS SERVEUR ---
const char* otaCheckPath  = "/cuves/ota_check.php";  // renvoie JSON version + url
const char* registerPath  = "/cuves/register.php";   // premiere inscription -> cle API

// --- CLE API (obtenue une fois via registerPath, puis stockee en NVS) ---
// CUVE_PROVISIONING_SECRET vient de secrets.h (non commite) - secret
// PARTAGE par toute l'installation (pas par capteur), grave a la
// compilation - n'autorise que la toute premiere inscription d'un
// capteur. Doit correspondre EXACTEMENT a la valeur cote serveur
// (cuves/secrets.local.php).
String apiKey = ""; // vide tant que non inscrit

// --- Certificat racine (Let's Encrypt ISRG Root X1, valide jusqu'en 2035) ---
// Recupere le 20/09/2026 depuis https://letsencrypt.org/certs/isrgrootx1.pem
// et verifie empiriquement contre la chaine reelle servie par le serveur
// (openssl verify -CAfile isrgrootx1.pem ... -> OK).
const char* ISRG_ROOT_X1 = R"CERT(-----BEGIN CERTIFICATE-----
MIIFazCCA1OgAwIBAgIRAIIQz7DSQONZRGPgu2OCiwAwDQYJKoZIhvcNAQELBQAw
TzELMAkGA1UEBhMCVVMxKTAnBgNVBAoTIEludGVybmV0IFNlY3VyaXR5IFJlc2Vh
cmNoIEdyb3VwMRUwEwYDVQQDEwxJU1JHIFJvb3QgWDEwHhcNMTUwNjA0MTEwNDM4
WhcNMzUwNjA0MTEwNDM4WjBPMQswCQYDVQQGEwJVUzEpMCcGA1UEChMgSW50ZXJu
ZXQgU2VjdXJpdHkgUmVzZWFyY2ggR3JvdXAxFTATBgNVBAMTDElTUkcgUm9vdCBY
MTCCAiIwDQYJKoZIhvcNAQEBBQADggIPADCCAgoCggIBAK3oJHP0FDfzm54rVygc
h77ct984kIxuPOZXoHj3dcKi/vVqbvYATyjb3miGbESTtrFj/RQSa78f0uoxmyF+
0TM8ukj13Xnfs7j/EvEhmkvBioZxaUpmZmyPfjxwv60pIgbz5MDmgK7iS4+3mX6U
A5/TR5d8mUgjU+g4rk8Kb4Mu0UlXjIB0ttov0DiNewNwIRt18jA8+o+u3dpjq+sW
T8KOEUt+zwvo/7V3LvSye0rgTBIlDHCNAymg4VMk7BPZ7hm/ELNKjD+Jo2FR3qyH
B5T0Y3HsLuJvW5iB4YlcNHlsdu87kGJ55tukmi8mxdAQ4Q7e2RCOFvu396j3x+UC
B5iPNgiV5+I3lg02dZ77DnKxHZu8A/lJBdiB3QW0KtZB6awBdpUKD9jf1b0SHzUv
KBds0pjBqAlkd25HN7rOrFleaJ1/ctaJxQZBKT5ZPt0m9STJEadao0xAH0ahmbWn
OlFuhjuefXKnEgV4We0+UXgVCwOPjdAvBbI+e0ocS3MFEvzG6uBQE3xDk3SzynTn
jh8BCNAw1FtxNrQHusEwMFxIt4I7mKZ9YIqioymCzLq9gwQbooMDQaHWBfEbwrbw
qHyGO0aoSCqI3Haadr8faqU9GY/rOPNk3sgrDQoo//fb4hVC1CLQJ13hef4Y53CI
rU7m2Ys6xt0nUW7/vGT1M0NPAgMBAAGjQjBAMA4GA1UdDwEB/wQEAwIBBjAPBgNV
HRMBAf8EBTADAQH/MB0GA1UdDgQWBBR5tFnme7bl5AFzgAiIyBpY9umbbjANBgkq
hkiG9w0BAQsFAAOCAgEAVR9YqbyyqFDQDLHYGmkgJykIrGF1XIpu+ILlaS/V9lZL
ubhzEFnTIZd+50xx+7LSYK05qAvqFyFWhfFQDlnrzuBZ6brJFe+GnY+EgPbk6ZGQ
3BebYhtF8GaV0nxvwuo77x/Py9auJ/GpsMiu/X1+mvoiBOv/2X/qkSsisRcOj/KK
NFtY2PwByVS5uCbMiogziUwthDyC3+6WVwW6LLv3xLfHTjuCvjHIInNzktHCgKQ5
ORAzI4JMPJ+GslWYHb4phowim57iaztXOoJwTdwJx4nLCgdNbOhdjsnvzqvHu7Ur
TkXWStAmzOVyyghqpZXjFaH3pO3JLF+l+/+sKAIuvtd7u+Nxe5AW0wdeRlN8NwdC
jNPElpzVmbUq4JUagEiuTDkHzsxHpFKVK7q4+63SM1N95R1NbdWhscdCb+ZAJzVc
oyi3B43njTOQ5yOf+1CceWxG1bQVs5ZufpsMljq4Ui0/1lvh+wjChP4kqKOJ2qxq
4RgqsahDYVvTH9w7jXbyLeiNdd8XM2w9U/t7y0Ff/9yi0GE44Za4rF2LN9d11TPA
mRGunUHBcnWEvgJBQl9nJEiU0Zsnvgc/ubhPgXRR4Xq37Z0j4r7g1SgEEzwxA57d
emyPxgcYxn/eR44/KJ4EBs+lVDR3veyJm+kXQ99b21/+jh5Xos1AnX5iItreGCc=
-----END CERTIFICATE-----
)CERT";

// --- Etat NTP (voir syncTimeNTP() / connectSecure()) ---
bool timeIsSynced = false;

// --- BROCHE DU BOUTON BOOT ---
#define BOOT_PIN 0  // sur ESP32 classique, bouton BOOT = GPIO0

// --- VARIABLES DE CUVE ---
String nomCuve = "";   // pas de valeur par défaut

// --- TEMPO ---
unsigned long lastNotifyMillis   = 0;
const unsigned long intervalMs   = 8000UL; // mesure luna toutes les 8 secondes

// --- OTA TEMPO --- mise à jour OTA seulement au démarrage.
//unsigned long lastOtaCheck       = 0;
//const unsigned long otaCheckInterval = 180000UL; // 3 minutes

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
// === OUTILS : comparaison de versions "x.y.z"      ===
// =====================================================

// Parse "1.2.3" -> a,b,c. Retourne false si format invalide.
bool parseSemver3(const String& s, int &a, int &b, int &c) {
  a = b = c = 0;

  String t = s;
  t.trim();
  if (t.length() == 0) return false;

  int p1 = t.indexOf('.');
  if (p1 < 0) return false;
  int p2 = t.indexOf('.', p1 + 1);
  if (p2 < 0) return false;

  String sa = t.substring(0, p1);
  String sb = t.substring(p1 + 1, p2);
  String sc = t.substring(p2 + 1);

  sa.trim(); sb.trim(); sc.trim();
  if (sa.length() == 0 || sb.length() == 0 || sc.length() == 0) return false;

  // Vérif chiffres uniquement (évite "1.2.3-beta")
  for (size_t i = 0; i < sa.length(); i++) if (!isDigit(sa[i])) return false;
  for (size_t i = 0; i < sb.length(); i++) if (!isDigit(sb[i])) return false;
  for (size_t i = 0; i < sc.length(); i++) if (!isDigit(sc[i])) return false;

  a = sa.toInt();
  b = sb.toInt();
  c = sc.toInt();
  return true;
}

// Retourne 1 si vA > vB, 0 si égal, -1 si vA < vB, et -2 si invalide
int compareSemver3(const String& vA, const String& vB) {
  int a1,b1,c1, a2,b2,c2;
  if (!parseSemver3(vA, a1,b1,c1)) return -2;
  if (!parseSemver3(vB, a2,b2,c2)) return -2;

  if (a1 != a2) return (a1 > a2) ? 1 : -1;
  if (b1 != b2) return (b1 > b2) ? 1 : -1;
  if (c1 != c2) return (c1 > c2) ? 1 : -1;
  return 0;
}


// =====================================================
// === HEURE (NTP) + CONNEXION HTTPS AVEC REPLI      ===
// =====================================================

// La validation de certificat exige une horloge a peu pres juste (sinon
// le certificat parait "pas encore valide"/"expire"). Tentative courte
// (10s max) - si ca echoue, timeIsSynced reste false et connectSecure()
// se rabat sur le mode non verifie (voir plus bas).
bool syncTimeNTP() {
  configTime(0, 0, "pool.ntp.org", "time.google.com");

  time_t now = time(nullptr);
  unsigned long start = millis();
  const unsigned long NTP_TIMEOUT_MS = 10000UL;
  const time_t SANE_EPOCH_MIN = 1700000000; // ~nov. 2023, largement avant toute utilisation reelle

  while (now < SANE_EPOCH_MIN && millis() - start < NTP_TIMEOUT_MS) {
    delay(200);
    now = time(nullptr);
  }

  return now >= SANE_EPOCH_MIN;
}

// Etablit la connexion HTTPS vers le serveur. Verifie le certificat si
// l'heure est synchronisee ; sinon (ou si la tentative verifiee echoue),
// se replie automatiquement sur un mode non verifie plutot que de rester
// muet - priorite absolue a ne jamais perdre la capacite d'envoyer des
// mesures ou de recevoir un correctif OTA (capteurs au plafond).
bool connectSecure(WiFiClientSecure &client) {
  if (timeIsSynced) {
    client.setCACert(ISRG_ROOT_X1);
    if (client.connect(server, httpsPort)) {
      return true;
    }
    client.stop();
    Serial.println("⚠️ Connexion vérifiée (certificat) échouée, repli en mode non vérifié.");
  } else {
    Serial.println("⚠️ Heure non synchronisée (NTP), connexion en mode non vérifié.");
  }
  client.setInsecure();
  return client.connect(server, httpsPort);
}


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

  // Wi-Fi plus stable / évite écritures flash inutiles
  WiFi.setAutoReconnect(true);
  WiFi.persistent(false);

  // v1.3.1 - TEST : petit delai pour laisser la puce radio finir de
  // s'initialiser/se stabiliser juste apres WiFi.mode(), avant de tenter
  // une connexion. Piste pour expliquer "wifi:Association refused too
  // many times, max allowed 1" observe au boot (le code tente la
  // connexion plus vite que ce que la puce peut encaisser) - hypothese a
  // valider sur le terrain, pas une certitude. Si ca n'aide pas, a
  // retirer en 1.3.2 (identique au 1.3.0 sinon).
  delay(300);

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

  timeIsSynced = syncTimeNTP();
  Serial.println(timeIsSynced
    ? "✅ Heure synchronisée (NTP) - connexions HTTPS vérifiées."
    : "⚠️ Échec synchro NTP - connexions HTTPS en mode non vérifié pour cette session.");

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
  if (!connectSecure(client)) {
    Serial.println("⚠️ Connexion HTTPS échouée (send)");
    return false;
  }

  String url = "/cuves/api_cuve.php";
  client.println("POST " + url + " HTTP/1.1");
  client.println("Host: " + String(server));
  client.println("Content-Type: application/json");
  if (apiKey.length() > 0) {
    client.println("X-Api-Key: " + apiKey);
  }
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
// === INSCRIPTION (clé API) — une seule fois          ===
// =====================================================
// Appelée uniquement si aucune clé n'est encore en NVS (voir setup()).
// Envoie le secret partagé de l'installation ; si accepté, stocke la
// clé propre à ce capteur en NVS (persistante) pour les prochains
// redémarrages - registerPath n'est alors plus jamais rappelé.
void registerWithServer() {
  if (WiFi.status() != WL_CONNECTED) return;

  String json = String("{\"id\":\"") + idCapteur +
                "\",\"cuve\":\"" + nomCuve +
                "\",\"secret\":\"" + CUVE_PROVISIONING_SECRET + "\"}";

  WiFiClientSecure client;
  if (!connectSecure(client)) {
    Serial.println("⚠️ Inscription: connexion HTTPS échouée");
    return;
  }

  client.println(String("POST ") + registerPath + " HTTP/1.1");
  client.println("Host: " + String(server));
  client.println("Content-Type: application/json");
  client.print("Content-Length: ");
  client.println(json.length());
  client.println("Connection: close");
  client.println();
  client.print(json);

  String payload;
  while (client.connected()) {
    String line = client.readStringUntil('\n');
    if (line == "\r") break;
  }
  while (client.available()) payload += client.readString();
  client.stop();

  int start = payload.indexOf('{');
  int end   = payload.lastIndexOf('}');
  if (start < 0 || end <= start) {
    Serial.println("⚠️ Inscription: réponse inattendue");
    return;
  }

  String jsonStr = payload.substring(start, end + 1);
  StaticJsonDocument<256> doc;
  if (deserializeJson(doc, jsonStr)) {
    Serial.println("⚠️ Inscription: erreur parsing JSON");
    return;
  }

  const char* receivedKey = doc["api_key"] | "";
  if (strlen(receivedKey) == 0) {
    Serial.println("⚠️ Inscription refusée (secret invalide ?)");
    return;
  }

  apiKey = String(receivedKey);
  prefs.begin("cuve", false);
  prefs.putString("api_key", apiKey);
  prefs.end();

  Serial.println("✅ Inscription réussie, clé API obtenue et enregistrée.");
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
  if (!connectSecure(client)) {
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

  const char* newVersionC = doc["version"] | "";
  String fwUrl            = doc["url"] | "";

  String newVersion = String(newVersionC);
  newVersion.trim();

  if (newVersion.length() == 0 || fwUrl.length() == 0) {
    Serial.println("⚠️ OTA: champs 'version' ou 'url' manquants");
    return;
  }

  Serial.print("OTA: version distante = ");
  Serial.println(newVersion);
  Serial.print("OTA: version locale   = ");
  Serial.println(FIRMWARE_VERSION);

  // ✅ FIX v1.2.3 : OTA uniquement si distante STRICTEMENT supérieure
  int cmp = compareSemver3(newVersion, String(FIRMWARE_VERSION));
  if (cmp == -2) {
    Serial.println("⚠️ OTA: format de version invalide (attendu x.y.z). OTA annulée.");
    return;
  }
  if (cmp <= 0) {
    Serial.println("OTA: pas de mise à jour (distante <= locale).");
    return;
  }

  Serial.println("✅ Nouvelle version STRICTEMENT supérieure détectée, lancement OTA...");
  Serial.print("URL firmware: ");
  Serial.println(fwUrl);

  // 2) Téléchargement et flash
  // HTTPClient gère lui-même connect()/deconnect() - le repli "vérifié
  // puis non vérifié" est donc fait en 2 tentatives explicites ici
  // plutôt que via connectSecure() (pensé pour un WiFiClientSecure
  // utilisé directement). Même logique/priorité : ne jamais rester
  // bloqué sans pouvoir récupérer un correctif OTA.
  HTTPClient https;
  WiFiClientSecure fwClient;
  bool fwVerified = timeIsSynced;
  if (fwVerified) {
    fwClient.setCACert(ISRG_ROOT_X1);
  } else {
    fwClient.setInsecure();
  }

  if (!https.begin(fwClient, fwUrl)) {
    Serial.println("⚠️ OTA: impossible d'initialiser la requête HTTP");
    return;
  }

  int httpCode = https.GET();

  if (httpCode <= 0 && fwVerified) {
    Serial.println("⚠️ OTA: échec en mode vérifié, nouvelle tentative en mode non vérifié...");
    https.end();
    fwClient.stop();
    fwClient.setInsecure();
    if (!https.begin(fwClient, fwUrl)) {
      Serial.println("⚠️ OTA: impossible d'initialiser la requête HTTP (repli)");
      return;
    }
    httpCode = https.GET();
  }

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

  nomCuve = "";

  // UART TF-Luna
  tfSerial.begin(115200, SERIAL_8N1, 16, 17);

  // Désynchronisation au démarrage (anti tempête Freebox)
  randomSeed((unsigned long)ESP.getEfuseMac());
  unsigned long startDelay = random(0UL, START_DELAY_MAX_MS + 1UL);
  Serial.print("⏳ Délai de démarrage aléatoire: ");
  Serial.print(startDelay);
  Serial.println(" ms");
  delay(startDelay);

  // Wi-Fi (+ synchro NTP, voir setupWiFi())
  setupWiFi();

  // Clé API : chargée depuis la NVS si déjà inscrit, sinon inscription
  // une seule fois auprès du serveur (voir registerWithServer()).
  prefs.begin("cuve", true); // lecture seule
  apiKey = prefs.getString("api_key", "");
  prefs.end();

  if (apiKey.length() == 0) {
    Serial.println("🔑 Aucune clé API en mémoire, inscription auprès du serveur...");
    registerWithServer();
  } else {
    Serial.println("🔑 Clé API déjà enregistrée (NVS).");
  }

  // OTA au démarrage (mais uniquement si distante > locale)
  checkForOTAUpdate();

  // Timers
  lastNotifyMillis = millis();
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
    // petit jitter après chaque cycle d’envoi (anti re-synchro)
    lastNotifyMillis = now + random(0UL, SEND_JITTER_MAX_MS + 1UL);

    if (lastDistance > 0) {
      int rssi = WiFi.RSSI();

      Serial.printf("%s | Distance: %d cm | RSSI: %d dBm\n",
                    nomCuve.c_str(), lastDistance, rssi);

      // Payload minimal : le calcul volume/%/hauteurs se fait desormais
      // cote serveur (interpretCuve() dans cuves_lib.php) a partir de la
      // seule distance brute.
      String json = String("{\"id\":\"") + idCapteur +
                    "\",\"cuve\":\"" + nomCuve +
                    "\",\"distance\":" + String(lastDistance) +
                    ",\"rssi\":" + String(rssi) +
                    ",\"fw\":\"" + String(FIRMWARE_VERSION) + "\"}";

      bool ok = sendDataToServer(json);
      if (ok) {
        lastSuccessfulSend = now;
      }

      // Si l'inscription initiale a echoue (WiFi capricieux au tout
      // premier boot, etc.), on retente ici tant qu'aucune cle n'est
      // obtenue - sans bloquer l'envoi des mesures pour autant (qui
      // fonctionne deja sans cle pendant la transition, voir
      // isValidCuveApiKey() cote serveur).
      if (apiKey.length() == 0) {
        registerWithServer();
      }
    }
  }

  // --- WATCHDOG : si plus d'envoi réussi pendant 5 minutes ---
  if (millis() - lastSuccessfulSend > sendWatchdogDelay) {
    Serial.println("⏱️ Plus de 5 minutes sans envoi réussi → redémarrage pour forcer reconnexion Wi-Fi...");
    delay(500);
    ESP.restart();
  }
}