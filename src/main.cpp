// ----------------------------------------------------------------------
// OpenMarine ESP32 MFD
// Copyright (c) 2026 T. Weinreich
// Licensed under the MIT License. See LICENSE file in the project root.
// ----------------------------------------------------------------------
// ACHTUNG: Keine Garantie für Navigation oder Autopilot-Steuerung.
// Nutzung auf eigene Gefahr!
// ----------------------------------------------------------------------


// ── Versionsinformation ────────────────────────────────────────────────────────
#define FW_VERSION      "0.5.2"
#define FW_DATE         "2026-05-10"
#define FW_DESCRIPTION  "Pilot WP Labl Sichtbarkeit - update_displays() lv_obj_add_flag und lv_obj_clear_flag HIDDEN überarbeitet und TWS und APS mehr gedämpft"
// ───────────────────────────────────────────────────────────────────────────────


#include <Arduino.h>
#include <lvgl.h>
#include "display.h"
#include <NMEA2000_CAN.h>
#include <N2kMessages.h>
#include <Wire.h>
#include <TAMC_GT911.h>
#include <math.h>

#include "driver/twai.h"

//────────────────────────────────────────────────────────────────────────────

// ── Backlight LEDC ─────────────────────────────────────────────────────────────
#define BL_PIN  38
#define BL_FREQ 1000
#define BL_BITS 8



// ── Eigene Schriftarten ────────────────────────────────────────────────────────
LV_FONT_DECLARE(montserrat_240_bold_numeric);
LV_FONT_DECLARE(montserrat_120);
LV_FONT_DECLARE(montserrat_72);
LV_FONT_DECLARE(montserrat_48);
LV_FONT_DECLARE(montserrat_36);
LV_FONT_DECLARE(montserrat_18);

// ── WATCHDOG stuff ────────────────────────────────────────────────────────
TaskHandle_t nmea_task_handle = NULL; // global, statt NULL in xTaskCreate
// Globale Watchdog-Timestamps (volatile, aus verschiedenen Tasks geschrieben)
volatile uint32_t wdg_lvgl_ms   = 0;  // vom LVGL-Timer-Callback gesetzt



// ── Touch ──────────────────────────────────────────────────────────────────────
#define TP_SDA 19
#define TP_SCL 45
TAMC_GT911 touch(TP_SDA, TP_SCL, -1, -1, 480, 480);

// ── LVGL Puffer ────────────────────────────────────────────────────────────────
static lv_disp_draw_buf_t draw_buf;
static lv_color_t *buf1;
static lv_color_t *buf2;

// Global
static lv_obj_t* nmea_dot = nullptr; //DOT fuer Datenalter NMEA2000

// ═══════════════════════════════════════════════════════════════════════════════
// NMEA Datenwerte
// ═══════════════════════════════════════════════════════════════════════════════
volatile float n_depth      = -1.0f;
volatile float n_water_temp = -1.0f;
volatile float n_stw        = -1.0f;
volatile float n_wind_spd   = -1.0f;
volatile float n_wind_ang   = -1.0f;
volatile float n_heading    = -1.0f;
volatile float n_rudder     = -1.0f;
double         n_lat        = 1000.0;
double         n_lon        = 1000.0;
volatile float n_gps_alt    = -1.0f;

volatile float n_sog      = -1.0f;
volatile float n_tws      = -1.0f;
volatile float n_twa      = -1.0f;

// Neue volatile Timestamps — neben den n_xxx Werten
volatile uint32_t n_ts_depth  = 0;
volatile uint32_t n_ts_sog    = 0;
volatile uint32_t n_ts_stw    = 0;
volatile uint32_t n_ts_wind   = 0;
volatile uint32_t n_ts_hdg    = 0;
volatile uint32_t n_ts_wtemp  = 0;
volatile uint32_t n_ts_gps    = 0;
volatile uint32_t n_ts_dtw    = 0;



volatile int n_autopilot_mode = -1;   // -1 = unbekannt, Werte siehe Tabelle

// ── NEU: Globale Variable für EV-1 Quelladresse ──────────────────────────────
static int g_pilot_addr = -1;  // wird automatisch gefunden
static float ap_target_hdg = -1.0f;  // -1 = unbekannt Kurskorrektur Daten wahrscheinlich
static float ap_locked_wind = 0.0f;  // Gesperrter Windwinkel in Grad, 0=unbekannt
static float ap_target_wind = -999.0f;  // -999 = unbekannt
static bool  sim_mode_active = false;   // true = NMEA-AP-Daten ignorieren (Simulation)

// Volatile NMEA-Werte
volatile float n_dtw    = -1.0f;   // Distance to Waypoint, NM
volatile float n_ttg    = -1.0f;   // Time to Go, Minuten
static volatile char    n_wpt_name[20] = "---";  // Waypoint-Name (kein volatile nötig, nur NMEA-Task schreibt)

// Snapshot-Kopien
float s_dtw = -1.0f;
float s_ttg = -1.0f;
char  s_wpt_name[20] = "---";

// Neues globales Label für die TRCK-Infoleiste
static lv_obj_t* pilot_wpt_lbl  = nullptr;
static lv_obj_t* pilot_wpt_info = nullptr;


#define NMEA_DEBUG false 
#define AUTOPILOT_DEBUG false


// ═══════════════════════════════════════════════════════════════════════════════
// Screen-Definitionen & Globals
// ═══════════════════════════════════════════════════════════════════════════════
#define NUM_SCREENS 11
static lv_obj_t *screens[NUM_SCREENS];
static lv_obj_t *lbl_main[NUM_SCREENS];
static lv_obj_t *lbl_sub[NUM_SCREENS];

// Swipe-Erkennung
static lv_coord_t swipe_start_x = 0, swipe_start_y = 0;
static lv_coord_t swipe_last_x  = 0, swipe_last_y  = 0;
static bool       swipe_active  = false;
static uint32_t   swipe_start_ms = 0;
#define SWIPE_MIN_DIST  90    // px Mindestweg
#define SWIPE_MAX_MS   700    // ms Maximale Dauer

static bool swipe_blocked = false;  // true = dieser Touch gehört einem Widget


// -1 = nichts  |  0..10 = absoluter Screen  |  -2 = relative -1  |  -3 = relative +1
static volatile int8_t swipe_nav_pending = -1;

static bool          nav_busy      = false;


// ═══════════════════════════════════════════════════════════════════════════════
// Wind Screen Konstanten & UI Zeiger
// ═══════════════════════════════════════════════════════════════════════════════
#define CIRCLE_Y_OFFSET  25
#define WIND_CX           240
#define WIND_CY           240

#define R_OUTER           225
#define R_COLOR           220
#define R_SCALE           205
#define R_SCALE_INNER     165
#define R_GREY            142
#define R_CENTER          142

// WICHTIG: Hier fehlte in deinem Code vorher das Semikolon beim active_nav_overlay!
static lv_obj_t   *wind_ang_lbl    = nullptr;
static lv_obj_t   *wind_sog_box    = nullptr;
static lv_obj_t   *wind_depth_box  = nullptr;
static lv_obj_t   *aws_int_lbl     = nullptr;
static lv_obj_t   *aws_dec_lbl     = nullptr;

static lv_obj_t   *wind_canvas     = nullptr;
static lv_color_t *canvas_buf      = nullptr;

static float current_awa_damped = 0.0f;
static const float DAMPING_FACTOR = 12.0f;
static float current_wspd_damped = 0.0f;       // gedämpfte Windgeschwindigkeit (AWS oder TWS)
static const float WSPD_DAMPING_FACTOR = 20.0f; // höher = träger; 20 ≈ ~1 Sekunde bei 50ms-Timer

enum WindMode { MODE_AWS_AWA, MODE_TWS_TWA };
static WindMode wind_mode = MODE_AWS_AWA;

// Heading Analog Screen (neu)
#define HDG_ROSE_SIZE       750
#define HDG_RADIUS          355
#define HDG_CENTER          (HDG_ROSE_SIZE / 2)

static lv_obj_t   *hdg_rose_img = nullptr;
static lv_color_t *hdg_rose_buf = nullptr;
static lv_obj_t   *hdg_rose_canvas = nullptr;

static lv_obj_t   *hdg_labels[16];          // Labels für Gradzahlen und Himmelsrichtungen
static float      hdg_label_base_angles[16];
static bool       hdg_label_is_dir[16];
static int        hdg_label_count = 0;

static float current_hdg_damped = 0.0f;
static const float HDG_DAMPING_FACTOR = 15.0f;

static lv_obj_t *hdg_analog_val_lbl = nullptr;  // ← NEU

static lv_obj_t* multi_lbl[6] = {};  // [0]=SOG [1]=HDG [2]=AWS [3]=AWA [4]=DEPTH [5]=TWS
static int multi_target = 10;


// ── Backlight ──────────────────────────────────────────────────────────────────
static lv_obj_t *slider_brightness = nullptr;
static lv_obj_t *label_brightness  = nullptr;

static lv_obj_t *mode_btn_label = nullptr;
static lv_obj_t *wind_speed_label = nullptr;


//=================================================================================================
//=================================================================================================
//=================================================================================================

volatile uint32_t last_nmea_msg_ms = 0;

// ── NEU: Send-Queue statt Mutex ─────────────────────────────────────────────
struct NmeaSendMsg { tN2kMsg msg; };
QueueHandle_t nmea_send_queue = nullptr;   // ← statt nmea_mutex

// Lokale Snapshot-Kopien für LVGL-Thread (nie direkt n_xxx lesen!)
float s_depth = -1.0f, s_sog = -1.0f, s_stw = -1.0f;
float s_wind_spd = -1.0f, s_wind_ang = -1.0f;
float s_heading = -1.0f, s_twa = -999.0f, s_tws = -1.0f;
float s_rudder = -1.0f, s_water_temp = -1.0f;
double s_lat = 1000.0, s_lon = 1000.0;
int s_autopilot_mode = -1;
// Snapshots (neben s_depth etc.)
uint32_t s_ts_depth = 0, s_ts_sog = 0, s_ts_stw = 0;
uint32_t s_ts_wind  = 0, s_ts_hdg = 0, s_ts_wtemp = 0;
uint32_t s_ts_gps   = 0, s_ts_dtw = 0;

//=================================================================================================
//=================================================================================================


void nmea_task(void* pv) {
    for (;;) {
        NMEA2000.ParseMessages();
        NmeaSendMsg qm;
        while (xQueueReceive(nmea_send_queue, &qm, 0) == pdTRUE)
            NMEA2000.SendMsg(qm.msg);

        // Alle Snapshots zusammen — hier vollständig:
        s_depth        = n_depth;
        s_sog          = n_sog;
        s_stw          = n_stw;
        s_wind_spd      = n_wind_spd;
        s_wind_ang      = n_wind_ang;
        s_heading      = n_heading;
        s_twa          = n_twa;
        s_tws          = n_tws;
        s_rudder       = n_rudder;
        s_water_temp    = n_water_temp;
        s_lat          = n_lat;
        s_lon          = n_lon;
        s_autopilot_mode = n_autopilot_mode;
        s_dtw          = n_dtw;          // ← neu hier drin
        s_ttg          = n_ttg;          // ← neu hier drin

        //TIME Snapshots
        s_ts_depth = n_ts_depth;
        s_ts_sog  = n_ts_sog;
        s_ts_stw   = n_ts_stw;    s_ts_wind = n_ts_wind;
        s_ts_hdg   = n_ts_hdg;    s_ts_wtemp= n_ts_wtemp;
        s_ts_gps   = n_ts_gps;    s_ts_dtw  = n_ts_dtw;

        memcpy(s_wpt_name, (const char*)n_wpt_name, sizeof(s_wpt_name)); // ← neu hier drin

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

//=================================================================================================
//=================================================================================================
//=================================================================================================


void backlight_set(uint8_t duty) {
    ledcWrite(BL_PIN, duty);
}

// ═══════════════════════════════════════════════════════════════════════════════
// Vorwärtsdeklarationen
// ═══════════════════════════════════════════════════════════════════════════════
// Vorwärtsdeklarationen
void update_displays();
void navigate_to(int idx);
static void navigate_relative(int delta);
void fmt_pos(lv_obj_t* lbl, double deg, bool is_lat);
static void ap_send_key(uint16_t command);
static void ap_adjust_course(float delta);
static void fmt_deg(lv_obj_t* lbl, float val);
static void fmt_val_cached(lv_obj_t* lbl, float val, float& last, const char* fmt = "%.1f", float maxval = 99.9f);
lv_obj_t* build_multi_screen();

// ═══════════════════════════════════════════════════════════════════════════════
// Zeiger-Update Funktion (Canvas-basiert)
// ═══════════════════════════════════════════════════════════════════════════════
#define CANVAS_SIZE 80

static void build_wind_needle_sprite() {
    if (!wind_canvas || !canvas_buf) return;
    lv_canvas_fill_bg(wind_canvas, lv_color_hex(0x000000), LV_OPA_TRANSP);
    lv_point_t pts[3] = {
        {CANVAS_SIZE/2, 5},
        {8, CANVAS_SIZE - 5},
        {CANVAS_SIZE - 8, CANVAS_SIZE - 5}
    };
    lv_draw_rect_dsc_t rect_dsc;
    lv_draw_rect_dsc_init(&rect_dsc);
    rect_dsc.bg_color = lv_color_hex(0xFF3300);
    rect_dsc.bg_opa = LV_OPA_COVER;
    lv_canvas_draw_polygon(wind_canvas, pts, 3, &rect_dsc);
    lv_img_set_pivot(wind_canvas, CANVAS_SIZE / 2, CANVAS_SIZE / 2);
}

void place_wind_needle(float awa_deg) {
    if (!wind_canvas) return;
    float rad = (270.0f + awa_deg) * (float)M_PI / 180.0f;
    int needle_center_r = (R_SCALE + R_GREY) / 2;
    int cx = WIND_CX + (int)(needle_center_r * cosf(rad));
    int cy = (WIND_CY - 10) + (int)(needle_center_r * sinf(rad));
    lv_obj_set_pos(wind_canvas, cx - CANVAS_SIZE/2, cy - CANVAS_SIZE/2);
    lv_img_set_angle(wind_canvas, (int16_t)(awa_deg * 10));
}

void wind_needle_anim_cb(lv_timer_t *t) {
    if (lv_scr_act() != screens[4]) return;  // ← NEU: nur wenn Wind-Screen aktiv
    float target;
    if (wind_mode == MODE_AWS_AWA) {
        if (s_wind_ang < 0.0f) return;   // AWA ist immer 0–360, negativ = ungültig
        target = s_wind_ang;
    } else {
        if (s_twa < -360.0f) return;     // n_twa = -999.0 als Sentinel für "kein Wert"
        target = s_twa;                  // TWA ist -180…+180, darf negativ sein!
    }

    float diff = target - current_awa_damped;
    while (diff < -180.0f) diff += 360.0f;
    while (diff >  180.0f) diff -= 360.0f;
    current_awa_damped += diff / DAMPING_FACTOR;
    while (current_awa_damped >= 360.0f) current_awa_damped -= 360.0f;
    while (current_awa_damped <    0.0f) current_awa_damped += 360.0f;
        // Mindestschwelle: Nadel nur neu zeichnen wenn Änderung > 1°
    static float last_drawn_awa = -9999.f;
    if (fabsf(current_awa_damped - last_drawn_awa) < 1.0f) return;  // ← NEU
    last_drawn_awa = current_awa_damped;                              // ← NEU
    place_wind_needle(current_awa_damped);


        // ── NEU: Windgeschwindigkeits-Dämpfung ─────────────────
    float spd_target = (wind_mode == MODE_TWS_TWA) ? s_tws : s_wind_spd;
    if (spd_target >= 0.0f) {
        current_wspd_damped += (spd_target - current_wspd_damped) / WSPD_DAMPING_FACTOR;
    }
    // (bei ungültigem Signal wird current_wspd_damped nicht verändert)
}
// ═══════════════════════════════════════════════════════════════════════════════
// True Wind nicht im NEtzwerk, wird selbst berechnet
// ═══════════════════════════════════════════════════════════════════════════════
void compute_true_wind() {
    float boat_speed = (n_stw > 0) ? n_stw : n_sog;

    if (n_wind_spd < 0 || boat_speed < 0) {
        n_tws = -1.0f;
        n_twa = -999.0f;
        return;
    }

    float awa = n_wind_ang;

    // Normalize [-180,180]
    if (awa > 180.0f) awa -= 360.0f;

    float awa_rad = awa * DEG_TO_RAD;

    float aws = n_wind_spd;

    n_tws = sqrtf(
        aws * aws + boat_speed * boat_speed -
        2.0f * aws * boat_speed * cosf(awa_rad)
    );

    float twa_rad = atan2f(
        aws * sinf(awa_rad),
        aws * cosf(awa_rad) - boat_speed
    );

    n_twa = twa_rad * RAD_TO_DEG;
}


// Hilfsfunktion — gibt statischen Buffer zurück (nur im LVGL-Task aufrufen!)
static const char* shorten_wpt_name(const char* name) {
    static char buf[13];  // war [12] — 9 Zeichen + 3 Bytes UTF-8 "…" + '\0' = 13
    // Bekannte Präfixe entfernen
    const char* prefixes[] = {"Waypoint", "waypoint", "WPT", "wpt", "GOTO_", "goto_", nullptr};
    const char* s = name;
    for (int i = 0; prefixes[i]; i++) {
        size_t plen = strlen(prefixes[i]);
        if (strncmp(name, prefixes[i], plen) == 0 && strlen(name) > plen) {
            s = name + plen;
            break;
        }
    }
    // Unterstriche → Leerzeichen, max 9 Zeichen
    int i = 0;
    for (; s[i] && i < 9; i++) {
        buf[i] = (s[i] == '_') ? ' ' : s[i];
    }
    if (s[i] != '\0') { buf[i++] = 0xE2; buf[i++] = 0x80; buf[i++] = 0xA6; } // UTF-8 "…"
    buf[i] = '\0';
    return buf;
}

// ═══════════════════════════════════════════════════════════════════════════════
// Color fuer veraltete NMEA Daten
// ═══════════════════════════════════════════════════════════════════════════════
// Nur im LVGL-Task aufrufen — gibt Farbe je nach Datealter zurück
static lv_color_t data_color(uint32_t ts_snap,
                              uint32_t fresh_ms = 5000,
                              uint32_t warn_ms  = 15000) {
    if (ts_snap == 0) return lv_color_hex(0x444444);         // nie empfangen: dunkelgrau
    uint32_t age = millis() - ts_snap;
    if (age < fresh_ms)  return lv_color_hex(0x00FFCC);      // frisch: teal (bestehende Farbe)
    if (age < warn_ms)   return lv_color_hex(0xBB8800);      // alt: gedämpftes Gelb
    return lv_color_hex(0x555555);                            // sehr alt: grau
}

// Farbe für Windgeschwindigkeit (int + dec Label)
// Fresh: Apparent=dunkel, True=blau  |  Alt: gedämpftes Gelb  |  Kein Signal: grau
static lv_color_t wind_speed_color(uint32_t ts_snap) {
    if (ts_snap == 0) return lv_color_hex(0x444444);
    uint32_t age = millis() - ts_snap;
    if (age > 15000) return lv_color_hex(0x555555);
    if (age >  5000) return lv_color_hex(0xBB8800);
    // frisch → Modus-Farbe
    return (wind_mode == MODE_TWS_TWA)
        ? lv_color_hex(0x0066CC)   // True  = Blau
        : lv_color_hex(0x111111);  // App   = fast schwarz
}

// Farbe für Windwinkel-Label (weißer Hintergrundbereich vs dunkle Box)
static lv_color_t wind_angle_color(uint32_t ts_snap) {
    if (ts_snap == 0) return lv_color_hex(0x444444);
    uint32_t age = millis() - ts_snap;
    if (age > 15000) return lv_color_hex(0x555555);
    if (age >  5000) return lv_color_hex(0xBB8800);
    return (wind_mode == MODE_TWS_TWA)
        ? lv_color_hex(0x0088FF)   // True  = Hellblau
        : lv_color_hex(0xFFFFFF);  // App   = weiß
}
// ═══════════════════════════════════════════════════════════════════════════════
// NMEA2000 Handler
// ═══════════════════════════════════════════════════════════════════════════════
void OnN2kMessage(const tN2kMsg &N2kMsg) {
    #if NMEA_DEBUG
        Serial.printf("PGN: %6lu Src: %3d\n", N2kMsg.PGN, N2kMsg.Source);
    #endif


    last_nmea_msg_ms = millis();  // ← NEU
        switch (N2kMsg.PGN) {
            case 129026: {
                unsigned char SID;
                tN2kHeadingReference ref;
                double COG, SOG;

                if (ParseN2kCOGSOGRapid(N2kMsg, SID, ref, COG, SOG)) {
                    if (!N2kIsNA(SOG)) {
                        n_sog = (float)(SOG * 1.94384f);   // m/s → Knoten
                        n_ts_sog   = millis();
                    }
                }
                break;
            }
            case 128259: {
                unsigned char SID;
                double STW, SOG_water;
                tN2kSpeedWaterReferenceType refType;

                if (ParseN2kBoatSpeed(N2kMsg, SID, STW, SOG_water, refType)) {
                    if (!N2kIsNA(STW)) {
                        n_stw = (float)(STW * 1.94384f);   // m/s → Knoten
                        n_ts_stw   = millis();
                    }
                }
                break;
            }

        case 130306: {
            unsigned char SID; double ws, wa;
            tN2kWindReference ref;
            if (ParseN2kWindSpeed(N2kMsg, SID, ws, wa, ref)) {
                if (!N2kIsNA(ws)) {
                    if (ref == N2kWind_Apparent) {
                        n_wind_spd = (float)(ws * 1.94384f);
                        n_wind_ang = (float)(wa * 180.0 / M_PI);
                        n_ts_wind = millis();
                    } else if (ref == 2) {   // 2 = N2kWind_True
                        n_tws = (float)(ws * 1.94384f);
                        n_twa = (float)(wa * 180.0 / M_PI);
                        n_ts_wind = millis();
                    }
                }
                // 👉 NEU:
                compute_true_wind();
            }
            break;
        }
    case 128267: {
        unsigned char SID; double depth, offset;
        if (ParseN2kWaterDepth(N2kMsg, SID, depth, offset)) {
            if (!N2kIsNA(depth)) {
                n_depth = (float)(depth + (N2kIsNA(offset) ? 0.0 : offset));
                n_ts_depth = millis();
            } else {
                n_depth = -1.0f;
            }
        }
        break;
    }
    case 127250: {
        unsigned char SID;
        double heading, deviation, variation;
        tN2kHeadingReference ref;

        if (ParseN2kHeading(N2kMsg, SID, heading, deviation, variation, ref)) {
            if (!N2kIsNA(heading)) {
                n_heading = heading * RAD_TO_DEG;
                n_ts_hdg = millis();
            }
        }
        break;
    }
        case 130311: {
            unsigned char SID; tN2kTempSource ts; double temp, st;
            tN2kHumiditySource hs; double hum, pres;
            if (ParseN2kEnvironmentalParameters(N2kMsg, SID, ts, temp, hs, hum, pres))
                if (ts == N2kts_SeaTemperature && !N2kIsNA(temp))
                    n_water_temp = (float)(temp - 273.15f);
                    n_ts_wtemp = millis();
            break;
        }
        case 127245: {
            unsigned char SID; tN2kRudderDirectionOrder ord; double rud, ao;
            if (ParseN2kRudder(N2kMsg, rud, SID, ord, ao))
                if (!N2kIsNA(rud)) n_rudder = (float)(rud * 180.0 / M_PI);
            break;
        }
        case 129029: {
            unsigned char SID, nRefStations;
            uint16_t daysSince1970, refStationID;
            double secSinceMidnight, lat, lon, alt, hdop, pdop, geoidalSep, refStationCorr;
            tN2kGNSStype gnssType, refType;
            tN2kGNSSmethod method;
            if (ParseN2kGNSS(N2kMsg, SID, daysSince1970, secSinceMidnight,
                            lat, lon, alt, gnssType, method,
                            nRefStations, hdop, pdop, geoidalSep,
                            nRefStations, refType, refStationID, refStationCorr)) {
                if (!N2kIsNA(lat)) n_lat = lat;
                if (!N2kIsNA(lon)) n_lon = lon;
                if (!N2kIsNA(alt)) n_gps_alt = (float)alt;
                n_ts_gps = millis();
            }
            break;
        }
        case 65379L: { //RAYMRINE AUTOPILOT
        if (N2kMsg.DataLen >= 8) {
            // ← NEU: Quelladresse des Autopiloten merken
            g_pilot_addr = N2kMsg.Source;

            int idx = 0;
            uint8_t b0 = N2kMsg.GetByte(idx);
            uint8_t b1 = N2kMsg.GetByte(idx);
            uint8_t b2 = N2kMsg.GetByte(idx);
            uint8_t b3 = N2kMsg.GetByte(idx);
            uint16_t mode = (b3 << 8) | b2;
            if (!sim_mode_active) n_autopilot_mode = mode;
        #if AUTOPILOT_DEBUG
            // Temporär zum Debuggen – zeigt unbekannte Werte im Serial Monitor
            if (mode != 0x0000 && mode != 0x0040 &&
                mode != 0x0100 && mode != 0x0180 && mode != 0x0181) {
                Serial.printf("AP unbekannter Mode: 0x%04X (b2=0x%02X b3=0x%02X)\n",
                            mode, b2, b3);
            }
            #endif
        }
            break;
        }
        case 129284L: {
            unsigned char SID;
            tN2kHeadingReference bearing_od;
            double dtw, bearing_pd, ttg;
            tN2kDistanceCalculationType calcType;
            bool arrivalCircle, perpCrossed;
            double eta;
            int16_t eta_date;    // int16_t, nicht double
            uint32_t origWpt, destWpt;  // uint32_t, nicht uint8_t
            double destLat, destLon, wcv;
            if (ParseN2kNavigationInfo(N2kMsg, SID, dtw, bearing_od, perpCrossed,
                arrivalCircle, calcType, ttg, eta_date, eta,
                bearing_pd, origWpt, destWpt, destLat, destLon, wcv)) {
                if (!N2kIsNA(dtw)) n_dtw = (float)(dtw / 1852.0);   // m → NM
                if (!N2kIsNA(ttg)) n_ttg = (float)(ttg / 60.0);     // s → Minuten
                n_ts_dtw = millis();
            }
            break;
        }
        case 129285L: {
            // Manuell: Bytes 6+ sind der Name als nullterminierter String
            // Die NMEA2000-Lib hat keinen fertigen Parser dafür
            if (N2kMsg.DataLen > 6) {
                int nameStart = 6;
                int len = 0;
                while (nameStart + len < N2kMsg.DataLen && N2kMsg.Data[nameStart + len] != 0 && len < 19) {
                    n_wpt_name[len] = (char)N2kMsg.Data[nameStart + len];
                    len++;
                }
                n_wpt_name[len] = '\0';
            }
            break;
        }

        }
}


void nmea_init() {
    NMEA2000.SetN2kCANSendFrameBufSize(100);
    NMEA2000.SetN2kCANReceiveFrameBufSize(250);

    // NEU: ESP32 als echten NMEA2000-Node anmelden (nötig zum Senden!)
    NMEA2000.SetProductInformation("00000001", 100,
        "Marine MFD", "1.0", "1.0");
    NMEA2000.SetDeviceInformation(1, 120, 25, 2046);

    NMEA2000.SetMsgHandler(OnN2kMessage);

    // ALT: N2km_ListenOnly  →  NEU: N2km_ListenAndNode
    // NEU — sendet nur was wir explizit senden, kein Heartbeat:
    NMEA2000.SetMode(tNMEA2000::N2km_ListenAndSend, 22);
    
    const unsigned long rxPGNs[] = {
        128267L, 128259L, 129026L, 130306L,
        65379L, 65359L, 65360L,   // Pilot Mode + Heading
        //65345L,   // ← NEU: Pilot Wind Datum (locked wind angle)
        129284L,   // Navigation Data (DTW, TTG)
        129285L,   // Route/WP Info  (Name)
        127250L, 130311L, 127245L, 129029L, 0
    };
    NMEA2000.ExtendReceiveMessages(rxPGNs);
    NMEA2000.Open();
    Serial.println("NMEA2000 gestartet");
}
// ═══════════════════════════════════════════════════════════════════════════════
// LVGL Callbacks & Init
// ═══════════════════════════════════════════════════════════════════════════════


void lvgl_flush(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *colorp) {
    gfx->draw16bitRGBBitmap(
        area->x1, area->y1,
        (uint16_t*)colorp,
        area->x2 - area->x1 + 1,
        area->y2 - area->y1 + 1
    );
    lv_disp_flush_ready(drv);
}

void lvgl_touch_read(lv_indev_drv_t* drv, lv_indev_data_t* data) {
    touch.read();
    if (touch.isTouched) {
        lv_coord_t tx = 480 - touch.points[0].x;
        lv_coord_t ty = 480 - touch.points[0].y;
        data->point.x = tx;
        data->point.y = ty;
        data->state   = LV_INDEV_STATE_PR;

        if (!swipe_active) {
            // ── TOUCHDOWN: nur einmal beim ersten Kontakt ──
            swipe_start_x  = tx;
            swipe_start_y  = ty;
            swipe_start_ms = millis();
            swipe_active   = true;

            // Slider-Block: nur beim ersten Touch prüfen
            swipe_blocked = false;
            if (slider_brightness && lv_scr_act() == screens[8]) {
                lv_area_t a;
                lv_obj_get_coords(slider_brightness, &a);
                if (tx >= a.x1 - 20 && tx <= a.x2 + 20 &&
                    ty >= a.y1 - 20 && ty <= a.y2 + 20) {
                    swipe_blocked = true;
                }
            }
        }
        // aktuelle Position immer nachführen
        swipe_last_x = tx;
        swipe_last_y = ty;
} else {
    // ── RELEASED ──
    data->state = LV_INDEV_STATE_REL;

    if (swipe_active && !swipe_blocked) {
        int dx = swipe_last_x - swipe_start_x;
        int dy = swipe_last_y - swipe_start_y;
        uint32_t dt = millis() - swipe_start_ms;

        if (abs(dx) >= SWIPE_MIN_DIST
            && abs(dx) > abs(dy) * 2
            && dt <= SWIPE_MAX_MS) {
            // Horizontal: links/rechts durch Screens
            swipe_nav_pending = (dx < 0) ? -2 : -3;

        } else if (abs(dy) >= SWIPE_MIN_DIST      // ← NEU: Vertikaler Swipe
                   && abs(dy) > abs(dx) * 2
                   && dt <= SWIPE_MAX_MS) {
            // dy < 0 = Finger nach OBEN  → Homescreen
            // dy > 0 = Finger nach UNTEN → Autopilot
            swipe_nav_pending = (dy < 0) ? 0 : 9;
        }
    }
    swipe_active = false;
    }
}


void lvgl_init() {
    lv_init();
 /*ALT
    Serial.printf("Freier PSRAM: %d bytes\n", ESP.getFreePsram());

    // Zurück zu 480×100 — kurze SPI-Writes = weniger Tearing
    buf1 = (lv_color_t*)ps_malloc(480 * 100 * sizeof(lv_color_t));
    buf2 = (lv_color_t*)ps_malloc(480 * 100 * sizeof(lv_color_t));
    lv_disp_draw_buf_init(&draw_buf, buf1, buf2, 480 * 100);

*/
// 1. Test ok... HIER ÄNDERN: Puffer zwingend im internen, schnellen SRAM anlegen
 /*   size_t buffer_size = 480 * 100 * sizeof(lv_color_t);
    buf1 = (lv_color_t *)heap_caps_malloc(buffer_size, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    buf2 = (lv_color_t *)heap_caps_malloc(buffer_size, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    
    // Falls das fehlschlägt, weil RAM zu voll, Puffer etwas verkleinern (z.B. 480 * 50)
    if (!buf1 || !buf2) {
        Serial.println("Fehler: Zu wenig internes SRAM für LVGL Puffer!");
    }

    lv_disp_draw_buf_init(&draw_buf, buf1, buf2, 480 * 100);
*/
// 2. Test
        size_t buffer_size = 480 * 100 * sizeof(lv_color_t);  // 96KB pro Buffer
        buf1 = (lv_color_t*)heap_caps_malloc(buffer_size, MALLOC_CAP_DMA);
        buf2 = (lv_color_t*)heap_caps_malloc(buffer_size, MALLOC_CAP_DMA);

        if (!buf1 || !buf2) {
            // Fallback: halbe Größe versuchen
            buffer_size = 480 * 50 * sizeof(lv_color_t);       // 48KB pro Buffer
            if (buf1) { heap_caps_free(buf1); }
            if (buf2) { heap_caps_free(buf2); }
            buf1 = (lv_color_t*)heap_caps_malloc(buffer_size, MALLOC_CAP_DMA);
            buf2 = (lv_color_t*)heap_caps_malloc(buffer_size, MALLOC_CAP_DMA);
            Serial.println("LVGL: Fallback auf 480x50 Buffer");
        }

        // Größe dynamisch übergeben:
        lv_disp_draw_buf_init(&draw_buf, buf1, buf2, buffer_size / sizeof(lv_color_t));


    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res      = 480;
    disp_drv.ver_res      = 480;
    disp_drv.flush_cb     = lvgl_flush;
    disp_drv.draw_buf     = &draw_buf;
    disp_drv.antialiasing = 1;
    //disp_drv.full_refresh = 1;
    lv_disp_drv_register(&disp_drv);

    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type    = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = lvgl_touch_read;
    lv_indev_drv_register(&indev_drv);
}
// ═══════════════════════════════════════════════════════════════════════════════
// Overlay Logik (Animation & Anti-Fragment)
// ═══════════════════════════════════════════════════════════════════════════════

// ═══════════════════════════════════════════════════════════════════════════════
// Navigation
// ═══════════════════════════════════════════════════════════════════════════════
void navigate_to(int idx) {
    if (nav_busy || idx < 0 || idx >= NUM_SCREENS) return;
    if (screens[idx] == lv_scr_act()) return;
    nav_busy = true;
 
    lv_scr_load_anim(screens[idx], LV_SCR_LOAD_ANIM_NONE, 0, 0, false);

    lv_timer_t* t = lv_timer_create([](lv_timer_t* t){
        nav_busy = false;
        lv_timer_del(t);   // ← selbst löschen, kein Memory Leak
        }, 100, NULL);
}

static void navigate_relative(int delta) {
    lv_obj_t* current = lv_scr_act();
    int current_idx = -1;
    for (int i = 0; i < NUM_SCREENS; i++) {
        if (screens[i] == current) { current_idx = i; break; }
    }
    if (current_idx < 0) return;

    // ── ALT: Home (0) war ausgesperrt ──
    // if (current_idx == 0) return;   ← ENTFERNEN

    int new_idx = current_idx + delta;

    // Wrap-around: 0..NUM_SCREENS-1 als vollständiger Ring (Home inklusive)
    if (new_idx < 0)           new_idx = NUM_SCREENS - 1;
    if (new_idx >= NUM_SCREENS) new_idx = 0;

    navigate_to(new_idx);
}
// ═════════════════════════════════════════════════════════════
// Callbacks für Buttons und Slider
// ═════════════════════════════════════════════════════════════
static void brightness_event_cb(lv_event_t *e) {
    lv_obj_t *slider = lv_event_get_target(e);
    int val = lv_slider_get_value(slider);
    uint8_t duty = (uint8_t)(30 + (val - 10) * (255 - 30) / 90);
    backlight_set(duty);
    if (label_brightness) {
        char buf[8];
        snprintf(buf, sizeof(buf), "%d%%", val);
        lv_label_set_text(label_brightness, buf);
    }
}

static void wind_mode_btn_cb(lv_event_t *e) {
    if (wind_mode == MODE_AWS_AWA) wind_mode = MODE_TWS_TWA;
    else wind_mode = MODE_AWS_AWA;

    bool is_true = (wind_mode == MODE_TWS_TWA);

    if (mode_btn_label)
        lv_label_set_text(mode_btn_label, is_true ? "True" : "App");

    lv_obj_t *btn = lv_event_get_target(e);
    lv_obj_set_style_bg_color(btn, is_true ? lv_color_hex(0x000088) : lv_color_hex(0x880000), 0);
    lv_obj_invalidate(btn);  // ← Fix: Streifen verhindern

    if (wind_speed_label)
        lv_label_set_text(wind_speed_label, is_true ? "TWS" : "AWS");

    // ── NEU: Labels umfärben ──────────────────────────────────────
    lv_color_t ang_col  = is_true ? lv_color_hex(0x0088FF) : lv_color_hex(0xFFFFFF);
    lv_color_t spd_col  = is_true ? lv_color_hex(0x0066CC) : lv_color_hex(0x111111);
    lv_color_t spdd_col = is_true ? lv_color_hex(0x0066CC) : lv_color_hex(0x222222);

        if (wind_ang_lbl) { lv_obj_set_style_text_color(wind_ang_lbl, ang_col, 0);  lv_obj_invalidate(wind_ang_lbl); }
        if (aws_int_lbl)  { lv_obj_set_style_text_color(aws_int_lbl,  spd_col, 0);  lv_obj_invalidate(aws_int_lbl); }
        if (aws_dec_lbl)  { lv_obj_set_style_text_color(aws_dec_lbl,  spdd_col, 0); lv_obj_invalidate(aws_dec_lbl); }

    // ─────────────────────────────────────────────────────────────

    update_displays();
}
// ═══════════════════════════════════════════════════════════════════════════════
// AUTOPILOT STEUERUNG – Raymarine EV-1 / Evolution
// PGN 126208 (Mode + Kurs), PGN 126720 (Key-Events)
// Quelle: matztam/raymarine-evo-pilot-remote, AK-Homberger/ESP32-Evo-Remote-Pilot
// ═══════════════════════════════════════════════════════════════════════════════

// Key-Command Codes (EV-1 reverse-engineered)
#define AP_KEY_MINUS_1      0xf801
#define AP_KEY_MINUS_10     0xf802
#define AP_KEY_PLUS_1       0xf805
#define AP_KEY_PLUS_10      0xf806
#define AP_KEY_TACK_PORT    0xf841
#define AP_KEY_TACK_STBD    0xf842

   

static void ap_send_key(uint16_t command) {
    if (!nmea_send_queue || g_pilot_addr < 0) {
        Serial.printf("[AP] ABORTED – pilot_addr=%d\n", g_pilot_addr);
        return;
    }
    Serial.printf("[AP] Sending cmd=0x%04X to addr=%d\n", command, g_pilot_addr);
    // ... rest wie gehabt
    NmeaSendMsg qm;

    qm.msg.SetPGN(126720UL);
    qm.msg.Priority = 7;
    qm.msg.Destination = g_pilot_addr;
    qm.msg.AddByte(0x3b); qm.msg.AddByte(0x9f);
    qm.msg.AddByte(0xf0); qm.msg.AddByte(0x81);
    qm.msg.AddByte(0x86); qm.msg.AddByte(0x21);

    qm.msg.AddByte(command & 0xff);  // 0x01 zuerst
    qm.msg.AddByte(command >> 8);    // dann 0xF8

    qm.msg.AddByte(0xff); qm.msg.AddByte(0xff);
    qm.msg.AddByte(0xff); qm.msg.AddByte(0xff); qm.msg.AddByte(0xff);
    qm.msg.AddByte(0xc1); qm.msg.AddByte(0xc2); qm.msg.AddByte(0xcd);
    qm.msg.AddByte(0x66); qm.msg.AddByte(0x80); qm.msg.AddByte(0xd3);
    qm.msg.AddByte(0x42); qm.msg.AddByte(0xb1); qm.msg.AddByte(0xc8);
    xQueueSend(nmea_send_queue, &qm, 0); // non-blocking!
}



static void ap_set_mode(uint8_t mode_byte, uint8_t sub_byte) {
    if (!nmea_send_queue || g_pilot_addr < 0) return;
    NmeaSendMsg qm;
    qm.msg.SetPGN(126208UL);
    qm.msg.Priority = 3;
    qm.msg.Destination = g_pilot_addr;
    qm.msg.AddByte(0x01); qm.msg.AddByte(0x63);
    qm.msg.AddByte(0xff); qm.msg.AddByte(0x00); qm.msg.AddByte(0xf8);
    qm.msg.AddByte(0x04); qm.msg.AddByte(0x01); qm.msg.AddByte(0x3b);
    qm.msg.AddByte(0x07); qm.msg.AddByte(0x03); qm.msg.AddByte(0x04);
    qm.msg.AddByte(0x04);
    qm.msg.AddByte(mode_byte);
    qm.msg.AddByte(sub_byte);
    qm.msg.AddByte(0x05); qm.msg.AddByte(0xff); qm.msg.AddByte(0xff);
    xQueueSend(nmea_send_queue, &qm, 0); // non-blocking!


 

}

static void ap_set_course(float new_heading_deg) {
    if (!nmea_send_queue || g_pilot_addr < 0) return;
    while (new_heading_deg >= 360.0f) new_heading_deg -= 360.0f;
    while (new_heading_deg < 0.0f) new_heading_deg += 360.0f;
    uint16_t rad10000 = (uint16_t)(new_heading_deg * DEG_TO_RAD * 10000.0f);
    NmeaSendMsg qm;
    qm.msg.SetPGN(126208UL);
    qm.msg.Priority = 3;
    qm.msg.Destination = g_pilot_addr;
    qm.msg.AddByte(0x01); qm.msg.AddByte(0x50);
    qm.msg.AddByte(0xff); qm.msg.AddByte(0x00); qm.msg.AddByte(0xf8);
    qm.msg.AddByte(0x03); qm.msg.AddByte(0x01); qm.msg.AddByte(0x3b);
    qm.msg.AddByte(0x07); qm.msg.AddByte(0x03); qm.msg.AddByte(0x04);
    qm.msg.AddByte(0x06);
    qm.msg.AddByte(rad10000 & 0xff);
    qm.msg.AddByte(rad10000 >> 8);
    xQueueSend(nmea_send_queue, &qm, 0); // non-blocking!
}


// Neue Funktion – ersetzt ap_send_key für Kursänderungen:
static void ap_adjust_course(float delta) {
    if (!nmea_send_queue || g_pilot_addr < 0) return;
    if (ap_target_hdg < 0.0f) {
        if (s_heading < 0.0f) return;  // kein gültiges Heading → kein Befehl senden
        ap_target_hdg = s_heading;
    }
    ap_target_hdg += delta;
    while (ap_target_hdg >= 360.0f) ap_target_hdg -= 360.0f;
    while (ap_target_hdg <   0.0f) ap_target_hdg += 360.0f;
    ap_set_course(ap_target_hdg);  // PGN 126208 – funktioniert bereits!
}

// ── Global: Status-Label für den Pilot-Screen ─────────────────────────────────
static lv_obj_t *pilot_status_lbl = nullptr;
static lv_obj_t *pilot_hdg_lbl    = nullptr;
static lv_obj_t *pilot_statusbox  = nullptr;  // NEU


        // Zentrale Farbtabelle – einmal definiert, überall genutzt
        struct ApModeColor { uint16_t mode; uint32_t col; };
        static const ApModeColor AP_COLORS[] = {
            { 0x0040, 0x007700 },  // AUTO  – Grün
            { 0x0100, 0x0088CC },  // WIND  – Hellblau
            { 0x0180, 0xAA8800 },  // TRCK  – Gelb
            { 0x0181, 0xAA8800 },  // TRCK  – Gelb
            { 0x0000, 0xAA1111 },  // STBY  – Rot (default)
        };
        lv_color_t ap_mode_color(uint16_t mode) {
            for (auto &c : AP_COLORS)
                if (c.mode == mode) return lv_color_hex(c.col);
            return lv_color_hex(0xAA1111);  // Rot als Fallback
        }

lv_obj_t* build_pilot_screen() {
    lv_obj_t *scr = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(scr, lv_color_hex(0x080810), 0);
    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);
    

    // Titel
    lv_obj_t *title = lv_label_create(scr);
    lv_label_set_text(title, "AUTOPILOT");
    lv_obj_set_style_text_font(title, &montserrat_48, 0);
    lv_obj_set_style_text_color(title, lv_color_hex(0x0080FF), 0);
    lv_obj_align(title, LV_ALIGN_TOP_LEFT, 12, 10);

   

    // statusbox anlegen – Pointer global speichern:
    pilot_statusbox = lv_obj_create(scr);
    lv_obj_set_size(pilot_statusbox, 460, 215);
    lv_obj_set_pos(pilot_statusbox, 10, 66);
    lv_obj_set_style_bg_color(pilot_statusbox, lv_color_hex(0xAA1111), 0); // Start: STBY=Rot
    lv_obj_set_style_bg_opa(pilot_statusbox, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(pilot_statusbox, lv_color_hex(0xFF4444), 0);
    lv_obj_set_style_border_width(pilot_statusbox, 3, 0);
    lv_obj_set_style_radius(pilot_statusbox, 14, 0);
    lv_obj_clear_flag(pilot_statusbox, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE);

    pilot_status_lbl = lv_label_create(pilot_statusbox);
    lv_label_set_text(pilot_status_lbl, "STBY");
    lv_obj_set_style_text_font(pilot_status_lbl, &montserrat_72, 0);
    lv_obj_set_style_text_color(pilot_status_lbl, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align(pilot_status_lbl, LV_ALIGN_TOP_MID, 0, 10);

    pilot_hdg_lbl = lv_label_create(pilot_statusbox);
    lv_label_set_text(pilot_hdg_lbl, "---");
    lv_obj_set_style_text_font(pilot_hdg_lbl, &montserrat_120, 0);
    lv_obj_set_style_text_color(pilot_hdg_lbl, lv_color_hex(0x00FFCC), 0);
    lv_obj_align(pilot_hdg_lbl, LV_ALIGN_TOP_MID, 0, 85);

            // Waypoint-Name (TRCK-Modus), zunächst versteckt
        pilot_wpt_lbl = lv_label_create(pilot_statusbox);
        lv_label_set_text(pilot_wpt_lbl, "---");
        lv_obj_set_style_text_font(pilot_wpt_lbl, &montserrat_72, 0);
        lv_obj_set_style_text_color(pilot_wpt_lbl, lv_color_hex(0x1A0800), 0); // dunkel auf gelbem BG
        lv_obj_set_width(pilot_wpt_lbl, 440);
        lv_obj_set_style_text_align(pilot_wpt_lbl, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_align(pilot_wpt_lbl, LV_ALIGN_TOP_MID, 0, 75);
        lv_obj_add_flag(pilot_wpt_lbl, LV_OBJ_FLAG_HIDDEN);

        // DTW + TTG Info-Zeile
        pilot_wpt_info = lv_label_create(pilot_statusbox);
        lv_label_set_text(pilot_wpt_info, "");
        lv_obj_set_style_text_font(pilot_wpt_info, &montserrat_36, 0); // größer: 24 -> 36
        lv_obj_set_style_text_color(pilot_wpt_info, lv_color_hex(0x1A0800), 0); // dunkel auf gelbem BG
        lv_obj_set_width(pilot_wpt_info, 440);
        lv_obj_set_style_text_align(pilot_wpt_info, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_align(pilot_wpt_info, LV_ALIGN_TOP_MID, 0, 158);
        lv_obj_add_flag(pilot_wpt_info, LV_OBJ_FLAG_HIDDEN);

    struct { const char *lbl; uint8_t m; uint8_t s; uint32_t col; } modes[] = {
        { "STBY", 0x00, 0x00, 0xAA1111 },  // Rot
        { "AUTO", 0x40, 0x00, 0x007700 },  // Grün
        { "WND",  0x00, 0x01, 0x0088CC },  // Hellblau
        { "TRCK", 0x80, 0x01, 0xAA8800 },  // Gelb
    };

    int bx = 10, by = 290, bw = 108, bh = 55, gap = 8;
    for (int i = 0; i < 4; i++) {
        lv_obj_t *btn = lv_btn_create(scr);
        lv_obj_set_size(btn, bw, bh);
        lv_obj_set_pos(btn, bx + i * (bw + gap), by);
        lv_obj_set_style_bg_color(btn, lv_color_hex(modes[i].col), 0);
        lv_obj_set_style_border_width(btn, 0, 0);
        lv_obj_set_style_radius(btn, 10, 0);
        lv_obj_t *l = lv_label_create(btn);
        lv_label_set_text(l, modes[i].lbl);
        lv_obj_set_style_text_font(l, &montserrat_36, 0);
        lv_obj_center(l);
        // user_data: mode-Byte (8 bit) | sub-Byte (8 bit) gepackt
        uint32_t packed = ((uint32_t)modes[i].m << 8) | modes[i].s;
        lv_obj_set_user_data(btn, (void*)(uintptr_t)packed);


        lv_obj_add_event_cb(btn, [](lv_event_t *e) {
            uint32_t p = (uint32_t)(uintptr_t)lv_obj_get_user_data(lv_event_get_target(e));
            uint8_t m = (p >> 8) & 0xff;
            uint8_t s = p & 0xff;

        if (m == 0x40 && s == 0x00) {
        if (ap_target_hdg < 0.0f) ap_target_hdg = s_heading;
        } else if (m == 0x00 && s == 0x01) {  // WND
            if (ap_target_wind <= -999.0f) ap_target_wind = s_wind_ang; // aktuellen AWA übernehmen
        } else if (m == 0x00 && s == 0x00) {  // STBY
            ap_target_hdg = -1.0f;
            ap_target_wind = -999.0f;
        }
        ap_set_mode(m, s);
        }, LV_EVENT_CLICKED, NULL);

    }

    // ── Kurs-Buttons  ─────────────────────────────────────────────────────────
  
    // Kurs-Buttons
    struct { const char *lbl; int delta10; } keys[] = {
        { "-10", -100 },
        { " -1",  -10 },
        { " +1",  +10 },
        { "+10", +100 },
    };
    by = 365;
    for (int i = 0; i < 4; i++) {
        lv_obj_t *btn = lv_btn_create(scr);
        lv_obj_set_size(btn, bw, bh);
        lv_obj_set_pos(btn, bx + i * (bw + gap), by);
        lv_obj_set_style_bg_color(btn, lv_color_hex(0x1A3A5C), 0);
        lv_obj_set_style_border_width(btn, 0, 0);
        lv_obj_set_style_radius(btn, 10, 0);
        lv_obj_t *l = lv_label_create(btn);
        lv_label_set_text(l, keys[i].lbl);
        lv_obj_set_style_text_font(l, &montserrat_48, 0);
        lv_obj_center(l);
        lv_obj_set_user_data(btn, (void*)(intptr_t)keys[i].delta10);
        lv_obj_add_event_cb(btn, [](lv_event_t *e) {       // ← INNERHALB der for-Schleife!
            lv_obj_t *b = lv_event_get_target(e);
            int delta10 = (int)(intptr_t)lv_obj_get_user_data(b);
        if (s_autopilot_mode == 0x0100) {  // WIND mode
            if (ap_target_wind <= -999.0f) ap_target_wind = s_wind_ang;  // Fallback
            float step = (float)delta10 / 10.0f;
            ap_target_wind += step;
            while (ap_target_wind >  180.0f) ap_target_wind -= 360.0f;
            while (ap_target_wind < -180.0f) ap_target_wind += 360.0f;

            // cmd HIER berechnen, bevor ap_send_key aufgerufen wird:
        uint16_t cmd = (delta10 < 0)
            ? (abs(delta10) >= 100 ? AP_KEY_MINUS_10 : AP_KEY_MINUS_1)
            : (abs(delta10) >= 100 ? AP_KEY_PLUS_10  : AP_KEY_PLUS_1);
            ap_send_key(cmd);
        }

       }, LV_EVENT_CLICKED, NULL);
    }
 
    // ── Tack-Buttons ──────────────────────────────────────────────────────────
    by = 430;
    lv_obj_t *tp = lv_btn_create(scr);
    lv_obj_set_size(tp, 225, 45);
    lv_obj_set_pos(tp, 10, by);
    lv_obj_set_style_bg_color(tp, lv_color_hex(0x5C1A1A), 0);
    lv_obj_set_style_radius(tp, 8, 0);
    lv_obj_set_style_border_width(tp, 0, 0);
    lv_obj_t *tpl = lv_label_create(tp);
    lv_label_set_text(tpl, "TACK PORT");
    lv_obj_set_style_text_font(tpl, &montserrat_36, 0);
    lv_obj_center(tpl);
    lv_obj_t *ts = lv_btn_create(scr);

        // RICHTIG – nach lv_obj_t *ts = lv_btn_create(scr); und lv_obj_center(tsl):
        lv_obj_add_event_cb(tp, [](lv_event_t *e){ ap_send_key(AP_KEY_TACK_PORT); }, LV_EVENT_CLICKED, NULL);
        lv_obj_add_event_cb(ts, [](lv_event_t *e){ ap_send_key(AP_KEY_TACK_STBD); }, LV_EVENT_CLICKED, NULL);

    lv_obj_set_size(ts, 225, 45);
    lv_obj_set_pos(ts, 245, by);
    lv_obj_set_style_bg_color(ts, lv_color_hex(0x1A3A1A), 0);
    lv_obj_set_style_radius(ts, 8, 0);
    lv_obj_set_style_border_width(ts, 0, 0);
    lv_obj_t *tsl = lv_label_create(ts);
    lv_label_set_text(tsl, "TACK STBD");
    lv_obj_set_style_text_font(tsl, &montserrat_36, 0);
    lv_obj_center(tsl);

    return scr;
}
// ═══════════════════════════════════════════════════════════════════════════════
// Home-Screen
// ═══════════════════════════════════════════════════════════════════════════════
static int depth_target = 1;
static int sog_target = 2;
static int stw_target = 3;
static int wind_target = 4;
static int hdg_analog_target = 5;
static int hdg_digital_target = 6;
static int pos_target = 7;
static int setup_target = 8;
static int autopilot_target = 9;

lv_obj_t* build_home_screen() {
    lv_obj_t *scr = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(scr, lv_color_hex(0x080810), 0);
    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);
    

    lv_obj_t *title = lv_label_create(scr);
    lv_label_set_text(title, "Marine MFD");
    lv_obj_set_style_text_font(title, &montserrat_48, 0);
    lv_obj_set_style_text_color(title, lv_color_hex(0x0080FF), 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 14);

    // DEPTH — jetzt halbe Breite
    lv_obj_t* depth_btn = lv_btn_create(scr);
    lv_obj_set_size(depth_btn, 100, 100);          // ← war 212
    lv_obj_set_pos(depth_btn, 14, 80);
    lv_obj_set_style_bg_color(depth_btn, lv_color_hex(0x007ACC), 0);
    lv_obj_set_style_radius(depth_btn, 14, 0);
    lv_obj_set_style_border_width(depth_btn, 0, 0);
    lv_obj_t* depth_lbl = lv_label_create(depth_btn);
    lv_label_set_text(depth_lbl, "DEPTH");
    lv_obj_set_style_text_font(depth_lbl, &lv_font_montserrat_24, 0);
    lv_obj_center(depth_lbl);
    lv_obj_set_user_data(depth_btn, (void*)depth_target);
    lv_obj_add_event_cb(depth_btn, [](lv_event_t* e){
        int idx = (int)lv_obj_get_user_data(lv_event_get_target(e));
        navigate_to(idx);
    }, LV_EVENT_CLICKED, NULL);

    // MULTI — neuer Button, rechts daneben
    lv_obj_t* multi_btn = lv_btn_create(scr);
    lv_obj_set_size(multi_btn, 100, 100);
    lv_obj_set_pos(multi_btn, 126, 80);           // 14+100+12=126
    lv_obj_set_style_bg_color(multi_btn, lv_color_hex(0x006655), 0);  // Dunkelgrün
    lv_obj_set_style_radius(multi_btn, 14, 0);
    lv_obj_set_style_border_width(multi_btn, 0, 0);
    lv_obj_t* multi_lbl_btn = lv_label_create(multi_btn);
    lv_label_set_text(multi_lbl_btn, "MULTI");
    lv_obj_set_style_text_font(multi_lbl_btn, &lv_font_montserrat_24, 0);
    lv_obj_center(multi_lbl_btn);
    lv_obj_set_user_data(multi_btn, (void*)multi_target);
    lv_obj_add_event_cb(multi_btn, [](lv_event_t* e){
        int idx = (int)lv_obj_get_user_data(lv_event_get_target(e));
        navigate_to(idx);
    }, LV_EVENT_CLICKED, NULL);

    int x_start = 254;
    int btn_w = 100;
    int spacing = 12;

    lv_obj_t *sog_btn = lv_btn_create(scr);
    lv_obj_set_size(sog_btn, btn_w, 100);
    lv_obj_set_pos(sog_btn, x_start, 80);
    lv_obj_set_style_bg_color(sog_btn, lv_color_hex(0x007ACC), 0);
    lv_obj_set_style_radius(sog_btn, 14, 0);
    lv_obj_set_style_border_width(sog_btn, 0, 0);
    lv_obj_t *sog_lbl = lv_label_create(sog_btn);
    lv_label_set_text(sog_lbl, "SOG");
    lv_obj_set_style_text_font(sog_lbl, &lv_font_montserrat_24, 0);
    lv_obj_center(sog_lbl);
    lv_obj_set_user_data(sog_btn, (void *)&sog_target);
    lv_obj_add_event_cb(sog_btn, [](lv_event_t *e) {
        int idx = *(int *)lv_obj_get_user_data(lv_event_get_target(e));
        navigate_to(idx);
    }, LV_EVENT_CLICKED, NULL);

    lv_obj_t *stw_btn = lv_btn_create(scr);
    lv_obj_set_size(stw_btn, btn_w, 100);
    lv_obj_set_pos(stw_btn, x_start + btn_w + spacing, 80);
    lv_obj_set_style_bg_color(stw_btn, lv_color_hex(0x007ACC), 0);
    lv_obj_set_style_radius(stw_btn, 14, 0);
    lv_obj_set_style_border_width(stw_btn, 0, 0);
    lv_obj_t *stw_lbl = lv_label_create(stw_btn);
    lv_label_set_text(stw_lbl, "STW");
    lv_obj_set_style_text_font(stw_lbl, &lv_font_montserrat_24, 0);
    lv_obj_center(stw_lbl);
    lv_obj_set_user_data(stw_btn, (void *)&stw_target);
    lv_obj_add_event_cb(stw_btn, [](lv_event_t *e) {
        int idx = *(int *)lv_obj_get_user_data(lv_event_get_target(e));
        navigate_to(idx);
    }, LV_EVENT_CLICKED, NULL);

// LINKS: WIND
lv_obj_t *wind_btn = lv_btn_create(scr);
lv_obj_set_size(wind_btn, 100, 100);
lv_obj_set_pos(wind_btn, 14, 200);
lv_obj_set_style_bg_color(wind_btn, lv_color_hex(0x00AA55), 0);
lv_obj_set_style_radius(wind_btn, 14, 0);
lv_obj_set_style_border_width(wind_btn, 0, 0);
lv_obj_t *wind_lbl = lv_label_create(wind_btn);
lv_label_set_text(wind_lbl, "WIND");
lv_obj_set_style_text_font(wind_lbl, &lv_font_montserrat_24, 0);
lv_obj_center(wind_lbl);
lv_obj_set_user_data(wind_btn, (void *)&wind_target);
lv_obj_add_event_cb(wind_btn, [](lv_event_t *e) {
    int idx = *(int *)lv_obj_get_user_data(lv_event_get_target(e));
    navigate_to(idx);
}, LV_EVENT_CLICKED, NULL);

// MITTE: POS
lv_obj_t *pos_btn = lv_btn_create(scr);
lv_obj_set_size(pos_btn, 100, 100);
lv_obj_set_pos(pos_btn, 126, 200);
lv_obj_set_style_bg_color(pos_btn, lv_color_hex(0x880055), 0);
lv_obj_set_style_radius(pos_btn, 14, 0);
lv_obj_set_style_border_width(pos_btn, 0, 0);
lv_obj_t *pos_lbl = lv_label_create(pos_btn);
lv_label_set_text(pos_lbl, "POS");
lv_obj_set_style_text_font(pos_lbl, &lv_font_montserrat_24, 0);
lv_obj_center(pos_lbl);
lv_obj_set_user_data(pos_btn, (void *)&pos_target);
lv_obj_add_event_cb(pos_btn, [](lv_event_t *e) {
    int idx = *(int *)lv_obj_get_user_data(lv_event_get_target(e));
    navigate_to(idx);
}, LV_EVENT_CLICKED, NULL);

// UNTEN LINKS: AUTOPILOT (unter WIND + POS)
lv_obj_t *ap_btn = lv_btn_create(scr);
lv_obj_set_size(ap_btn, 212, 100);
lv_obj_set_pos(ap_btn, 14, 320);           // ← war (366, 200) → jetzt links unten
lv_obj_set_style_bg_color(ap_btn, lv_color_hex(0xB00000), 0);
lv_obj_set_style_bg_opa(ap_btn, LV_OPA_COVER, 0);
lv_obj_set_style_border_width(ap_btn, 0, 0);
lv_obj_set_style_radius(ap_btn, 14, 0);
lv_obj_t *ap_lbl = lv_label_create(ap_btn);
lv_label_set_text(ap_lbl, "AUTOPILOT");
lv_obj_set_style_text_font(ap_lbl, &lv_font_montserrat_24, 0);
lv_obj_set_style_text_color(ap_lbl, lv_color_hex(0xFFFFFF), 0);
lv_obj_center(ap_lbl);
lv_obj_set_user_data(ap_btn, (void *)&autopilot_target);
lv_obj_add_event_cb(ap_btn, [](lv_event_t *e) {
    int idx = *(int *)lv_obj_get_user_data(lv_event_get_target(e));
    navigate_to(idx);
}, LV_EVENT_CLICKED, NULL);


     // HDG A - Analoger Heading Screen
    lv_obj_t *hdg_a_btn = lv_btn_create(scr);
    lv_obj_set_size(hdg_a_btn, 100, 100);
    lv_obj_set_pos(hdg_a_btn, 254, 200);
    lv_obj_set_style_bg_color(hdg_a_btn, lv_color_hex(0x886600), 0);
    lv_obj_set_style_radius(hdg_a_btn, 14, 0);
    lv_obj_set_style_border_width(hdg_a_btn, 0, 0);
    lv_obj_t *hdg_a_lbl = lv_label_create(hdg_a_btn);
    lv_label_set_text(hdg_a_lbl, "HDG A");
    lv_obj_set_style_text_font(hdg_a_lbl, &lv_font_montserrat_24, 0);
    lv_obj_center(hdg_a_lbl);
    lv_obj_set_user_data(hdg_a_btn, (void *)&hdg_analog_target);
    lv_obj_add_event_cb(hdg_a_btn, [](lv_event_t *e) {
        int idx = *(int *)lv_obj_get_user_data(lv_event_get_target(e));
        navigate_to(idx);
    }, LV_EVENT_CLICKED, NULL);

    // HDG D - Digitaler Heading Screen
    lv_obj_t *hdg_d_btn = lv_btn_create(scr);
    lv_obj_set_size(hdg_d_btn, 100, 100);
    lv_obj_set_pos(hdg_d_btn, 366, 200);
    lv_obj_set_style_bg_color(hdg_d_btn, lv_color_hex(0x886600), 0);
    lv_obj_set_style_radius(hdg_d_btn, 14, 0);
    lv_obj_set_style_border_width(hdg_d_btn, 0, 0);
    lv_obj_t *hdg_d_lbl = lv_label_create(hdg_d_btn);
    lv_label_set_text(hdg_d_lbl, "HDG D");
    lv_obj_set_style_text_font(hdg_d_lbl, &lv_font_montserrat_24, 0);
    lv_obj_center(hdg_d_lbl);
    lv_obj_set_user_data(hdg_d_btn, (void *)&hdg_digital_target);
    lv_obj_add_event_cb(hdg_d_btn, [](lv_event_t *e) {
        int idx = *(int *)lv_obj_get_user_data(lv_event_get_target(e));
        navigate_to(idx);
    }, LV_EVENT_CLICKED, NULL);


 

    lv_obj_t *setup_btn = lv_btn_create(scr);
    lv_obj_set_size(setup_btn, 212, 100);
    lv_obj_set_pos(setup_btn, 254, 320);
    lv_obj_set_style_bg_color(setup_btn, lv_color_hex(0x333355), 0);
    lv_obj_set_style_radius(setup_btn, 14, 0);
    lv_obj_set_style_border_width(setup_btn, 0, 0);
    lv_obj_t *setup_lbl = lv_label_create(setup_btn);
    lv_label_set_text(setup_lbl, "SETUP");
    lv_obj_set_style_text_font(setup_lbl, &lv_font_montserrat_24, 0);
    lv_obj_center(setup_lbl);
    lv_obj_set_user_data(setup_btn, (void *)&setup_target);
    lv_obj_add_event_cb(setup_btn, [](lv_event_t *e) {
        int idx = *(int *)lv_obj_get_user_data(lv_event_get_target(e));
        navigate_to(idx);
    }, LV_EVENT_CLICKED, NULL);

    return scr;
}

// ═══════════════════════════════════════════════════════════════════════════════
// Setup-Screen
// ═══════════════════════════════════════════════════════════════════════════════
lv_obj_t* build_setup_screen() {
    lv_obj_t *scr = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(scr, lv_color_hex(0x080810), 0);
    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);
    

    lv_obj_t *title = lv_label_create(scr);
    lv_label_set_text(title, "SETUP");
    lv_obj_set_style_text_font(title, &montserrat_48, 0);
    lv_obj_set_style_text_color(title, lv_color_hex(0x0080FF), 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 20);

    lv_obj_t *lbl = lv_label_create(scr);
    lv_label_set_text(lbl, "Helligkeit");
    lv_obj_set_style_text_font(lbl, &lv_font_montserrat_24, 0);
    lv_obj_set_style_text_color(lbl, lv_color_hex(0x888888), 0);
    lv_obj_align(lbl, LV_ALIGN_CENTER, 0, -90);
    slider_brightness = lv_slider_create(scr);


    lv_obj_set_width(slider_brightness, 420);
    lv_obj_set_height(slider_brightness, 30);
    lv_obj_align(slider_brightness, LV_ALIGN_CENTER, 0, -20);
    lv_slider_set_range(slider_brightness, 10, 100);
    lv_slider_set_value(slider_brightness, 100, LV_ANIM_OFF);
    lv_obj_add_event_cb(slider_brightness, brightness_event_cb, LV_EVENT_VALUE_CHANGED, NULL);

    label_brightness = lv_label_create(scr);
    lv_label_set_text(label_brightness, "100%");
    lv_obj_set_style_text_font(label_brightness, &montserrat_48, 0);
    lv_obj_set_style_text_color(label_brightness, lv_color_hex(0x00FFCC), 0);
    lv_obj_align(label_brightness, LV_ALIGN_CENTER, 0, 60);
    

    // ── Firmware-Version ──────────────────────────────────────────────────────────
        lv_obj_t *ver_lbl = lv_label_create(scr);
        lv_label_set_text(ver_lbl, "v" FW_VERSION " (" FW_DATE ")");
        lv_obj_set_style_text_font(ver_lbl, &lv_font_montserrat_24, 0);
        lv_obj_set_style_text_color(ver_lbl, lv_color_hex(0x446688), 0);
        lv_obj_set_width(ver_lbl, 460);                          // ← Fix: feste Breite
        lv_obj_set_style_text_align(ver_lbl, LV_TEXT_ALIGN_CENTER, 0);  // ← zentriert
        lv_obj_align(ver_lbl, LV_ALIGN_BOTTOM_MID, 0, -16);
    // ─────────────────────────────────────────────────────────────────────────────

    // ── Simulations-Buttons (AP-Test ohne Netzwerk) ──────────────────────────
    {
        lv_obj_t *sim_title = lv_label_create(scr);
        lv_label_set_text(sim_title, "AP Simulation");
        lv_obj_set_style_text_font(sim_title, &montserrat_18, 0);
        lv_obj_set_style_text_color(sim_title, lv_color_hex(0xAABBCC), 0);
        lv_obj_align(sim_title, LV_ALIGN_TOP_LEFT, 10, 300);

        // Basis-Farben: Index 0=STBY, 1=AUTO, 2=TRCK, 3=SIM OFF
        static const uint32_t sim_base_cols[4] = { 0x223344, 0x223344, 0x223344, 0x553333 };
        static const uint32_t sim_green        = 0x007700;

        const char*  labels[] = { "STBY", "AUTO", "TRCK", "SIM\nOFF" };
        const uint16_t vals[] = { 0x0000,  0x0040, 0x0180, 0xFFFF };
        static lv_obj_t* sim_btns[4];  // Pointer global im Block merken

        for (int i = 0; i < 4; i++) {
            lv_obj_t *btn = lv_btn_create(scr);
            sim_btns[i] = btn;
            lv_obj_set_size(btn, 100, 50);
            lv_obj_set_pos(btn, 10 + i * 112, 325);
            // SIM OFF startet grün (Sim ist aus = SIM OFF aktiv)
            lv_obj_set_style_bg_color(btn, lv_color_hex(i == 3 ? sim_green : sim_base_cols[i]), 0);
            lv_obj_set_style_radius(btn, 8, 0);
            lv_obj_set_style_border_width(btn, 0, 0);
            lv_obj_t *l = lv_label_create(btn);
            lv_label_set_text(l, labels[i]);
            lv_obj_set_style_text_font(l, &montserrat_18, 0);
            lv_obj_set_style_text_align(l, LV_TEXT_ALIGN_CENTER, 0);
            lv_obj_center(l);
            lv_obj_set_user_data(btn, (void*)(uintptr_t)vals[i]);
            lv_obj_add_event_cb(btn, [](lv_event_t *e) {
                uint16_t mode = (uint16_t)(uintptr_t)lv_obj_get_user_data(lv_event_get_target(e));
                // Alle 4 Buttons auf Grundfarbe zurücksetzen
                for (int j = 0; j < 4; j++) {
                   if (sim_btns[j]) {
                        lv_obj_set_style_bg_color(sim_btns[j], lv_color_hex(sim_base_cols[j]), 0);
                        lv_obj_invalidate(sim_btns[j]);  // ← Fix
                    }
                }
                // Gedrückten Button grün markieren
                    lv_obj_set_style_bg_color(lv_event_get_target(e), lv_color_hex(sim_green), 0);
                    lv_obj_invalidate(lv_event_get_target(e));  // ← Fix
                if (mode == 0xFFFF) {
                    // SIM OFF
                    sim_mode_active  = false;
                    n_autopilot_mode = -1;
                } else {
                    // SIM ON: gewählten Modus simulieren
                    sim_mode_active  = true;
                    n_autopilot_mode = mode;
                    g_pilot_addr     = 7;
                    n_dtw            = 2.35f;
                    n_ttg            = 47.0f;
                    n_ts_dtw         = millis();
                    strncpy((char*)n_wpt_name, "TARIFA", sizeof(n_wpt_name));
                }
            }, LV_EVENT_CLICKED, NULL);
        }
    }
    // ─────────────────────────────────────────────────────────────────────────
    return scr;
}
// ═══════════════════════════════════════════════════════════════════════════════
// MULTI-Screen
// ═══════════════════════════════════════════════════════════════════════════════
lv_obj_t* build_multi_screen() {
    lv_obj_t* scr = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(scr, lv_color_hex(0x080810), 0);
    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);
    

    lv_obj_t* title = lv_label_create(scr);
    lv_label_set_text(title, "INSTRUMENTS");
    lv_obj_set_style_text_font(title, &montserrat_48, 0);
    lv_obj_set_style_text_color(title, lv_color_hex(0x0080FF), 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 8);

    struct TileInfo { const char* lbl; const char* unit; int idx; };
    TileInfo tiles[6] = {
        {"SOG",   "kn",  0},
        {"HDG",   "\xc2\xb0",  1},   // °
        {"AWS",   "kn",  2},
        {"AWA",   "\xc2\xb0",  3},
        {"DEPTH", "m",   4},
        {"TWS",   "kn",  5},
    };

    const int TW = 226, TH = 130, GAP = 8, X0 = 10, Y0 = 55;

    for (int i = 0; i < 6; i++) {
        int col = i % 2, row = i / 2;
        int tx = X0 + col * (TW + GAP);
        int ty = Y0 + row * (TH + GAP);

        lv_obj_t* tile = lv_obj_create(scr);
        lv_obj_set_size(tile, TW, TH);
        lv_obj_set_pos(tile, tx, ty);
        lv_obj_set_style_bg_color(tile, lv_color_hex(0x0D1525), 0);
        lv_obj_set_style_border_color(tile, lv_color_hex(0x1E2E50), 0);
        lv_obj_set_style_border_width(tile, 2, 0);
        lv_obj_set_style_radius(tile, 12, 0);
        lv_obj_clear_flag(tile, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE);
        lv_obj_set_style_pad_all(tile, 0, 0);

        // Kürzel oben links
        lv_obj_t* name = lv_label_create(tile);
        lv_label_set_text(name, tiles[i].lbl);
        lv_obj_set_style_text_font(name, &montserrat_18, 0);
        lv_obj_set_style_text_color(name, lv_color_hex(0x6677AA), 0);
        lv_obj_align(name, LV_ALIGN_TOP_LEFT, 10, 7);

        // Einheit unten rechts
        lv_obj_t* unit = lv_label_create(tile);
        lv_label_set_text(unit, tiles[i].unit);
        lv_obj_set_style_text_font(unit, &montserrat_18, 0);
        lv_obj_set_style_text_color(unit, lv_color_hex(0x6677AA), 0);
        lv_obj_align(unit, LV_ALIGN_BOTTOM_RIGHT, -10, -7);

        // Großer Wert
        multi_lbl[i] = lv_label_create(tile);
        lv_label_set_text(multi_lbl[i], "---");
        lv_obj_set_style_text_font(multi_lbl[i], &montserrat_72, 0);
        lv_obj_set_style_text_color(multi_lbl[i], lv_color_hex(0x00FFCC), 0);
        lv_obj_set_width(multi_lbl[i], TW - 16);
        lv_obj_set_style_text_align(multi_lbl[i], LV_TEXT_ALIGN_RIGHT, 0);
        lv_obj_align(multi_lbl[i], LV_ALIGN_BOTTOM_MID, -8, -24);
    }

    return scr;
}

// ═══════════════════════════════════════════════════════════════════════════════
// Instrument-Screen
// ═══════════════════════════════════════════════════════════════════════════════
lv_obj_t* build_instrument_screen(int idx, const char *title, const char *unit, const char *sub_init) {
   
   
    lv_obj_t *scr = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(scr, lv_color_hex(0x0A0A14), 0);
    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);
    

    lv_obj_t *t = lv_label_create(scr);
    lv_label_set_text(t, title);
    lv_obj_set_style_text_font(t, &montserrat_48, 0);
    lv_obj_set_style_text_color(t, lv_color_hex(0xCCCCCC), 0);
    lv_obj_align(t, LV_ALIGN_TOP_LEFT, 12, 10);

    lbl_main[idx] = lv_label_create(scr);
    lv_label_set_text(lbl_main[idx], "---");

    // --- HIER DIE KORREKTUR (idx statt id) ---
    lv_obj_set_style_border_width(lbl_main[idx], 0, 0); 
    lv_obj_set_style_pad_all(lbl_main[idx], 0, 0);      
    lv_obj_set_style_bg_opa(lbl_main[idx], 0, 0);       
    lv_obj_clear_flag(lbl_main[idx], LV_OBJ_FLAG_SCROLLABLE);
    // -----------------------------------------

    lv_obj_set_style_text_font(lbl_main[idx], &montserrat_240_bold_numeric, 0);
    lv_obj_set_style_text_color(lbl_main[idx], lv_color_hex(0x00FFCC), 0);
    lv_obj_set_width(lbl_main[idx], 480);
    lv_obj_set_style_text_align(lbl_main[idx], LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(lbl_main[idx], LV_ALIGN_CENTER, 0, 0);

    lv_obj_t *u = lv_label_create(scr);
    lv_label_set_text(u, unit);
    lv_obj_set_style_text_font(u, &montserrat_48, 0);
    lv_obj_set_style_text_color(u, lv_color_hex(0xCCCCCC), 0);
    lv_obj_align(u, LV_ALIGN_BOTTOM_RIGHT, -12, -80);

    lbl_sub[idx] = lv_label_create(scr);
    lv_label_set_text(lbl_sub[idx], sub_init);
    lv_obj_set_style_text_font(lbl_sub[idx], &montserrat_48, 0);
    lv_obj_set_style_text_color(lbl_sub[idx], lv_color_hex(0xCCCCCC), 0);
    lv_obj_align(lbl_sub[idx], LV_ALIGN_BOTTOM_LEFT, 12, -80);

    return scr;
}

// ═══════════════════════════════════════════════════════════════════════════════
// Wind Screen – Original Layout
// ═══════════════════════════════════════════════════════════════════════════════
lv_obj_t* build_wind_screen() {
    lv_obj_t *scr = lv_obj_create(NULL);
    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);
    
    lv_obj_set_style_bg_color(scr, lv_color_hex(0x0A0A12), 0);

    lv_obj_t *sog_box = lv_obj_create(scr);
    lv_obj_set_size(sog_box, 180, 100);
    lv_obj_set_pos(sog_box, 0, 0);
    lv_obj_set_style_bg_color(sog_box, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_border_color(sog_box, lv_color_hex(0xCCCCCC), 0);
    lv_obj_set_style_border_width(sog_box, 2, 0);
    lv_obj_set_style_radius(sog_box, 12, 0);
    lv_obj_clear_flag(sog_box, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE);

    lv_obj_t *depth_box = lv_obj_create(scr);
    lv_obj_set_size(depth_box, 180, 100);
    lv_obj_set_pos(depth_box, 480 - 180, 0);
    lv_obj_set_style_bg_color(depth_box, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_border_color(depth_box, lv_color_hex(0xCCCCCC), 0);
    lv_obj_set_style_border_width(depth_box, 2, 0);
    lv_obj_set_style_radius(depth_box, 12, 0);
    lv_obj_clear_flag(depth_box, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE);

    lv_obj_set_style_pad_all(sog_box, 0, 0);
    lv_obj_set_style_pad_all(depth_box, 0, 0);

    int circle_center_y = WIND_CY + CIRCLE_Y_OFFSET;

    lv_obj_t *scale_bg = lv_obj_create(scr);
    lv_obj_set_size(scale_bg, (R_SCALE + 20) * 2, (R_SCALE + 20) * 2);
    lv_obj_align(scale_bg, LV_ALIGN_CENTER, 0, CIRCLE_Y_OFFSET);
    lv_obj_set_style_bg_color(scale_bg, lv_color_hex(0xE0E0E0), 0);
    lv_obj_set_style_bg_opa(scale_bg, LV_OPA_100, 0);
    lv_obj_set_style_border_width(scale_bg, 0, 0);
    lv_obj_set_style_radius(scale_bg, R_SCALE + 20, 0);
    lv_obj_clear_flag(scale_bg, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE);

    lv_obj_t *center_bg = lv_obj_create(scr);
    lv_obj_set_size(center_bg, R_CENTER * 2, R_CENTER * 2);
    lv_obj_align(center_bg, LV_ALIGN_CENTER, 0, CIRCLE_Y_OFFSET);
    lv_obj_set_style_bg_color(center_bg, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_bg_opa(center_bg, LV_OPA_100, 0);
    lv_obj_set_style_border_width(center_bg, 0, 0);
    lv_obj_set_style_radius(center_bg, R_CENTER, 0);
    lv_obj_clear_flag(center_bg, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE);

    lv_obj_t *grey_ring = lv_obj_create(scr);
    lv_obj_set_size(grey_ring, R_GREY * 2, R_GREY * 2);
    lv_obj_align(grey_ring, LV_ALIGN_CENTER, 0, CIRCLE_Y_OFFSET);
    lv_obj_set_style_bg_opa(grey_ring, 0, 0);
    lv_obj_set_style_border_color(grey_ring, lv_color_hex(0x606068), 0);
    lv_obj_set_style_border_width(grey_ring, 5, 0);
    lv_obj_set_style_radius(grey_ring, R_GREY, 0);
    lv_obj_clear_flag(grey_ring, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE);

    lv_obj_t *arc_red = lv_arc_create(scr);
    lv_obj_set_size(arc_red, R_COLOR * 2, R_COLOR * 2);
    lv_obj_align(arc_red, LV_ALIGN_CENTER, 0, CIRCLE_Y_OFFSET);
    lv_obj_remove_style(arc_red, NULL, LV_PART_INDICATOR);
    lv_obj_remove_style(arc_red, NULL, LV_PART_KNOB);
    lv_arc_set_bg_angles(arc_red, 90, 270);
    lv_obj_set_style_arc_color(arc_red, lv_color_hex(0xDD3333), LV_PART_MAIN);
    lv_obj_set_style_arc_width(arc_red, 14, LV_PART_MAIN);
    lv_obj_set_style_arc_rounded(arc_red, false, LV_PART_MAIN);
    lv_obj_clear_flag(arc_red, LV_OBJ_FLAG_CLICKABLE);

    lv_obj_t *arc_green = lv_arc_create(scr);
    lv_obj_set_size(arc_green, R_COLOR * 2, R_COLOR * 2);
    lv_obj_align(arc_green, LV_ALIGN_CENTER, 0, CIRCLE_Y_OFFSET);
    lv_obj_remove_style(arc_green, NULL, LV_PART_INDICATOR);
    lv_obj_remove_style(arc_green, NULL, LV_PART_KNOB);
    lv_arc_set_bg_angles(arc_green, 270, 90);
    lv_obj_set_style_arc_color(arc_green, lv_color_hex(0x33AA33), LV_PART_MAIN);
    lv_obj_set_style_arc_width(arc_green, 14, LV_PART_MAIN);
    lv_obj_set_style_arc_rounded(arc_green, false, LV_PART_MAIN);
    lv_obj_clear_flag(arc_green, LV_OBJ_FLAG_CLICKABLE);

    static lv_point_t tick_pts[36][2];
    int tick_idx = 0;
    int outer_ring_radius = R_SCALE + 20;
    for (int deg = 0; deg < 360; deg += 10) {
        bool major = (deg % 30 == 0);
        float rad = (270.0f - deg) * (float)M_PI / 180.0f;
        int r_outer = outer_ring_radius - 2;
        int r_inner = major ? (outer_ring_radius - 35) : (outer_ring_radius - 20);
        int x1 = WIND_CX + (int)(r_outer * cosf(rad));
        int y1 = circle_center_y + (int)(r_outer * sinf(rad));
        int x2 = WIND_CX + (int)(r_inner * cosf(rad));
        int y2 = circle_center_y + (int)(r_inner * sinf(rad));
        tick_pts[tick_idx][0] = {(lv_coord_t)x1, (lv_coord_t)y1};
        tick_pts[tick_idx][1] = {(lv_coord_t)x2, (lv_coord_t)y2};
        lv_obj_t *line = lv_line_create(scr);
        lv_line_set_points(line, tick_pts[tick_idx], 2);
        lv_obj_set_style_line_color(line, lv_color_hex(major ? 0x222222 : 0x555555), 0);
        lv_obj_set_style_line_width(line, major ? 4 : 2, 0);
        tick_idx++;
    }

    struct DegLbl { int deg; const char* txt; };
    DegLbl labels[] = {
        {  0,   "0" }, { 30,  "30" }, { 60,  "60" }, { 90,  "90" },
        {120, "120" }, {150, "150" }, {210, "150" }, {240, "120" },
        {270,  "90" }, {300,  "60" }, {330,  "30" }
    };
    int R_LABEL = (R_SCALE + R_GREY) / 2 - 8;
    for (int i = 0; i < 11; i++) {
        float rad = (270.0f - labels[i].deg) * (float)M_PI / 180.0f;
        int lx = WIND_CX + (int)(R_LABEL * cosf(rad));
        int ly = circle_center_y + (int)(R_LABEL * sinf(rad));
        lv_obj_t *lbl = lv_label_create(scr);
        lv_label_set_text(lbl, labels[i].txt);
        lv_obj_set_style_text_font(lbl, &montserrat_36, 0);
        lv_obj_set_style_text_color(lbl, lv_color_hex(0x111111), 0);
        lv_obj_set_pos(lbl, lx - 15, ly - 14);
    }

    wind_sog_box = lv_label_create(sog_box);
    lv_label_set_text(wind_sog_box, "---");
    lv_obj_set_style_text_font(wind_sog_box, &montserrat_48, 0);
    lv_obj_set_style_text_color(wind_sog_box, lv_color_hex(0x000000), 0);
    lv_obj_align(wind_sog_box, LV_ALIGN_TOP_LEFT, 12, 12);
    
    lv_obj_t *sog_title = lv_label_create(sog_box);
    lv_label_set_text(sog_title, "SOG");
    lv_obj_set_style_text_font(sog_title, &montserrat_18, 0);
    lv_obj_set_style_text_color(sog_title, lv_color_hex(0x000000), 0);
    lv_obj_align(sog_title, LV_ALIGN_BOTTOM_LEFT, 12, -12);

    wind_depth_box = lv_label_create(depth_box);
    lv_label_set_text(wind_depth_box, "---");
    lv_obj_set_style_text_font(wind_depth_box, &montserrat_48, 0);
    lv_obj_set_style_text_color(wind_depth_box, lv_color_hex(0x000000), 0);
    lv_obj_align(wind_depth_box, LV_ALIGN_TOP_RIGHT, -12, 12);
    
    lv_obj_t *depth_title = lv_label_create(depth_box);
    lv_label_set_text(depth_title, "Depth");
    lv_obj_set_style_text_font(depth_title, &montserrat_18, 0);
    lv_obj_set_style_text_color(depth_title, lv_color_hex(0x000000), 0);
    lv_obj_align(depth_title, LV_ALIGN_BOTTOM_RIGHT, -12, -12);

    lv_obj_t *spd_label = lv_label_create(scr);
    lv_label_set_text(spd_label, "AWS");
    lv_obj_set_style_text_font(spd_label, &lv_font_montserrat_24, 0);
    lv_obj_set_style_text_color(spd_label, lv_color_hex(0x666666), 0);
    lv_obj_align(spd_label, LV_ALIGN_CENTER, 0, -70);
    wind_speed_label = spd_label;

    aws_int_lbl = lv_label_create(scr);
    lv_label_set_text(aws_int_lbl, "--");
    lv_obj_set_style_text_font(aws_int_lbl, &montserrat_120, 0);
    lv_obj_set_style_text_color(aws_int_lbl, lv_color_hex(0x111111), 0);
    lv_obj_align(aws_int_lbl, LV_ALIGN_CENTER, -40, -2);

    aws_dec_lbl = lv_label_create(scr);
    lv_label_set_text(aws_dec_lbl, ".-");
    lv_obj_set_style_text_font(aws_dec_lbl, &montserrat_72, 0);
    lv_obj_set_style_text_color(aws_dec_lbl, lv_color_hex(0x222222), 0);
    lv_obj_align_to(aws_dec_lbl, aws_int_lbl, LV_ALIGN_OUT_RIGHT_BOTTOM, 2, 0);

    lv_obj_t *kts_label = lv_label_create(scr);
    lv_label_set_text(kts_label, "Kts");
    lv_obj_set_style_text_font(kts_label, &lv_font_montserrat_24, 0);
    lv_obj_set_style_text_color(kts_label, lv_color_hex(0x888888), 0);
    lv_obj_align(kts_label, LV_ALIGN_CENTER, 0, 70);

    lv_obj_t *mode_btn = lv_btn_create(scr);
    lv_obj_set_size(mode_btn, 120, 60);
    lv_obj_align(mode_btn, LV_ALIGN_CENTER, 0, 115);
    lv_obj_set_style_bg_color(mode_btn, lv_color_hex(0x880000), 0);
    lv_obj_set_style_border_width(mode_btn, 0, 0);
    lv_obj_set_style_radius(mode_btn, 8, 0);
    lv_obj_clear_flag(mode_btn, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(mode_btn, wind_mode_btn_cb, LV_EVENT_CLICKED, NULL);
    
    mode_btn_label = lv_label_create(mode_btn);
    lv_label_set_text(mode_btn_label, "App");
    lv_obj_set_style_text_font(mode_btn_label, &montserrat_48, 0);
    lv_obj_set_style_text_color(mode_btn_label, lv_color_hex(0xFFFFFF), 0);
    lv_obj_center(mode_btn_label);
    
    lv_obj_t *bottom_box = lv_obj_create(scr);
    lv_obj_set_size(bottom_box, 260, 75);
    lv_obj_align(bottom_box, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_obj_set_style_bg_color(bottom_box, lv_color_hex(0x000000), 0);
    lv_obj_set_style_border_color(bottom_box, lv_color_hex(0x333344), 0);
    lv_obj_set_style_border_width(bottom_box, 2, 0);
    lv_obj_set_style_radius(bottom_box, 10, 0);
    lv_obj_clear_flag(bottom_box, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE);

    wind_ang_lbl = lv_label_create(bottom_box);
    lv_label_set_text(wind_ang_lbl, "---°");
    lv_obj_set_style_text_font(wind_ang_lbl, &montserrat_72, 0);
    lv_obj_set_style_text_color(wind_ang_lbl, lv_color_hex(0xFFFFFF), 0);
    lv_obj_center(wind_ang_lbl);
    
    canvas_buf = (lv_color_t *)ps_malloc(LV_CANVAS_BUF_SIZE_TRUE_COLOR_ALPHA(CANVAS_SIZE, CANVAS_SIZE));
    wind_canvas = lv_canvas_create(scr);
    lv_canvas_set_buffer(wind_canvas, canvas_buf, CANVAS_SIZE, CANVAS_SIZE, LV_IMG_CF_TRUE_COLOR_ALPHA);
    lv_canvas_fill_bg(wind_canvas, lv_color_hex(0x000000), LV_OPA_TRANSP);
    lv_obj_clear_flag(wind_canvas, LV_OBJ_FLAG_CLICKABLE);
    build_wind_needle_sprite();
    place_wind_needle(0.0f);

    return scr;
}

// ═══════════════════════════════════════════════════════════════════════════════
// HEADING ANALOG
// ═══════════════════════════════════════════════════════════════════════════════
void draw_compass_rose(lv_obj_t *canvas) {
    lv_canvas_fill_bg(canvas, lv_color_hex(0x001A33), LV_OPA_TRANSP);

    int cx = HDG_CENTER;
    int cy = HDG_CENTER;
    int r = HDG_RADIUS;

    lv_draw_arc_dsc_t arc_dsc;
    lv_draw_arc_dsc_init(&arc_dsc);

    // Äußerer Ring
    arc_dsc.color = lv_color_hex(0x445566);
    arc_dsc.width = 36;
    lv_canvas_draw_arc(canvas, cx, cy, r - 20, 0, 360, &arc_dsc);

    // Cyan Glow Ring
    arc_dsc.color = lv_color_hex(0x00CCFF);
    arc_dsc.width = 6;
    lv_canvas_draw_arc(canvas, cx, cy, r - 14, 0, 360, &arc_dsc);

    // Skalenstriche
    lv_draw_line_dsc_t line_dsc;
    lv_draw_line_dsc_init(&line_dsc);
    int r_outer_ticks = r - 20;

    for (int deg = 0; deg < 360; deg += 5) {
        bool label_pos = (deg % 30 == 0);
        bool ten_deg   = (deg % 10 == 0);
        int length = 0;

        if (label_pos) {
            line_dsc.color = lv_color_hex(0xFFFFFF);
            line_dsc.width = 8;
            length = 45;
        } else if (ten_deg) {
            line_dsc.color = lv_color_hex(0xFFFFFF);
            line_dsc.width = 4;
            length = 30;
        } else {
            line_dsc.color = lv_color_hex(0xFFFFFF);
            line_dsc.width = 1;
            length = 15;
        }

        float rad = deg * (float)M_PI / 180.0f; 
        lv_point_t pts[2];
        pts[0].x = cx + (int)(r_outer_ticks * sinf(rad));   // sin/cos tauschen!
        pts[0].y = cy - (int)(r_outer_ticks * cosf(rad));   // 0° → oben
        pts[1].x = cx + (int)((r_outer_ticks - length) * sinf(rad));
        pts[1].y = cy - (int)((r_outer_ticks - length) * cosf(rad));
        lv_canvas_draw_line(canvas, pts, 2, &line_dsc);
    }
}

void heading_anim_cb(lv_timer_t *t) {
    if (lv_scr_act() != screens[5]) return;  // ← NEU: nur wenn HDG-Analog aktiv
    // Rose nur drehen wenn echtes Heading vorhanden
    if (s_heading >= 0.0f) {
        float diff = s_heading - current_hdg_damped;
        while (diff < -180.0f) diff += 360.0f;
        while (diff > 180.0f) diff -= 360.0f;
        current_hdg_damped += diff / HDG_DAMPING_FACTOR;
        while (current_hdg_damped >= 360.0f) current_hdg_damped -= 360.0f;
        while (current_hdg_damped < 0.0f)    current_hdg_damped += 360.0f;
        // Mindestschwelle: Rose nur drehen wenn Änderung > 0.5°
        static float last_drawn_hdg = -9999.f;
        if (fabsf(current_hdg_damped - last_drawn_hdg) < 0.5f) return;  // ← NEU
        last_drawn_hdg = current_hdg_damped;                              // ← NEU
        if (hdg_rose_img) {
            //lv_img_set_angle(hdg_rose_img, (int16_t)(current_hdg_damped * 10));
            int16_t rose_angle = (int16_t)(fmodf(360.0f - current_hdg_damped, 360.0f) * 10.0f);
            lv_img_set_angle(hdg_rose_img, rose_angle);
        }
    }

    // Labels IMMER positionieren (auch ohne Heading → zeigt Nordausrichtung)
    int cx_screen = 240;
    int cy_screen = 665;
    float rot = current_hdg_damped;   // ist 0.0 solange kein Heading

    for (int i = 0; i < hdg_label_count; i++) {
        if (!hdg_labels[i]) continue;
        float theta_new_deg = hdg_label_base_angles[i] - rot;

        float rad = theta_new_deg * (float)M_PI / 180.0f;
        int r = hdg_label_is_dir[i] ? (HDG_RADIUS - 110) : (HDG_RADIUS - 90);
        int x = cx_screen + (int)(r * sinf(rad));   // sin/cos tauschen!
        int y = cy_screen - (int)(r * cosf(rad));   // 0° → oben (unter Zeiger) ✓

  
        lv_obj_set_pos(hdg_labels[i], x - 25, y - 20);
    }

    // ── Digitale Anzeige aktualisieren ──────────────────────────────────────
    if (hdg_analog_val_lbl) {
        static int last_hdg_txt = -1000;
        if (s_heading < 0.0f) {
            if (last_hdg_txt != -9999) {
                lv_label_set_text(hdg_analog_val_lbl, "---");
                last_hdg_txt = -9999;
            }
        } else {
            int shown = (int)roundf(fmodf(current_hdg_damped + 360.0f, 360.0f));
            if (shown != last_hdg_txt) {
                last_hdg_txt = shown;
                char buf[12];
                snprintf(buf, sizeof(buf), "%03d\xC2\xB0", shown);
                lv_label_set_text(hdg_analog_val_lbl, buf);
            }
        }
    }
}

lv_obj_t* build_heading_analog_screen() {
    lv_obj_t *scr = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(scr, lv_color_hex(0x001A33), 0);
    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);
    

    // ─── SOG‑Box oben links ─────────────────────────────────────────
    lv_obj_t *sog_box = lv_obj_create(scr);
    lv_obj_set_size(sog_box, 180, 100);
    lv_obj_set_pos(sog_box, 0, 0);
    lv_obj_set_style_bg_color(sog_box, lv_color_hex(0x1A2A3A), 0);
    lv_obj_set_style_border_color(sog_box, lv_color_hex(0x0080FF), 0);
    lv_obj_set_style_border_width(sog_box, 2, 0);
    lv_obj_set_style_radius(sog_box, 12, 0);
    lv_obj_clear_flag(sog_box, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_style_pad_all(sog_box, 0, 0);

    lbl_main[5] = lv_label_create(sog_box);
    lv_label_set_text(lbl_main[5], "---");
    lv_obj_set_style_text_font(lbl_main[5], &montserrat_48, 0);
    lv_obj_set_style_text_color(lbl_main[5], lv_color_hex(0x00FFCC), 0);
    lv_obj_set_width(lbl_main[5], 156);
    lv_obj_set_style_text_align(lbl_main[5], LV_TEXT_ALIGN_RIGHT, 0);
    lv_obj_align(lbl_main[5], LV_ALIGN_TOP_RIGHT, -12, 25);

    lv_obj_t *sog_title = lv_label_create(sog_box);
    lv_label_set_text(sog_title, "SOG");
    lv_obj_set_style_text_font(sog_title, &montserrat_18, 0);
    lv_obj_set_style_text_color(sog_title, lv_color_hex(0x8888AA), 0);
    lv_obj_align(sog_title, LV_ALIGN_BOTTOM_LEFT, 12, -12);

    // ─── Depth‑Box oben rechts ─────────────────────────────────────
    lv_obj_t *depth_box = lv_obj_create(scr);
    lv_obj_set_size(depth_box, 180, 100);
    lv_obj_set_pos(depth_box, 480 - 180, 0);
    lv_obj_set_style_bg_color(depth_box, lv_color_hex(0x1A2A3A), 0);
    lv_obj_set_style_border_color(depth_box, lv_color_hex(0x0080FF), 0);
    lv_obj_set_style_border_width(depth_box, 2, 0);
    lv_obj_set_style_radius(depth_box, 12, 0);
    lv_obj_clear_flag(depth_box, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_style_pad_all(depth_box, 0, 0);

    lbl_sub[5] = lv_label_create(depth_box);
    lv_label_set_text(lbl_sub[5], "---");
    lv_obj_set_style_text_font(lbl_sub[5], &montserrat_48, 0);
    lv_obj_set_style_text_color(lbl_sub[5], lv_color_hex(0x00FFCC), 0);
    lv_obj_set_width(lbl_sub[5], 156);
    lv_obj_set_style_text_align(lbl_sub[5], LV_TEXT_ALIGN_LEFT, 0);
    lv_obj_align(lbl_sub[5], LV_ALIGN_TOP_LEFT, 12, 25);

    lv_obj_t *depth_title = lv_label_create(depth_box);
    lv_label_set_text(depth_title, "Depth");
    lv_obj_set_style_text_font(depth_title, &montserrat_18, 0);
    lv_obj_set_style_text_color(depth_title, lv_color_hex(0x8888AA), 0);
    lv_obj_align(depth_title, LV_ALIGN_BOTTOM_RIGHT, -12, -12);

    // ─── Kompassrose (Canvas) ─────────────────────────────────
    hdg_rose_buf = (lv_color_t *)ps_malloc(
        LV_CANVAS_BUF_SIZE_TRUE_COLOR_ALPHA(HDG_ROSE_SIZE, HDG_ROSE_SIZE));
    if (!hdg_rose_buf) { Serial.println("ERROR: compass buf"); while(1); }

    hdg_rose_canvas = lv_canvas_create(scr);
    lv_canvas_set_buffer(hdg_rose_canvas, hdg_rose_buf,
        HDG_ROSE_SIZE, HDG_ROSE_SIZE, LV_IMG_CF_TRUE_COLOR_ALPHA);
    lv_canvas_fill_bg(hdg_rose_canvas, lv_color_hex(0x000000), LV_OPA_TRANSP);
    draw_compass_rose(hdg_rose_canvas);

    // Direkt den Canvas positionieren und als Rotations-Objekt verwenden
    // (wie im Original – kein separates lv_img_create!)
    int offset_x = (480 - HDG_ROSE_SIZE) / 2;   // = -135
    int offset_y = 300;
    lv_obj_set_pos(hdg_rose_canvas, offset_x, offset_y);
    lv_img_set_pivot(hdg_rose_canvas, HDG_CENTER, HDG_CENTER);
    lv_obj_clear_flag(hdg_rose_canvas, LV_OBJ_FLAG_CLICKABLE);

    // hdg_rose_img zeigt auf denselben Canvas (für heading_anim_cb)
    hdg_rose_img = hdg_rose_canvas;   // ← kein separates lv_img_create mehr!

    // ── lv_img_create und lv_img_set_src KOMPLETT LÖSCHEN ──
    // hdg_rose_img = lv_img_create(scr);          ← ENTFERNEN
    // lv_img_set_src(hdg_rose_img, hdg_rose_canvas); ← ENTFERNEN

    // ─── Labels (Gradzahlen und Himmelsrichtungen) ─────────────────
    hdg_label_count = 0;

    for (int deg = 0; deg < 360; deg += 30) {
        if (deg % 90 == 0) continue;  // Himmelsrichtungen separat
        hdg_labels[hdg_label_count] = lv_label_create(scr);
        lv_obj_set_style_text_font(hdg_labels[hdg_label_count], &montserrat_48, 0);
        lv_obj_set_style_text_color(hdg_labels[hdg_label_count], lv_color_hex(0xFFFFFF), 0);
        lv_label_set_text_fmt(hdg_labels[hdg_label_count], "%d", deg);
        hdg_label_base_angles[hdg_label_count] = (float)deg;
        hdg_label_is_dir[hdg_label_count] = false;
        hdg_label_count++;
    }

    const char* dirs[] = {"N", "E", "S", "W"};
    const float angles[] = {0, 90, 180, 270};
    lv_color_t dir_colors[] = {lv_color_hex(0xFF6600), lv_color_hex(0xFFD700),
                               lv_color_hex(0xFFD700), lv_color_hex(0xFFD700)};

    for (int i = 0; i < 4; i++) {
        hdg_labels[hdg_label_count] = lv_label_create(scr);
        lv_obj_set_style_text_font(hdg_labels[hdg_label_count], &montserrat_48, 0);
        lv_obj_set_style_text_color(hdg_labels[hdg_label_count], dir_colors[i], 0);
        lv_label_set_text(hdg_labels[hdg_label_count], dirs[i]);
        hdg_label_base_angles[hdg_label_count] = angles[i];
        hdg_label_is_dir[hdg_label_count] = true;
        hdg_label_count++;
    }

// ─── Digitale Heading-Anzeige, mittig zwischen Boxen und Zeiger ──────────────

    hdg_analog_val_lbl = lv_label_create(scr);
    lv_label_set_text(hdg_analog_val_lbl, "---");
    lv_obj_set_style_text_font(hdg_analog_val_lbl, &montserrat_72, 0);
    lv_obj_set_style_text_color(hdg_analog_val_lbl, lv_color_hex(0x00FFCC), 0);
    lv_obj_set_style_bg_opa(hdg_analog_val_lbl, LV_OPA_TRANSP, 0);
    lv_obj_set_width(hdg_analog_val_lbl, 260);      // breiter wegen "°"
    lv_obj_set_style_text_align(hdg_analog_val_lbl, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(hdg_analog_val_lbl, LV_ALIGN_TOP_MID, 0, 220); // ← näher am Zeiger

// ─── Fester Zeiger (Dreieck) ──────────────────────────────────── (war hier)

    // ─── Fester Zeiger (Dreieck) ───────────────────────────────────
    lv_color_t *needle_buf = (lv_color_t *)ps_malloc(LV_CANVAS_BUF_SIZE_TRUE_COLOR_ALPHA(100, 100));
    if (needle_buf) {
        lv_obj_t *needle_canvas = lv_canvas_create(scr);
        lv_canvas_set_buffer(needle_canvas, needle_buf, 100, 100, LV_IMG_CF_TRUE_COLOR_ALPHA);
        lv_canvas_fill_bg(needle_canvas, lv_color_hex(0x000000), LV_OPA_TRANSP);
        lv_draw_rect_dsc_t rect_dsc;
        lv_draw_rect_dsc_init(&rect_dsc);
        rect_dsc.bg_color = (wind_mode == MODE_TWS_TWA)
                        ? lv_color_hex(0x0066FF)   // Blau = True Wind
                        : lv_color_hex(0xFF3300);  // Rot  = Apparent Wind
        rect_dsc.bg_opa = LV_OPA_COVER;

        lv_point_t pts[3] = {{50, 85}, {40, 10}, {60, 10}};
        lv_canvas_draw_polygon(needle_canvas, pts, 3, &rect_dsc);
        lv_obj_set_pos(needle_canvas, 190, 295);   // Position über der Rose
        lv_obj_clear_flag(needle_canvas, LV_OBJ_FLAG_CLICKABLE);
    }

    return scr;
}
// ═══════════════════════════════════════════════════════════════════════════════
// Hilfsfunktionen für Formatierung
// ═══════════════════════════════════════════════════════════════════════════════
// War: float last  ← pass-by-value, Cache funktioniert NIE
// Fix: float& last ← pass-by-reference
static void fmt_val_cached(lv_obj_t* lbl, float val, float& last,
                            const char* fmt, float maxval) {
    if (!lbl) return;
    if (fabsf(val - last) < 0.05f) return;
    last = val;
    if (val < 0.0f) { lv_label_set_text(lbl, "---"); return; }
    if (val > maxval) { lv_label_set_text(lbl, "MAX"); return; }
    char buf[32];
    snprintf(buf, sizeof(buf), fmt, val);
    lv_label_set_text(lbl, buf);
}

void fmt_deg(lv_obj_t *lbl, float val) {
    if (!lbl) return;
    if (val < 0.0f) { lv_label_set_text(lbl, "---"); return; }
    char buf[16];
    snprintf(buf, sizeof(buf), "%.0f", fmodf(val + 360.0f, 360.0f));
    lv_label_set_text(lbl, buf);
}

void fmt_pos(lv_obj_t* lbl, double deg, bool is_lat) {
    if (!lbl) return;
    if (fabs(deg) > 180.0) { lv_label_set_text(lbl, "---"); return; }
    char dir = is_lat ? (deg >= 0 ? 'N' : 'S') : (deg >= 0 ? 'E' : 'W');
    double abs_deg = fabs(deg);   // ← NEU: Absolutwert verwenden
    int d = (int)abs_deg;
    double m = (abs_deg - d) * 60.0;
    char buf[32];
    snprintf(buf, sizeof(buf), "%d° %06.3f' %c", d, m, dir);
    lv_label_set_text(lbl, buf);
}

// ═══════════════════════════════════════════════════════════════════════════════
// Display-Update
// ═══════════════════════════════════════════════════════════════════════════════
void update_displays() {
    wdg_lvgl_ms = millis();   // ← NEU: Watchdog-Heartbeat
    static float last_depth = 0, last_stw = 0, last_sog = 0,
                 last_hdg = 0, last_wtemp = 0, last_rudder = 0;

    // ── NMEA Bus Dot ─────────────────────────────────────────────────────────
    if (nmea_dot) {
        uint32_t age = millis() - last_nmea_msg_ms;
        lv_color_t dot_col;
        if      (age < 2000)  dot_col = lv_color_hex(0x00CC44);
        else if (age < 10000) dot_col = lv_color_hex(0xFFAA00);
        else                  dot_col = lv_color_hex(0xFF2200);
        lv_obj_set_style_bg_color(nmea_dot, dot_col, 0);
    }

    // ── lbl_main[1]  DEPTH ───────────────────────────────────────────────────
    if (fabsf(s_depth - last_depth) > 0.05f) {
        last_depth = s_depth;
        if (s_depth < 0.0f) {
            lv_label_set_text(lbl_main[1], "---");
        } else {
            char buf[16];
            if (s_depth < 10.0f) snprintf(buf, sizeof(buf), "%.1f", s_depth);
            else                  snprintf(buf, sizeof(buf), "%.0f", s_depth);
            lv_label_set_text(lbl_main[1], buf);
        }
    }
    // ✅ Farbe IMMER aktualisieren — auch bei eingefrorenen Daten
    lv_obj_set_style_text_color(lbl_main[1], data_color(s_ts_depth), 0);

    // ── lbl_sub[1]  Wassertemperatur ─────────────────────────────────────────
    if (fabsf(s_water_temp - last_wtemp) > 0.1f) {
        last_wtemp = s_water_temp;
        if (s_water_temp < -100.0f) {
            lv_label_set_text(lbl_sub[1], "---");
        } else {
            char buf[24];
            snprintf(buf, sizeof(buf), "%.1f\xc2\xb0""C", s_water_temp);   // °C UTF-8
            lv_label_set_text(lbl_sub[1], buf);
        }
    }

    // ── lbl_main[2]  SOG ─────────────────────────────────────────────────────
    fmt_val_cached(lbl_main[2], s_sog, last_sog);
    lv_obj_set_style_text_color(lbl_main[2], data_color(s_ts_sog), 0);

    // ── lbl_sub[2]  STW-Anzeige auf SOG-Screen ───────────────────────────────
    if (lbl_sub[2]) {
        static float last_stw_for_sog = -99.0f;
        if (fabsf(s_stw - last_stw_for_sog) > 0.05f) {
            last_stw_for_sog = s_stw;
            char buf[32];
            if (s_stw > 0) snprintf(buf, sizeof(buf), "STW %.1f kn", s_stw);
            else            strcpy(buf, "---");
            lv_label_set_text(lbl_sub[2], buf);
        }
    }

    // ── lbl_main[3]  STW ─────────────────────────────────────────────────────
    fmt_val_cached(lbl_main[3], s_stw, last_stw);
    lv_obj_set_style_text_color(lbl_main[3], data_color(s_ts_stw), 0);

    // ── lbl_sub[3]  SOG-Anzeige auf STW-Screen ───────────────────────────────
    if (lbl_sub[3]) {
        static float last_sog_for_stw = -99.0f;
        if (fabsf(s_sog - last_sog_for_stw) > 0.05f) {
            last_sog_for_stw = s_sog;
            char buf[32];
            if (s_sog > 0) snprintf(buf, sizeof(buf), "SOG %.1f kn", s_sog);
            else            strcpy(buf, "---");
            lv_label_set_text(lbl_sub[3], buf);
        }
    }

    // ── Wind Screen (idx 4) ──────────────────────────────────────────────────
    if (wind_sog_box) {
        static float last_sogbox = -1.0f;
        if (fabsf(s_sog - last_sogbox) > 0.05f) {
            last_sogbox = s_sog;
            char buf[16];
            if (s_sog >= 0) snprintf(buf, sizeof(buf), "%.1f", s_sog); else strcpy(buf, "---");
            lv_label_set_text(wind_sog_box, buf);
        }
    }
    if (wind_depth_box) {
        static float last_depbox = -99.0f;
        if (fabsf(s_depth - last_depbox) > 0.1f) {
            last_depbox = s_depth;
            char buf[16];
            if (s_depth >= 0) snprintf(buf, sizeof(buf), "%.1f", s_depth); else strcpy(buf, "---");
            lv_label_set_text(wind_depth_box, buf);
        }
    }

        // ── NEU ──────────────────────────────────────────────────────────
        if (aws_int_lbl && aws_dec_lbl) {
            float spd_raw = (wind_mode == MODE_TWS_TWA) ? s_tws : s_wind_spd;
            if (spd_raw >= 0.0f) {
                // Gedämpften Wert anzeigen – Reset wenn Signal nach Ausfall wiederkommt
                if (current_wspd_damped <= 0.0f)
                    current_wspd_damped = spd_raw;          // sofort einspringen beim ersten Wert
                int w_int = (int)current_wspd_damped;
                int w_dec = (int)((current_wspd_damped - w_int) * 10.0f);
                lv_label_set_text_fmt(aws_int_lbl, "%d",  w_int);
                lv_label_set_text_fmt(aws_dec_lbl, ".%d", w_dec);
            } else {
                current_wspd_damped = 0.0f;                 // Reset bei kein Signal
                lv_label_set_text(aws_int_lbl, "--");
                lv_label_set_text(aws_dec_lbl, ".-");
            }
            lv_obj_set_style_text_color(aws_int_lbl, wind_speed_color(s_ts_wind), 0);
            lv_obj_set_style_text_color(aws_dec_lbl, wind_speed_color(s_ts_wind), 0);
        }




    
    if (wind_ang_lbl) {
        if (wind_mode == MODE_AWS_AWA) {
            float ang = s_wind_ang;
            if (ang >= -720.0f && ang <= 720.0f) {
                float ang360 = fmodf(ang, 360.0f);
                if (ang360 < 0) ang360 += 360.0f;
                char buf[16];
                snprintf(buf, sizeof(buf), "%03.0f\xc2\xb0", ang360);
                lv_label_set_text(wind_ang_lbl, buf);
            } else {
                lv_label_set_text(wind_ang_lbl, "---");
            }
        } else {
            // TWA: -180..+180, P = Backbord, S = Steuerbord
            float twa = s_twa;
            if (twa >= -720.0f && twa <= 720.0f) {
                char buf[16];
                snprintf(buf, sizeof(buf), "%03.0f\xc2\xb0%c", fabsf(twa), twa < 0 ? 'P' : 'S');
                lv_label_set_text(wind_ang_lbl, buf);
            } else {
                lv_label_set_text(wind_ang_lbl, "---");
            }
        }
        lv_obj_set_style_text_color(wind_ang_lbl, wind_angle_color(s_ts_wind), 0);
    }

    // ── HDG A Screen (idx 5): SOG + Depth in den Boxen ──────────────────────
    if (lbl_main[5]) {
        char buf[16];
        if (s_sog >= 0) snprintf(buf, sizeof(buf), "%.1f", s_sog); else strcpy(buf, "---");
        lv_label_set_text(lbl_main[5], buf);
    }
    if (lbl_sub[5]) {
        char buf[16];
        if (s_depth >= 0) snprintf(buf, sizeof(buf), "%.1f", s_depth); else strcpy(buf, "---");
        lv_label_set_text(lbl_sub[5], buf);
    }

    // ── HDG D Screen (idx 6): Kurs-Zahl ─────────────────────────────────────
    if (fabsf(s_heading - last_hdg) > 0.5f) {
        last_hdg = s_heading;
        fmt_deg(lbl_main[6], s_heading);
    }
    // ✅ Farbe IMMER aktualisieren
    lv_obj_set_style_text_color(lbl_main[6], data_color(s_ts_hdg, 3000), 0);

    // Autopilot-Status als Untertitel
    if (lbl_sub[6]) {
        static int last_mode_d = -2;
        if (s_autopilot_mode != last_mode_d) {
            last_mode_d = s_autopilot_mode;
            const char* mode_str = "---";
            lv_color_t  text_col = lv_color_hex(0xCCCCCC);
            switch (s_autopilot_mode) {
                case 0x0000: mode_str = "STANDBY";  text_col = lv_color_hex(0xFF6666); break;
                case 0x0040: mode_str = "HDG";      text_col = lv_color_hex(0x00FFCC); break;
                case 0x0100: mode_str = "WIND";     text_col = lv_color_hex(0x0088CC); break;
                case 0x0180: mode_str = "TRACK";    text_col = lv_color_hex(0x00FF88); break;
                case 0x0181: mode_str = "WAYPOINT"; text_col = lv_color_hex(0x00CCFF); break;
            }
            lv_obj_set_style_text_font(lbl_sub[6], &montserrat_48, 0);
            lv_label_set_text(lbl_sub[6], mode_str);
            lv_obj_set_style_text_color(lbl_sub[6], text_col, 0);
        }
    }

    // ── POS Screen (idx 7): GPS-Koordinaten ─────────────────────────────────
    fmt_pos(lbl_main[7], s_lat, true);
    fmt_pos(lbl_sub[7],  s_lon, false);

    // ── Pilot Screen (idx 9) ─────────────────────────────────────────────────
    if (pilot_status_lbl) {
        lv_color_t col = ap_mode_color((uint16_t)s_autopilot_mode);

        const char* txt = "STBY";
        if      (s_autopilot_mode == 0x0040)                                txt = "AUTO";
        else if (s_autopilot_mode == 0x0100)                                txt = "WIND";
        else if (s_autopilot_mode == 0x0180 || s_autopilot_mode == 0x0181) txt = "TRCK";

        lv_label_set_text(pilot_status_lbl, txt);
        lv_obj_set_style_text_color(pilot_status_lbl, lv_color_hex(0xFFFFFF), 0);

        if (pilot_statusbox) {
            lv_obj_set_style_bg_color(pilot_statusbox, col, 0);
            lv_obj_set_style_border_color(pilot_statusbox,
                lv_color_mix(col, lv_color_hex(0xFFFFFF), 80), 0);
        }

        bool is_trck = (s_autopilot_mode == 0x0180 || s_autopilot_mode == 0x0181);

        if (pilot_hdg_lbl) {
            if (is_trck) {
                // ── TRCK: "TRCK" groß zentriert oben, darunter WPT-Name + DTW/TTG ──
                lv_obj_add_flag(pilot_hdg_lbl, LV_OBJ_FLAG_HIDDEN);
                // pilot_status_lbl zeigt "TRCK" – groß zentriert wie AUTO/STBY/WND
                lv_obj_set_style_text_font(pilot_status_lbl, &montserrat_72, 0);
                lv_obj_set_style_text_color(pilot_status_lbl, lv_color_hex(0xFFFFFF), 0);
                lv_obj_align(pilot_status_lbl, LV_ALIGN_TOP_MID, 0, 10);
                lv_obj_clear_flag(pilot_status_lbl, LV_OBJ_FLAG_HIDDEN);
                // WPT-Name etwas kleiner darunter (montserrat_48 statt 72 → passt in Box)
                lv_obj_set_style_text_font(pilot_wpt_lbl, &montserrat_48, 0);
                lv_obj_align(pilot_wpt_lbl, LV_ALIGN_TOP_MID, 0, 90);
                lv_obj_clear_flag(pilot_wpt_lbl,  LV_OBJ_FLAG_HIDDEN);
                lv_obj_clear_flag(pilot_wpt_info, LV_OBJ_FLAG_HIDDEN);

                lv_label_set_text(pilot_wpt_lbl, shorten_wpt_name(s_wpt_name));

                char info[32];
                if (s_dtw >= 0 && s_ttg >= 0) {
                    int h = (int)(s_ttg / 60);
                    int m = (int)s_ttg % 60;
                    if (h > 0) snprintf(info, sizeof(info), "%.1f NM  %dh%02dm", s_dtw, h, m);
                    else        snprintf(info, sizeof(info), "%.2f NM  %d min",   s_dtw, m);
                } else if (s_dtw >= 0) {
                    snprintf(info, sizeof(info), "%.2f NM", s_dtw);
                } else {
                    snprintf(info, sizeof(info), "warten...");
                }
                lv_label_set_text(pilot_wpt_info, info);

            } else {
                // ── AUTO / WIND / STBY: großes Heading- oder Windwinkel-Label ──
                lv_obj_clear_flag(pilot_hdg_lbl,  LV_OBJ_FLAG_HIDDEN);
                lv_obj_clear_flag(pilot_status_lbl, LV_OBJ_FLAG_HIDDEN);
                lv_obj_set_style_text_font(pilot_status_lbl, &montserrat_72, 0);
                lv_obj_align(pilot_status_lbl, LV_ALIGN_TOP_MID, 0, 10);
                lv_obj_add_flag(pilot_wpt_lbl,  LV_OBJ_FLAG_HIDDEN);
                lv_obj_add_flag(pilot_wpt_info, LV_OBJ_FLAG_HIDDEN);

                if (s_autopilot_mode == 0x0100) {
                    // WIND mode: gesperrten AWA anzeigen
                    if (ap_locked_wind != 0.0f) {
                        char buf[16];
                        snprintf(buf, sizeof(buf), "%.0f\xc2\xb0", ap_locked_wind);
                        lv_label_set_text(pilot_hdg_lbl, buf);
                    } else {
                        lv_label_set_text(pilot_hdg_lbl, "---");
                    }
                } else {
                    // AUTO / STBY: aktuelles Heading
                    if (s_heading >= 0.0f) {
                        char buf[16];
                        snprintf(buf, sizeof(buf), "%03.0f", fmodf(s_heading, 360.0f));
                        lv_label_set_text(pilot_hdg_lbl, buf);
                    } else {
                        lv_label_set_text(pilot_hdg_lbl, "---");
                    }
                }
            }
        }
    }

    // ── Multi-Instrument Screen (idx 10) ─────────────────────────────────────
    if (multi_lbl[0]) {
        char buf[16];

        // [0] SOG
        if (s_sog >= 0) snprintf(buf, sizeof(buf), "%.1f", s_sog); else strcpy(buf, "---");
        lv_label_set_text(multi_lbl[0], buf);
        lv_obj_set_style_text_color(multi_lbl[0], data_color(s_ts_sog), 0);

        // [1] HDG
        if (s_heading >= 0) snprintf(buf, sizeof(buf), "%d", (int)roundf(s_heading));
        else                 strcpy(buf, "---");
        lv_label_set_text(multi_lbl[1], buf);
        lv_obj_set_style_text_color(multi_lbl[1], data_color(s_ts_hdg, 3000), 0);

        // [2] AWS
        if (s_wind_spd >= 0) snprintf(buf, sizeof(buf), "%.1f", s_wind_spd);
        else                 strcpy(buf, "---");
        lv_label_set_text(multi_lbl[2], buf);
        lv_obj_set_style_text_color(multi_lbl[2], data_color(s_ts_wind), 0);

        // [3] AWA — intern 0..360, anzeigen als 0..180
        if (s_wind_ang >= 0) {
            int awa = (int)roundf(s_wind_ang);
            if (awa > 180) awa = 360 - awa;
            snprintf(buf, sizeof(buf), "%d", awa);
        } else { strcpy(buf, "---"); }
        lv_label_set_text(multi_lbl[3], buf);
        lv_obj_set_style_text_color(multi_lbl[3], data_color(s_ts_wind), 0);

        // [4] DEPTH
        if (s_depth >= 0) snprintf(buf, sizeof(buf), "%.1f", s_depth);
        else               strcpy(buf, "---");
        lv_label_set_text(multi_lbl[4], buf);
        lv_obj_set_style_text_color(multi_lbl[4], data_color(s_ts_depth), 0);

        // [5] TWS
        if (s_tws >= 0) snprintf(buf, sizeof(buf), "%.1f", s_tws);
        else             strcpy(buf, "---");
        lv_label_set_text(multi_lbl[5], buf);
        lv_obj_set_style_text_color(multi_lbl[5], data_color(s_ts_wind), 0);
    }
}
//===============================================================================
//          WATCHDOG GLOBAL

void watchdog_task(void* pv) {
    vTaskDelay(pdMS_TO_TICKS(30000));  // 30s Startgnadenfrist abwarten

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(5000));
        uint32_t now = millis();

        // 1. LVGL eingefroren? (update_displays() schreibt wdg_lvgl_ms)
        if (wdg_lvgl_ms > 0 && now - wdg_lvgl_ms > 15000) {
            Serial.printf("WDG: LVGL frozen %lums — restarting\n", now - wdg_lvgl_ms);
            Serial.flush();
            esp_restart();
        }

        // 2. NMEA-Task Stack-Overflow
        if (nmea_task_handle) {
            UBaseType_t stack_left = uxTaskGetStackHighWaterMark(nmea_task_handle);
            if (stack_left < 256) {
                Serial.printf("WDG: NMEA stack critical %u words — restarting\n", stack_left);
                Serial.flush();
                esp_restart();
            }
        }

        // 3. Heap-Alarm — nur warnen, kein Restart
        uint32_t free_heap = ESP.getFreeHeap();
        if (free_heap < 15000) {
            Serial.printf("WDG: Low heap %lu bytes\n", free_heap);
        }

        // NMEA-Ausfall: NUR loggen, niemals restarten
        if (last_nmea_msg_ms > 0 && now - last_nmea_msg_ms > 8000) {
            // Stille Anzeige via nmea_dot reicht völlig aus
            // Serial.printf("WDG: NMEA silent %lums\n", now - last_nmea_msg_ms);
        }
    }
}
// ═══════════════════════════════════════════════════════════════════════════════
// Setup & Loop
// ═══════════════════════════════════════════════════════════════════════════════
void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n=== Marine MFD Start ===");

	gfx->begin();
    
    // --- DEINE BACKLIGHT LOGIK ---
    //gfx->setBacklight(true);
    ledcAttach(BL_PIN, BL_FREQ, BL_BITS);
    ledcWrite(BL_PIN, 255);
	
    Wire.begin(TP_SDA, TP_SCL);
    touch.begin();
    
    lvgl_init();

    // NMEA Status-Dot — liegt permanent über allen Screens
    nmea_dot = lv_obj_create(lv_layer_top());
    lv_obj_set_size(nmea_dot, 18, 18);
    lv_obj_align(nmea_dot, LV_ALIGN_TOP_RIGHT, -3, 3);
    lv_obj_set_style_radius(nmea_dot, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_border_width(nmea_dot, 2, 0);
    lv_obj_set_style_border_color(nmea_dot, lv_color_hex(0x000000), 0);
    lv_obj_set_style_bg_color(nmea_dot, lv_color_hex(0x444444), 0);
    lv_obj_clear_flag(nmea_dot, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);
    
    lv_timer_create(wind_needle_anim_cb, 80, NULL); //war 30
    lv_timer_create(heading_anim_cb,     50, NULL);// war 30 

    screens[1] = build_instrument_screen(1, "DEPTH",   "m",   "---");
    screens[2] = build_instrument_screen(2, "SOG",     "kn",  "---");
    screens[3] = build_instrument_screen(3, "STW",     "kn",  "---");
    screens[4] = build_wind_screen();

    screens[5] = build_heading_analog_screen();

    screens[6] = build_instrument_screen(6, "HEADING", "deg", "---");  // "deg" nicht "°" !
    // lbl_main[6] bleibt bei montserrat_240_bold_numeric — NICHTS überschreiben!
    lv_obj_align(lbl_main[6], LV_ALIGN_CENTER, 0, -20);          // leicht nach oben
    lv_obj_set_style_text_font(lbl_sub[6], &montserrat_48, 0);
    lv_obj_align(lbl_sub[6], LV_ALIGN_BOTTOM_LEFT, 12, -60);

       // POS Screen
    screens[7] = build_instrument_screen(7, "POS", "", "---");
    lv_obj_set_style_text_font(lbl_main[7], &montserrat_72, 0);   // Lat/Lon kleiner
    lv_obj_align(lbl_main[7], LV_ALIGN_CENTER, 0, -40);
    lv_obj_set_style_text_font(lbl_sub[7], &montserrat_72, 0);
    lv_obj_align(lbl_sub[7], LV_ALIGN_CENTER, 0, 40);

    screens[8] = build_setup_screen();
    screens[9] = build_pilot_screen();
    screens[10] = build_multi_screen();
    screens[0] = build_home_screen();



    lv_timer_create([](lv_timer_t*){ update_displays(); }, 750, NULL); // war 500
    navigate_to(0);
 
    nmea_init();
    last_nmea_msg_ms = millis();   // ← direkt nach nmea_init()

        // ← DIESE ZEILE FEHLT KOMPLETT:
    nmea_send_queue = xQueueCreate(8, sizeof(NmeaSendMsg));


    //================================================================
    //================================================================
        // nmea_task wie gehabt auf Core 0:
    xTaskCreatePinnedToCore(nmea_task, "NMEA2000", 12288, NULL, 5,
                            &nmea_task_handle, 0);

    // Watchdog auf Core 1 (wo auch loop() läuft):
    xTaskCreatePinnedToCore(watchdog_task, "WDG", 2048, NULL, 2,
                            NULL, 1);
    //================================================================
    //================================================================

}

void loop() {
    // Deferred Swipe-Navigation — MUSS vor lv_timer_handler() stehen!
    if (swipe_nav_pending != -1) {
        int8_t nav = swipe_nav_pending;
        swipe_nav_pending = -1;              // Flag sofort löschen
        if      (nav == -2) navigate_relative(-1);
        else if (nav == -3) navigate_relative(+1);
        else                navigate_to(nav);
    }
    lv_timer_handler();
    delay(5);
}
