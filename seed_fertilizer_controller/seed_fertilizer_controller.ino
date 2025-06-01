#define LV_CONF_INCLUDE_SIMPLE
#include <Arduino.h>
#include <Waveshare_ST7262_LVGL.h>
#include <lvgl.h>
#include <Preferences.h>
#include <time.h>

#define BUZZER_PIN 4 // Use 4 for Arduino pin

// --- Variables ---
lv_obj_t *seed_rate_input, *fert_rate_input, *flow_label;
lv_obj_t *seed_bar, *fert_bar, *alert_label;
float grams_per_rev = 22.5;
float gpr_fert = 19.8;
float gps_speed_kmph = 10.0;
float target_rate = 30.0;
float seed_level_mm = 150.0;
float fert_level_mm = 160.0;
float seed_threshold = 30.0;
float fert_threshold = 30.0;
bool buzzer_enabled = true;

const float CALIBRATION_REVOLUTIONS = 35.0f;
const float METERS_PER_DRIVE_REVOLUTION = 0.2f; // Assumption: this 0.2 is meters per some revolution/pulse related to ground speed

void update_motor_rate() {
  const char* target_rate_str = lv_textarea_get_text(seed_rate_input);
  target_rate = atof(target_rate_str);
  if (target_rate < 2.0 || target_rate > 400.0) {
      lv_obj_t *mbox1 = lv_msgbox_create(NULL);
      lv_msgbox_set_title(mbox1, "Input Error");
      lv_msgbox_set_text(mbox1, "Target rate must be between 2.0 and 400.0 kg/ha.");
      lv_msgbox_add_close_button(mbox1);
      return;
  }
  if (grams_per_rev < 0.01) {
    lv_obj_t *mbox2 = lv_msgbox_create(NULL);
    lv_msgbox_set_title(mbox2, "Seed Calibration");
    lv_msgbox_set_text(mbox2, "Missing or invalid calibration");
    lv_msgbox_add_close_button(mbox2);
    return;
  }
  if (gpr_fert < 0.01) {
    lv_obj_t *mbox3 = lv_msgbox_create(NULL);
    lv_msgbox_set_title(mbox3, "Fert Calibration");
    lv_msgbox_set_text(mbox3, "Missing or invalid calibration");
    lv_msgbox_add_close_button(mbox3);
    return;
  }

  float revs_per_sec = (gps_speed_kmph * 1000.0 / 3600.0) / METERS_PER_DRIVE_REVOLUTION;
  float grams_per_sec = grams_per_rev * revs_per_sec;
  char buf[64];
  snprintf(buf, sizeof(buf), "Flow: %.1f g/s", grams_per_sec);
  lv_label_set_text(flow_label, buf);
  lv_bar_set_value(seed_bar, seed_level_mm, LV_ANIM_ON);
  lv_bar_set_value(fert_bar, fert_level_mm, LV_ANIM_ON);

  if (seed_level_mm < seed_threshold || fert_level_mm < fert_threshold) {
    lv_label_set_text(alert_label, "Warning: Low Bin Level");
    if (buzzer_enabled) {
      digitalWrite(BUZZER_PIN, HIGH);
      delay(100); // Beep for 100ms
      digitalWrite(BUZZER_PIN, LOW);
    }
  } else {
    lv_label_set_text(alert_label, "");
  }
}

lv_obj_t *main_screen;
lv_obj_t *calib_screen;

void switch_to_calib_screen(lv_event_t *e) {
  lv_scr_load(calib_screen);
}

void switch_to_main_screen(lv_event_t *e) {
  lv_scr_load(main_screen);
}

void create_ui() {
  main_screen = lv_obj_create(NULL);
  lv_scr_load(main_screen);
  // Simulated GPS speed input
  lv_obj_t *gps_input = lv_textarea_create(lv_scr_act());
  lv_obj_align(gps_input, LV_ALIGN_TOP_LEFT, 10, 140);
  lv_textarea_set_placeholder_text(gps_input, "GPS Speed (km/h)");
  lv_textarea_set_text(gps_input, String(gps_speed_kmph).c_str());
  lv_obj_add_event_cb(gps_input, [](lv_event_t *e) {
    lv_obj_t *ta = lv_event_get_target(e);
    gps_speed_kmph = atof(lv_textarea_get_text(ta));
  }, LV_EVENT_VALUE_CHANGED, NULL);

  lv_obj_t *scr = main_screen;

  // Seed rate input
  seed_rate_input = lv_textarea_create(scr);
  lv_obj_align(seed_rate_input, LV_ALIGN_TOP_LEFT, 10, 10);
  lv_textarea_set_placeholder_text(seed_rate_input, "Seed Rate (kg/ha)");
  lv_textarea_set_text(seed_rate_input, "30.0");

  // Fert rate input
  fert_rate_input = lv_textarea_create(scr);
  lv_obj_align(fert_rate_input, LV_ALIGN_TOP_LEFT, 10, 50);
  lv_textarea_set_placeholder_text(fert_rate_input, "Fert Rate (kg/ha)");
  lv_textarea_set_text(fert_rate_input, "30.0");

  // Update button
  lv_obj_t *btn = lv_btn_create(scr);
  lv_obj_align(btn, LV_ALIGN_TOP_LEFT, 10, 100);
  lv_obj_t *lbl = lv_label_create(btn);
  lv_label_set_text(lbl, "Update Rate");
  lv_obj_center(lbl);
  lv_obj_add_event_cb(btn, [](lv_event_t *) { update_motor_rate(); }, LV_EVENT_CLICKED, NULL);

  // Calibration screen button
  lv_obj_t *calib_nav_btn = lv_btn_create(scr);
  lv_obj_align(calib_nav_btn, LV_ALIGN_TOP_LEFT, 160, 100);
  lv_obj_t *calib_nav_lbl = lv_label_create(calib_nav_btn);
  lv_label_set_text(calib_nav_lbl, "Full Calibration");
  lv_obj_center(calib_nav_lbl);
  lv_obj_add_event_cb(calib_nav_btn, switch_to_calib_screen, LV_EVENT_CLICKED, NULL);

  // Flow display
  flow_label = lv_label_create(scr);
  lv_obj_align(flow_label, LV_ALIGN_BOTTOM_MID, 0, -30);
  lv_label_set_text(flow_label, "Flow: --.- g/s");

  // Alert display
  alert_label = lv_label_create(scr);
  lv_obj_align(alert_label, LV_ALIGN_BOTTOM_MID, 0, -10);
  lv_label_set_text(alert_label, "");

  // Bin level bars
  seed_bar = lv_bar_create(scr);
  lv_obj_set_size(seed_bar, 20, 100);
  lv_obj_align(seed_bar, LV_ALIGN_TOP_RIGHT, -50, 10);
  lv_bar_set_range(seed_bar, 0, 300); // Assuming mm
  lv_bar_set_value(seed_bar, seed_level_mm, LV_ANIM_OFF);
  lv_obj_t *seed_bar_lbl = lv_label_create(scr);
  lv_label_set_text(seed_bar_lbl, "Seed");
  lv_obj_align(seed_bar_lbl, LV_ALIGN_OUT_BOTTOM_MID, 0, 5);

  fert_bar = lv_bar_create(scr);
  lv_obj_set_size(fert_bar, 20, 100);
  lv_obj_align(fert_bar, LV_ALIGN_TOP_RIGHT, -20, 10);
  lv_bar_set_range(fert_bar, 0, 300); // Assuming mm
  lv_bar_set_value(fert_bar, fert_level_mm, LV_ANIM_OFF);
  lv_obj_t *fert_bar_lbl = lv_label_create(scr);
  lv_label_set_text(fert_bar_lbl, "Fert");
  lv_obj_align(fert_bar_lbl, LV_ALIGN_OUT_BOTTOM_MID, 0, 5);
}

lv_obj_t *seed_time_label;
lv_obj_t *fert_time_label;

void create_calibration_screen() {
  calib_screen = lv_obj_create(NULL);

  lv_obj_t *back_btn = lv_btn_create(calib_screen);
  lv_obj_align(back_btn, LV_ALIGN_TOP_LEFT, 10, 10);
  lv_obj_t *back_lbl = lv_label_create(back_btn);
  lv_label_set_text(back_lbl, "Back");
  lv_obj_center(back_lbl);
  lv_obj_add_event_cb(back_btn, switch_to_main_screen, LV_EVENT_CLICKED, NULL);

  // Seed calibration
  lv_obj_t *seed_input = lv_textarea_create(calib_screen);
  lv_obj_align(seed_input, LV_ALIGN_TOP_LEFT, 10, 60);
  lv_textarea_set_placeholder_text(seed_input, "Seed Calib (g)");
  lv_textarea_set_text(seed_input, "0.0");

  lv_obj_t *seed_btn = lv_btn_create(calib_screen);
  lv_obj_align(seed_btn, LV_ALIGN_TOP_LEFT, 180, 60);
  lv_obj_t *seed_btn_lbl = lv_label_create(seed_btn);
  lv_label_set_text(seed_btn_lbl, "Save Seed");
  lv_obj_center(seed_btn_lbl);
  seed_time_label = lv_label_create(calib_screen);
  lv_obj_align(seed_time_label, LV_ALIGN_TOP_LEFT, 10, 100);
  lv_label_set_text(seed_time_label, "--/--/----");

  lv_obj_add_event_cb(seed_btn, [](lv_event_t *e) {
    lv_obj_t* seed_input_ta = (lv_obj_t*)lv_event_get_user_data(e);
    float weight = atof(lv_textarea_get_text(seed_input_ta));
    if (weight >= 1.0 && weight <= 20000.0) {
      grams_per_rev = weight / CALIBRATION_REVOLUTIONS;
      Preferences prefs;
      prefs.begin("calib", false);
      prefs.putFloat("gpr_seed", grams_per_rev);
      time_t seed_time = time(NULL);
      prefs.putLong64("seed_time", seed_time);
      prefs.end();
      char date_buf[16];
      strftime(date_buf, sizeof(date_buf), "%d/%m/%Y", localtime(&seed_time));
      lv_label_set_text(seed_time_label, date_buf);
      lv_obj_t *mbox_seed_saved = lv_msgbox_create(NULL);
      lv_msgbox_set_title(mbox_seed_saved, "Saved");
      lv_msgbox_set_text(mbox_seed_saved, "Seed Calibrated & Saved!");
      lv_msgbox_add_close_button(mbox_seed_saved);
    } else {
      lv_obj_t *mbox_seed_error = lv_msgbox_create(NULL);
      lv_msgbox_set_title(mbox_seed_error, "Error");
      lv_msgbox_set_text(mbox_seed_error, "Invalid weight (1-20000g)");
      lv_msgbox_add_close_button(mbox_seed_error);
    }
  }, LV_EVENT_CLICKED, seed_input);

  // Fert calibration
  lv_obj_t *fert_input = lv_textarea_create(calib_screen);
  lv_obj_align(fert_input, LV_ALIGN_TOP_LEFT, 10, 120);
  lv_textarea_set_placeholder_text(fert_input, "Fert Calib (g)");
  lv_textarea_set_text(fert_input, "0.0");

  lv_obj_t *fert_btn = lv_btn_create(calib_screen);
  lv_obj_align(fert_btn, LV_ALIGN_TOP_LEFT, 180, 120);
  lv_obj_t *fert_btn_lbl = lv_label_create(fert_btn);
  lv_label_set_text(fert_btn_lbl, "Save Fert");
  lv_obj_center(fert_btn_lbl);
  fert_time_label = lv_label_create(calib_screen);
  lv_obj_align(fert_time_label, LV_ALIGN_TOP_LEFT, 10, 160);
  lv_label_set_text(fert_time_label, "--/--/----");

  lv_obj_add_event_cb(fert_btn, [](lv_event_t *e) {
    lv_obj_t* fert_input_ta = (lv_obj_t*)lv_event_get_user_data(e);
    float weight = atof(lv_textarea_get_text(fert_input_ta));
    if (weight >= 1.0 && weight <= 20000.0) {
      gpr_fert = weight / CALIBRATION_REVOLUTIONS;
      Preferences prefs;
      prefs.begin("calib", false);
      prefs.putFloat("gpr_fert", gpr_fert);
      time_t fert_time = time(NULL);
      prefs.putLong64("fert_time", fert_time);
      prefs.end();
      char date_buf[16];
      strftime(date_buf, sizeof(date_buf), "%d/%m/%Y", localtime(&fert_time));
      lv_label_set_text(fert_time_label, date_buf);
      lv_obj_t *mbox_fert_saved = lv_msgbox_create(NULL);
      lv_msgbox_set_title(mbox_fert_saved, "Saved");
      lv_msgbox_set_text(mbox_fert_saved, "Fertiliser Calibrated & Saved!");
      lv_msgbox_add_close_button(mbox_fert_saved);
    } else {
      lv_obj_t *mbox_fert_error = lv_msgbox_create(NULL);
      lv_msgbox_set_title(mbox_fert_error, "Error");
      lv_msgbox_set_text(mbox_fert_error, "Invalid weight (1-20000g)");
      lv_msgbox_add_close_button(mbox_fert_error);
    }
  }, LV_EVENT_CLICKED, fert_input);
}

void setup() {
  // IMPORTANT: For accurate date/time stamping, ensure ESP32 system time is synchronized (e.g., via NTP or a hardware RTC).
  // Serial.begin(115200);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  // Uncomment if supported by your library:
  // Waveshare_ST7262::begin();
  // lvgl_setup();

  Preferences prefs;
  prefs.begin("calib", true);
  grams_per_rev = prefs.getFloat("gpr_seed", grams_per_rev);
  gpr_fert = prefs.getFloat("gpr_fert", gpr_fert);
  time_t seed_ts = prefs.getLong64("seed_time", 0);
  time_t fert_ts = prefs.getLong64("fert_time", 0);
  prefs.end();

  create_calibration_screen();
  create_ui();

  if (seed_ts > 0 && seed_time_label) {
    char date_buf[16];
    strftime(date_buf, sizeof(date_buf), "%d/%m/%Y", localtime(&seed_ts));
    lv_label_set_text(seed_time_label, date_buf);
  }
  if (fert_ts > 0 && fert_time_label) {
    char date_buf[16];
    strftime(date_buf, sizeof(date_buf), "%d/%m/%Y", localtime(&fert_ts));
    lv_label_set_text(fert_time_label, date_buf);
  }
}

void loop() {
  lv_timer_handler();
  delay(5);
}