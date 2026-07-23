
//==========================================================================================

void start_display() {
  tft.begin();
  if (!ts.begin()) {
    DEBUG_PRINTLN("Couldn't start touchscreen controller");
    while (1)
      ;
  }
  tft.setRotation(2);
  show_time = true;
  DEBUG_PRINTLN("Touchscreen started");
}

//==========================================================================================

void WelcomeScreen() {
  tft.fillScreen(THEME_BG);

  // thin accent rules top and bottom to frame the mark
  tft.fillRect(0, 0, 240, 4, THEME_ACCENT);
  tft.fillRect(0, 316, 240, 4, THEME_ACCENT);

  drawBitmap(0, 60, welcome_logo, 240, 173, THEME_TEXT);

  // firmware build line under the mark
  tft.setFont();
  tft.setTextSize(1);
  tft.setTextColor(THEME_MUTED);
  NickText_center(F(FIRMWARE_VERSION), 120, 262, 0);

  wait(2000);
}

//==========================================================================================

void set_brightness() {
  if (force_full_brightness) {
    analogWrite(5, screen_brightness_day);
    return;
  }

  now = rtc.now();
  hour_now = now.hour();
  if (lights_on < lights_off) {
    if (hour_now >= lights_on && hour_now < lights_off) analogWrite(5, screen_brightness_day);
    if (hour_now < lights_on || hour_now >= lights_off) analogWrite(5, screen_brightness_night);
  }
  if (lights_on > lights_off) {
    if (hour_now >= lights_on || hour_now < lights_off) analogWrite(5, screen_brightness_day);
    if (hour_now < lights_on && hour_now >= lights_off) analogWrite(5, screen_brightness_night);
  }
}
//======================================================================================

void NickText(String text, int x, int y, int size) {
  tft.setTextSize(size);
  tft.setCursor(x, y);
  tft.print(text);
}

//======================================================================================

void NickText(int w, int x, int y, int size) {
  tft.setTextSize(size);
  tft.setCursor(x, y);
  tft.print(w);
}

//======================================================================================

void NickText_center(String text, int center_x, int center_y, int size) {
  tft.setTextSize(size);
  int16_t x1;
  int16_t y1;
  uint16_t w;
  uint16_t h;
  tft.getTextBounds(text, 0, 320, &x1, &y1, &w, &h);
  int xx = (center_x - 1) - (w / 2);
  int yy = (center_y - 1) + (h / 2);
  tft.setCursor(xx, yy);
  tft.print(text);
}


//======================================================================================

void NickText_center(int variable, int center_x, int center_y, int size) {
  tft.setTextSize(size);
  int16_t x1;
  int16_t y1;
  uint16_t w;
  uint16_t h;
  String text = String(variable);
  tft.getTextBounds(text, 0, 320, &x1, &y1, &w, &h);
  int xx = (center_x - 1) - (w / 2);
  int yy = (center_y - 1) + (h / 2);
  tft.setCursor(xx, yy);
  tft.print(variable);
}

//==========================================================================================

void Button_center(int center_x, int center_y, int sizeX, int sizeY, uint16_t button_color, uint16_t border_color, uint16_t text_color, String text, int textsize) {
  int16_t x = center_x - (sizeX / 2);
  int16_t y = center_y - (sizeY / 2);
  tft.fillRoundRect(x, y, sizeX, sizeY, 4, button_color);
  tft.drawRoundRect(x, y, sizeX, sizeY, 4, border_color);
  int16_t x1;
  int16_t y1;
  uint16_t w;
  uint16_t h;
  tft.getTextBounds(text, 0, 320, &x1, &y1, &w, &h);
  int xx = (center_x - 1) - (w / 2);
  int yy = (center_y - 1) + (h / 2);
  tft.setTextColor(text_color);
  tft.setTextSize(textsize);
  tft.setCursor(xx, yy);
  tft.print(text);

  int x_min;
  int x_max;
  int y_min;
  int y_max;
  x_min = x;
  x_max = x + sizeX;
  y_min = y;
  y_max = y + sizeY;
  DEBUG_PRINTLN();
  DEBUG_PRINT(text);
  DEBUG_PRINT(" button borders: p.x>");
  DEBUG_PRINT(x_min);
  DEBUG_PRINT(" && p.x<");
  DEBUG_PRINT(x_max);
  DEBUG_PRINT(" && p.y>");
  DEBUG_PRINT(y_min);
  DEBUG_PRINT(" && p.y<");
  DEBUG_PRINT(y_max);
  DEBUG_PRINT("");
}

//==========================================================================================

void Button_center(int center_x, int center_y, int sizeX, int sizeY, uint16_t button_color, uint16_t border_color, uint16_t text_color, int text, int textsize) {
  int16_t x = center_x - (sizeX / 2);
  int16_t y = center_y - (sizeY / 2);
  tft.fillRoundRect(x, y, sizeX, sizeY, 4, button_color);
  tft.drawRoundRect(x, y, sizeX, sizeY, 4, border_color);
  int16_t x1;
  int16_t y1;
  uint16_t w;
  uint16_t h;
  tft.getTextBounds(String(text), 0, 320, &x1, &y1, &w, &h);
  int xx = (center_x - 1) - (w / 2);
  int yy = (center_y - 1) + (h / 2);
  tft.setTextColor(text_color);
  tft.setTextSize(textsize);
  tft.setCursor(xx, yy);
  tft.print(text);

  int x_min;
  int x_max;
  int y_min;
  int y_max;
  x_min = x;
  x_max = x + sizeX;
  y_min = y;
  y_max = y + sizeY;
  DEBUG_PRINTLN();
  DEBUG_PRINT(text);
  DEBUG_PRINT(" button borders: p.x>");
  DEBUG_PRINT(x_min);
  DEBUG_PRINT(" && p.x<");
  DEBUG_PRINT(x_max);
  DEBUG_PRINT(" && p.y>");
  DEBUG_PRINT(y_min);
  DEBUG_PRINT(" && p.y<");
  DEBUG_PRINT(y_max);
  DEBUG_PRINT("");
}

//==========================================================================================

void Button(int x, int y, int sizeX, int sizeY, int rad, uint16_t color) {
  tft.fillRoundRect(x, y, sizeX, sizeY, rad, color);
  tft.drawRoundRect(x, y, sizeX, sizeY, rad, THEME_SURFACE);
}

//======================================================================================

void error(String str) {
  show_time = true;
  display_time();
  tft.fillRect(0, 20, 240, 320, THEME_BG);
  DEBUG_PRINT("error: ");
  DEBUG_PRINTLN(str);
  tft.setFont();
  tft.setTextWrap(true);
  tft.setFont(&FreeSans9pt7b);
  NickText(str, 5, 50, 1);
  Button_center(120, 250, 220, 60, THEME_SURFACE, THEME_SURFACE, THEME_TEXT, "Return to Main Menu", 1);
  display_page = "error";
  refresh_page = true;
}

//======================================================================================

void display_licks() {
  tft.fillRect(0, 20, 240, 300, THEME_BG);

  // ---- title block ----
  tft.setFont(&FreeSansBold9pt7b);
  tft.setTextColor(THEME_TEXT);
  NickText_center(F(LAB_NAME), 120, 40, 1);

  tft.setFont();
  tft.setTextSize(1);
  tft.setTextColor(THEME_MUTED);
  NickText_center(F("LICKS  -  CURRENT SESSION"), 120, 54, 0);

  // ---- refresh button, top right ----
  Button_center(212, 40, 48, 22, THEME_SURFACE, THEME_ACCENT, THEME_TEXT, "", 1);
  tft.setFont();
  tft.setTextSize(1);
  tft.setTextColor(THEME_TEXT);
  NickText_center(F("REFRESH"), 212, 37, 0);

  unsigned long leftShown = total_LN[0];
  unsigned long rightShown = total_LN[1];
  if (display_live_current_bin) {
    leftShown += LickNumber[0];
    rightShown += LickNumber[1];
  }

  draw_bottle_card(12,  70, F("LEFT"),  leftShown,  0, THEME_LEFT);
  draw_bottle_card(12, 168, F("RIGHT"), rightShown, 1, THEME_RIGHT);

  // ---- status strip ----
  tft.fillRect(0, 268, 240, 22, THEME_SURFACE);
  tft.setFont();
  tft.setTextSize(1);
  tft.setTextColor(THEME_MUTED, THEME_SURFACE);

  tft.setCursor(8, 276);
  tft.print(F("bin "));
  tft.print(LOG_INTERVAL);
  tft.print(F(" min"));

  tft.setCursor(92, 276);
  tft.print(F("thr "));
  tft.print(touch_threshold);
  tft.print(F("/"));
  tft.print(release_threshold);

  tft.setCursor(168, 276);
  tft.print(F("TO "));
  tft.print(timeouts);
}

//==========================================================================================
//  One bottle panel: label, live contact dot, big count, and a bout indicator.
//==========================================================================================

void draw_bottle_card(int x, int y, const __FlashStringHelper *label,
                      unsigned long count, uint8_t ch, uint16_t tint) {
  const int w = 216, h = 88;

  tft.fillRoundRect(x, y, w, h, 6, THEME_SURFACE);
  // coloured spine down the left edge identifies the bottle at a glance
  tft.fillRect(x, y + 6, 4, h - 12, colour_code_bottles ? tint : THEME_ACCENT);

  tft.setFont(&FreeSansBold9pt7b);
  tft.setTextColor(colour_code_bottles ? tint : THEME_TEXT);
  tft.setCursor(x + 14, y + 22);
  tft.print(label);

  // live contact dot - filled while the tongue is actually on the spout
  bool touching = (lk_state[ch] == LK_TOUCH);
  int dx = x + w - 20, dy = y + 17;
  tft.fillCircle(dx, dy, 6, THEME_SURFACE);
  if (touching) tft.fillCircle(dx, dy, 6, THEME_ACCENT);
  else tft.drawCircle(dx, dy, 6, THEME_MUTED);

  // the count, right aligned so digits do not jump around
  tft.setFont(&FreeSansBold24pt7b);
  tft.setTextColor(THEME_TEXT);
  int16_t bx, by;
  uint16_t bw, bh;
  String num = String(count);
  tft.getTextBounds(num, 0, 0, &bx, &by, &bw, &bh);
  tft.setCursor(x + w - 16 - bw, y + h - 16);
  tft.print(num);

  tft.setFont();
  tft.setTextSize(1);
  tft.setTextColor(THEME_MUTED);
  tft.setCursor(x + 14, y + h - 22);
  tft.print(F("licks"));

  if (in_bout[ch]) {
    tft.setTextColor(THEME_ACCENT);
    tft.setCursor(x + 14, y + h - 12);
    tft.print(F("IN BOUT"));
  }
}

//==========================================================================================
void display_time() {
  if (show_time) {
    show_time = false;

    now = rtc.now();
    min_now = now.minute();
    tft.setFont(&FreeSansBold9pt7b);
    tft.setTextSize(1);
    tft.fillRect(0, 0, 240, 20, THEME_SURFACE);
    tft.setCursor(0, 14);
    tft.setTextColor(THEME_TEXT, THEME_SURFACE);
    tft.print(now.month());
    tft.print("/");
    tft.print(now.day());
    tft.print("/");
    tft.print(now.year());
    tft.setCursor(190, 14);
    tft.print(now.hour());
    tft.print(":");
    if (now.minute() < 10) {
      tft.print("0");
    }
    tft.print(now.minute());
  }

  if (cached_minute() != min_now) {
    show_time = true;  //refresh page every minute
  }
}


//==========================================================================================

void drawBitmap(int16_t x, int16_t y, const uint8_t *bitmap, int16_t w, int16_t h, uint16_t color) {
  int16_t i, j, byteWidth = (w + 7) / 8;
  uint8_t byte;
  for (j = 0; j < h; j++) {
    for (i = 0; i < w; i++) {
      if (i & 7) byte <<= 1;
      else byte = pgm_read_byte(bitmap + j * byteWidth + i / 8);
      if (byte & 0x80) tft.drawPixel(x + i, y + j, color);
    }
  }
}

//======================================================================================

void GetNum(String prompt, int &Number) {
  // print a numeric keyboard and a prompt message
  // and get a numeric integer
  tft.fillScreen(THEME_BG);
  draw_BoxNButtons();  // draw the virtual keyboard
  tft.setCursor(10, 35);
  tft.setTextSize(1);
  tft.setTextColor(THEME_TEXT);
  tft.setFont(&FreeSansBold12pt7b);
  tft.println(prompt);  //print prompt
  tft.setFont(&FreeSansBold24pt7b);
  tft.setCursor(10, 110);
  tft.setTextSize(1);
  tft.setTextColor(THEME_TEXT);
  tft.println(Number);  //update new value

  while (!ok) {  // repeat until ok is presse
    if (ts.touched()) {
      TS_Point p = ts.getPoint();
      wait(100);
      p.x = map(p.x, 0, 240, 0, 240);
      p.y = map(p.y, 0, 320, 0, 320);

      if (p.x < 60 && p.x > 0 && p.y > 260 && p.y < 320) {
        Number = 0;  //If Cancel button is pressed
        p = {};
      }
      if (p.x < 60 && p.x > 0 && p.y > 200 && p.y < 260) {
        Number = (Number * 10) + 5;  // button 1 is pressed
        p = {};
      }
      if (p.x < 60 && p.x > 0 && p.y > 140 && p.y < 200) {
        Number = (Number * 10) + 1;  // button 6 is pressed
        p = {};
      }

      if (p.x < 120 && p.x > 60 && p.y > 260 && p.y < 320) {
        Number = (Number * 10) + 9;  // button 0 is pressed
        p = {};
      }
      if (p.x < 120 && p.x > 60 && p.y > 200 && p.y < 260) {
        Number = (Number * 10) + 6;  // button 3 is pressed
        p = {};
      }
      if (p.x < 120 && p.x > 60 && p.y > 140 && p.y < 200) {
        Number = (Number * 10) + 2;  // button 7 is pressed
        p = {};
      }

      if (p.x < 180 && p.x > 120 && p.y > 260 && p.y < 320) {
        Number = (Number * 10) + 0;  // button 1 is pressed
        p = {};
      }
      if (p.x < 180 && p.x > 120 && p.y > 200 && p.y < 260) {
        Number = (Number * 10) + 7;  // button 4 is pressed
        p = {};
      }
      if (p.x < 180 && p.x > 120 && p.y > 140 && p.y < 200) {
        Number = (Number * 10) + 3;  // button 8 is pressed
        p = {};
      }

      if (p.x < 240 && p.x > 180 && p.y > 260 && p.y < 320) {
        ok = true;  // button ok is pressed
        p = {};
      }
      if (p.x < 240 && p.x > 180 && p.y > 200 && p.y < 260) {
        Number = (Number * 10) + 8;  // button 5 is pressed
        p = {};
      }
      if (p.x < 240 && p.x > 180 && p.y > 140 && p.y < 200) {
        Number = (Number * 10) + 4;  // button 9 is pressed
        p = {};
      }

      tft.fillRect(0, 60, 240, 80, THEME_BG);  //clear result box
      tft.setFont(&FreeSansBold24pt7b);
      tft.setCursor(10, 110);
      tft.setTextSize(1);
      tft.setTextColor(THEME_TEXT);
      tft.println(Number);  //update new value
    }
  }
}

//======================================================================================

void draw_BoxNButtons() {
  String symbol[3][4] = {
    { "1", "2", "3", "4" },
    { "5", "6", "7", "8" },
    { "C", "9", "0", "OK" }
  };
  //Draw the Result Box
  tft.fillRect(0, 60, 240, 80, THEME_BG);

  //Draw keys
  tft.fillRect(0, 260, 60, 60, THEME_BG);
  tft.fillRect(0, 140, 240, 120, THEME_SURFACE);
  tft.fillRect(60, 260, 120, 60, THEME_SURFACE);
  tft.fillRect(180, 260, 60, 60, THEME_ACCENT);

  //Draw Horizontal Lines
  for (int h = 140; h <= 320; h += 60)
    tft.drawFastHLine(0, h, 240, THEME_TEXT);

  //Draw Vertical Lines
  for (int v = 0; v <= 240; v += 60)
    tft.drawFastVLine(v, 140, 1800, THEME_TEXT);

  tft.setFont(&FreeSansBold18pt7b);

  //Display keypad lables
  for (int j = 0; j < 3; j++) {
    for (int i = 0; i < 4; i++) {
      if (j == 2 && i == 3) {
        tft.setTextColor(THEME_BG);  // dark text reads better on the accent colour
        tft.setCursor(5 + (60 * i), 183 + (60 * j));
        tft.println(symbol[j][i]);
        return;
      }
      tft.setCursor(20 + (60 * i), 183 + (60 * j));
      tft.setTextSize(1);
      tft.setTextColor(THEME_TEXT);
      tft.println(symbol[j][i]);
    }
  }
}
