/*
 * ArcheryTimer - M5StickC 用 アーチェリー行射タイマー (RS-485 出力)
 *
 * old_python/snd_data.py の --timer モードを M5StickC に移植したものです。
 * モルテン ハンディータイマーアウトドア UD0040 を RS-485 で駆動しつつ、
 * M5StickC の画面（横向き）にも同じ内容を信号色つきで表示します。
 *
 *   開始前 --[A]--> ムーブアップ 10秒(赤) --> 行射 180秒(緑 / 残り30秒で黄)
 *          --> (次の立 または 行射終了)
 *
 * 通信は仕様どおり固定:
 *   EIA RS-485準拠 / 半二重 / 調歩同期 / 9600bps / スタート1 / データ8 / パリティ無 / ストップ1
 *
 * 操作 (Python版のキー割り当てに対応):
 *   ボタンA 短押し  = Space  開始 / 再開
 *   ボタンA 長押し  = 右矢印 早送り (この立を終わりにして次へ)
 *   ボタンB 短押し  = Enter  中断 (5声)
 */

// M5StickC Plus を使う場合は include を入れ替えてください
#include <M5StickC.h>
// #include <M5StickCPlus.h>

// ============================================================ 設定
static const uint16_t MOVEUP_SEC   = 10;   // ムーブアップ（準備）時間 [秒]
static const uint16_t SHOOTING_SEC = 180;  // 行射時間 [秒]
static const uint16_t WARN_SEC     = 30;   // 行射残りこの秒数から黄色 [秒]
static const uint16_t REPEAT       = 1;    // 繰り返す「立」の数

// UD0040 のブザー音色: 1 = ブザー1 (0x01) / 2 = ブザー2 (0x00) / 0 = 鳴らさない
static const uint8_t  BUZZER_KIND_SEL = 1;

// M5StickC 本体側でも鳴らす場合のピン。-1 で鳴らさない。
//   M5StickC Plus : 内蔵ブザーが G2 にあるので 2 を指定する
//   無印 M5StickC : 内蔵ブザーは無い。SPK HAT などを繋いだピン (例: 26) を指定する
// UD0040 のブザーは、この設定とは無関係に BUZZER_KIND_SEL で鳴る。
static const int      BUZZER_PIN  = -1;
static const uint16_t BUZZER_FREQ = 2000;

static const uint8_t  SCREEN_ROTATION = 3;   // 3 または 1 で横向き（上下逆なら 1）
// 画面の明るさ。M5StickC ライブラリの ScreenBreath() は 7..12 で、12 が最大
// (それ以上を渡しても 12 に丸められる)。0..100 を取るライブラリを使っていて
// 画面が暗い場合は 100 にしてください。
static const uint8_t  SCREEN_BRIGHT   = 12;
static const uint32_t LONG_PRESS_MS   = 800; // ボタンA長押し（早送り）の判定時間

// RS-485 ユニットの接続先 (M5StickC の Grove 端子)。RX/TX が逆の場合は入れ替えてください。
static const int  RS485_RX_PIN = 33;
static const int  RS485_TX_PIN = 32;
static const long RS485_BAUD   = 9600;
#define RS485 Serial2

// ============================================================ プロトコル
// フレーム: STX(0x02) + 本体を16進にしたASCII文字列 + ETX(0x03) + BCC 2文字
//   BCC = XOR(STXの次 〜 ETXまで、ETXを含む) を2桁の大文字16進にしたもの
static const uint8_t STX = 0x02;
static const uint8_t ETX = 0x03;

static const uint8_t CMD_DISPLAY = 0x01;
static const uint8_t CMD_BUZZER  = 0x0D;
static const uint8_t CMD_INIT    = 0x0E;

static const uint8_t DIGITS         = 7;     // プロトコル上の桁数（実機が持つのは右5桁）
static const uint8_t SEPARATOR_MODE = 0x02;  // 本体[8] 区切り(コロン)を使う時間表示

static const uint32_t FRAME_INTERVAL_MS = 85;  // 実機の送信周期

static const char HEX_CHARS[] = "0123456789ABCDEF";

// 7セグメントの字形 (下位7ビット。0x80 が小数点)。アーチェリー用タイマーは
// 数字しか使わないので、数字と '-' だけを持つ。
static uint8_t segForChar(char c) {
  switch (c) {
    case '0': return 0x3F;
    case '1': return 0x06;
    case '2': return 0x5B;
    case '3': return 0x4F;
    case '4': return 0x66;
    case '5': return 0x6D;
    case '6': return 0x7D;
    case '7': return 0x07;
    case '8': return 0x7F;
    case '9': return 0x6F;
    case '-': return 0x40;
    default:  return 0x00;  // 消灯
  }
}

// 本体バイト列から、送出できる1フレームを組み立てる。
// 戻り値はフレーム長 (本体 n バイト -> 2n + 4 バイト)。
static size_t buildFrame(const uint8_t* body, size_t n, uint8_t* out) {
  size_t p = 0;
  uint8_t bcc = 0;
  out[p++] = STX;
  for (size_t i = 0; i < n; i++) {
    const uint8_t hi = HEX_CHARS[body[i] >> 4];
    const uint8_t lo = HEX_CHARS[body[i] & 0x0F];
    out[p++] = hi;  bcc ^= hi;
    out[p++] = lo;  bcc ^= lo;
  }
  out[p++] = ETX;
  bcc ^= ETX;
  out[p++] = HEX_CHARS[bcc >> 4];
  out[p++] = HEX_CHARS[bcc & 0x0F];
  return p;  // 表示=32 / ブザー=16 / INIT=10
}

// 表示更新の本体 (14バイト)。value を区切り無しの数字のまま右詰めで表示する。
// 120 を渡すと "2.00"(2分00秒) ではなく "120" と出る。blank=true で全消灯。
static void buildDisplayBody(uint32_t value, bool blank, uint8_t* body) {
  memset(body, 0x00, 14);
  body[0] = CMD_DISPLAY;
  if (!blank) {
    char text[12];
    const int len = snprintf(text, sizeof(text), "%lu", (unsigned long)value);
    const int start = (len >= DIGITS) ? 0 : (DIGITS - len);  // 右詰め、上位は消灯
    for (int i = 0; i < len && start + i < DIGITS; i++) {
      body[1 + start + i] = segForChar(text[i]);
    }
  }
  body[8]  = 0x00;  // 区切り(コロン)は使わない
  body[12] = 0x00;  // 得点画面フラグ
}

static void sendDisplay(uint32_t value, bool blank) {
  uint8_t body[14];
  uint8_t frame[32];
  buildDisplayBody(value, blank, body);
  RS485.write(frame, buildFrame(body, sizeof(body), frame));
}

static void sendBuzzer(uint8_t kind) {
  const uint8_t body[6] = {CMD_BUZZER, 0x00, 0x01, kind, 0x00, 0x00};
  uint8_t frame[16];
  RS485.write(frame, buildFrame(body, sizeof(body), frame));
}

// 送信開始の宣言フレーム。02 30 45 30 30 30 31 03 37 37
static void sendInit() {
  const uint8_t body[3] = {CMD_INIT, 0x00, 0x01};
  uint8_t frame[10];
  RS485.write(frame, buildFrame(body, sizeof(body), frame));
  RS485.flush();
}

// ============================================================ 本体ブザー
// M5StickC ライブラリには M5.Beep が無いため、ledc を直接使う。
// ESP32 Arduino core は 2.x と 3.x で API が違うので両方に対応する。
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3

static void buzzerBegin() {
  if (BUZZER_PIN >= 0) ledcAttach(BUZZER_PIN, BUZZER_FREQ, 10);
}
static void buzzerOn() {
  if (BUZZER_PIN >= 0) ledcWriteTone(BUZZER_PIN, BUZZER_FREQ);
}
static void buzzerOff() {
  if (BUZZER_PIN >= 0) ledcWrite(BUZZER_PIN, 0);
}

#else

static const uint8_t BUZZER_CH = 0;
static void buzzerBegin() {
  if (BUZZER_PIN >= 0) {
    ledcSetup(BUZZER_CH, BUZZER_FREQ, 10);
    ledcAttachPin(BUZZER_PIN, BUZZER_CH);
    ledcWrite(BUZZER_CH, 0);
  }
}
static void buzzerOn() {
  if (BUZZER_PIN >= 0) ledcWriteTone(BUZZER_CH, BUZZER_FREQ);
}
static void buzzerOff() {
  if (BUZZER_PIN >= 0) ledcWrite(BUZZER_CH, 0);
}

#endif

// ============================================================ 合図(ブザー)
// 「0.7秒鳴動 -> 0.3秒インターバル」を count 回。回数だけ場面ごとに違う。
static const uint32_t BEEP_ON_MS    = 700;
static const uint32_t BEEP_CYCLE_MS = 1000;

static const uint8_t BUZZER_READY  = 2;  // ムーブアップ開始の予告
static const uint8_t BUZZER_START  = 1;  // 行射開始 / 再開
static const uint8_t BUZZER_FINISH = 3;  // 全ラウンド終了
static const uint8_t BUZZER_ALARM  = 5;  // ボタンBによる中断

// ============================================================ 状態機械
enum State : uint8_t {
  PRE_START,             // 開始前
  MOVEUP,                // ムーブアップ
  SHOOTING,              // 行射
  INTERRUPTED_MOVEUP,    // 中断（ムーブアップ中）
  INTERRUPTED_SHOOTING,  // 中断（行射中、時間はそのまま停止）
  FINISHED               // 行射終了
};

static State    state       = PRE_START;
static uint16_t roundNo     = 1;
static uint16_t durationSec = MOVEUP_SEC;
static uint32_t elapsedMs   = 0;      // 停止中はここに固定される
static uint32_t runOrigin   = 0;
static bool     running     = false;  // false = 停止中

static uint32_t beepAnchor  = 0;
static uint8_t  beepCount   = 0;
static bool     beepBlink   = false;
static bool     beepArmed   = false;

static uint32_t currentElapsedMs(uint32_t now) {
  return running ? elapsedMs + (now - runOrigin) : elapsedMs;
}

static uint32_t totalMs() {
  return (uint32_t)durationSec * 1000UL;
}

static uint16_t remainingSec(uint32_t now) {
  const uint32_t sec = currentElapsedMs(now) / 1000;
  return (durationSec > sec) ? (uint16_t)(durationSec - sec) : 0;
}

static void triggerBeep(uint8_t count, uint32_t now, bool blink = false) {
  beepAnchor = now;
  beepCount  = count;
  beepBlink  = blink;
  beepArmed  = true;
}

static void silence() {
  beepArmed = false;
  beepBlink = false;
  beepCount = 0;
}

// 本来なら鳴っているはずの時間帯か。BUZZER_KIND_SEL=0 でも点滅と画面表示に使う。
static bool beepActiveAt(uint32_t now) {
  if (!beepArmed) return false;
  const uint32_t rel = now - beepAnchor;
  if (rel >= (uint32_t)beepCount * BEEP_CYCLE_MS) return false;
  return (rel % BEEP_CYCLE_MS) < BEEP_ON_MS;
}

// 点滅させる時間帯。最後の鳴動が終わったところまでで、末尾のインターバルは含まない。
static bool beepBlinkWindow(uint32_t now) {
  if (!beepBlink || !beepArmed) return false;
  const uint32_t total = (uint32_t)beepCount * BEEP_CYCLE_MS - (BEEP_CYCLE_MS - BEEP_ON_MS);
  return (now - beepAnchor) < total;
}

// --- 状態遷移の内部処理 ---
static void startMoveup(uint32_t now) {
  state       = MOVEUP;
  durationSec = MOVEUP_SEC;
  elapsedMs   = 0;
  runOrigin   = now;
  running     = true;
  triggerBeep(BUZZER_READY, now);  // 2声
}

static void startShooting(uint32_t now, bool resuming) {
  state = SHOOTING;
  if (!resuming) {
    durationSec = SHOOTING_SEC;
    elapsedMs   = 0;
  }
  runOrigin = now;
  running   = true;
  // 新規開始・再開のどちらも1声。緑/黄色は関係ない
  triggerBeep(BUZZER_START, now);
}

static void finish(uint32_t now) {
  state       = FINISHED;
  durationSec = 0;
  elapsedMs   = 0;
  running     = false;
  triggerBeep(BUZZER_FINISH, now, true);  // 3声 + 点滅
}

// ムーブアップ+行射の1ラウンド(「立」)が終わった。早送りでも自然終了でも同じ処理。
static void completeRound(uint32_t now) {
  if (roundNo < REPEAT) {
    roundNo++;
    startMoveup(now);  // 2声
  } else {
    finish(now);       // 3声
  }
}

static void resetToPreStart() {
  roundNo     = 1;
  state       = PRE_START;
  durationSec = MOVEUP_SEC;
  elapsedMs   = 0;
  running     = false;
  silence();
}

// --- ボタン操作 (対応する状態以外では何も起きない) ---
static void keySpace(uint32_t now) {   // ボタンA 短押し
  switch (state) {
    case PRE_START:            startMoveup(now); break;              // 2声
    case INTERRUPTED_MOVEUP:                                          // 無音。ここでリセット
      state       = PRE_START;
      durationSec = MOVEUP_SEC;
      elapsedMs   = 0;
      break;
    case INTERRUPTED_SHOOTING: startShooting(now, true); break;       // 1声
    case FINISHED:             resetToPreStart(); break;              // 無音
    default: break;
  }
}

static void keyRight(uint32_t now) {   // ボタンA 長押し = 早送り
  if (state == MOVEUP || state == SHOOTING) completeRound(now);
}

static void keyEnter(uint32_t now) {   // ボタンB 短押し = 中断
  if (state == MOVEUP) {
    elapsedMs = currentElapsedMs(now);
    running   = false;
    state     = INTERRUPTED_MOVEUP;
    triggerBeep(BUZZER_ALARM, now);  // 5声
  } else if (state == SHOOTING) {
    elapsedMs = currentElapsedMs(now);
    running   = false;
    state     = INTERRUPTED_SHOOTING;
    triggerBeep(BUZZER_ALARM, now);  // 5声
  }
}

// 時間切れで自動的に次の状態へ進んでいないか確認する。
static void updateState(uint32_t now) {
  if (!running) return;
  if (currentElapsedMs(now) < totalMs()) return;
  if (state == MOVEUP) {
    startShooting(now, false);  // 1声
  } else if (state == SHOOTING) {
    completeRound(now);         // 2声 or 3声
  }
}

// ============================================================ 画面
// 画面には「信号の色」と「残り秒数」だけを出す。遠くから一目で読めるように、
// ラベル・立表示・残時間バーは置かず、数字を画面いっぱいまで大きくする。
static TFT_eSprite spr = TFT_eSprite(&M5.Lcd);
static int16_t W = 160, H = 80;

// 信号の色。緑は白文字が読めるよう、原色より濃くしてある。
static const uint16_t COLOR_RED    = 0xF800;  // #FF0000
static const uint16_t COLOR_GREEN  = 0x04A8;  // #009946
static const uint16_t COLOR_YELLOW = 0xFFE0;  // #FFFF00

// 数字に使うフォントの候補。「フォント番号, 拡大率」で、描画される高さは
//   8x2=150 / 7x2=96 / 4x3=78 / 8x1=75 / 4x2=52 / 6x1=48 / 7x1=48 / ...
// 大きい順に並べ、画面に収まる最初のものを使う。同じくらいの高さなら、
// 拡大でギザギザにならない等倍のフォントを先に置いている。
struct FontChoice { uint8_t font; uint8_t size; };
static const FontChoice FONT_CANDIDATES[] = {
  {8, 2}, {7, 2}, {8, 1}, {4, 3}, {4, 2}, {6, 1}, {7, 1}, {2, 2}, {4, 1}, {2, 1}
};

static uint8_t  bigFont  = 7;
static uint8_t  bigSize  = 1;
static uint16_t sizedFor = 0xFFFF;  // このフォントを選んだときの基準値

// maxValue を表示しても収まる、いちばん大きなフォントを選ぶ。
// フェーズごとに一度だけ決めるので、桁が減っても数字の大きさは変わらない。
static void chooseFont(uint16_t maxValue) {
  char text[8];
  snprintf(text, sizeof(text), "%u", (unsigned)maxValue);

  bigFont = 2;
  bigSize = 1;
  for (uint8_t i = 0; i < sizeof(FONT_CANDIDATES) / sizeof(FONT_CANDIDATES[0]); i++) {
    const FontChoice c = FONT_CANDIDATES[i];
    spr.setTextSize(c.size);
    // 高さを先に見る。ビルドに含まれていないフォントは高さ 0 になるので、
    // その場合は幅テーブルを引きにいかずに飛ばす。
    const int16_t h = spr.fontHeight(c.font);
    if (h <= 0 || h > H) continue;
    const int16_t w = spr.textWidth(text, c.font);
    if (w > 0 && w <= W - 6) {
      bigFont = c.font;
      bigSize = c.size;
      break;
    }
  }
  spr.setTextSize(1);
}

// 信号の色。行射だけが緑/黄で、それ以外は赤。
static uint16_t signalColor(uint32_t now) {
  if (state == SHOOTING) {
    return (remainingSec(now) <= WARN_SEC) ? COLOR_YELLOW : COLOR_GREEN;
  }
  return COLOR_RED;
}

static void render(uint32_t now, uint16_t value, bool blank) {
  if (durationSec != sizedFor) {
    sizedFor = durationSec;
    chooseFont(durationSec);
  }

  const uint16_t bg = signalColor(now);
  const uint16_t fg = (bg == COLOR_YELLOW) ? TFT_BLACK : TFT_WHITE;

  spr.fillSprite(bg);

  if (!blank) {
    char text[8];
    snprintf(text, sizeof(text), "%u", (unsigned)value);
    spr.setTextColor(fg, bg);
    spr.setTextDatum(MC_DATUM);
    spr.setTextSize(bigSize);
    spr.drawString(text, W / 2, H / 2, bigFont);
    spr.setTextSize(1);
  }

  spr.pushSprite(0, 0);
}

// ============================================================ 本体
void setup() {
  M5.begin();
  M5.Axp.ScreenBreath(SCREEN_BRIGHT);

  M5.Lcd.setRotation(SCREEN_ROTATION);
  W = M5.Lcd.width();
  H = M5.Lcd.height();

  spr.setColorDepth(16);
  spr.createSprite(W, H);

  buzzerBegin();

  RS485.begin(RS485_BAUD, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);
  sendInit();

  resetToPreStart();
}

void loop() {
  M5.update();

  const uint32_t now = millis();

  // --- 入力 ---
  static bool longFired = false;
  if (M5.BtnA.isPressed() && !longFired && M5.BtnA.pressedFor(LONG_PRESS_MS)) {
    longFired = true;
    keyRight(now);                       // 長押し = 早送り
  }
  if (M5.BtnA.wasReleased()) {
    if (!longFired) keySpace(now);       // 短押し = 開始/再開
    longFired = false;
  }
  if (M5.BtnB.wasPressed()) {
    keyEnter(now);                       // 中断
  }

  updateState(now);

  // --- 表示内容の決定 (行射終了の点滅は 0 と全消灯の切り替え) ---
  uint16_t value = remainingSec(now);
  bool blank = false;
  if (state == FINISHED && beepBlinkWindow(now)) {
    value = 0;
    blank = !beepActiveAt(now);
  }

  // --- 85ms 周期で送信 (実機は静止画面でも送り続けている) ---
  static uint32_t nextFrameMs = 0;
  if ((int32_t)(now - nextFrameMs) >= 0) {
    nextFrameMs = now + FRAME_INTERVAL_MS;
    if (BUZZER_KIND_SEL != 0 && beepActiveAt(now)) {
      sendBuzzer(BUZZER_KIND_SEL == 1 ? 0x01 : 0x00);  // 表示の合間にブザーを挟む
    }
    sendDisplay(value, blank);
  }

  // --- 本体側のブザー (BUZZER_PIN < 0 なら何もしない) ---
  {
    static bool toneOn = false;
    const bool want = beepActiveAt(now);
    if (want != toneOn) {
      toneOn = want;
      if (want) buzzerOn(); else buzzerOff();
    }
  }

  // --- 描画 (約20Hz) ---
  static uint32_t nextDrawMs = 0;
  if ((int32_t)(now - nextDrawMs) >= 0) {
    nextDrawMs = now + 50;
    render(now, value, blank);
  }
}
