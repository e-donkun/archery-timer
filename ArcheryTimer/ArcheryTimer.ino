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
 * 待機(STANDBY)または行射終了(FINISHED)からボタンBを長押しすると、補充矢・
 * シュートオフの設定画面(画面は黒)に入ります。長押しするたびに
 *   Setting - MAKEUP -> Setting - SHOOTOFF -> STANDBY -> Setting - MAKEUP -> ...
 * と回り、設定画面でボタンBを短押しすると行射時間が切り替わります。
 * ボタンAを押すと 10秒のムーブアップ(赤) + 選んだ行射 を1回だけ行って、
 * もとの設定に戻ります。
 *
 * 通信は仕様どおり固定:
 *   EIA RS-485準拠 / 半二重 / 調歩同期 / 9600bps / スタート1 / データ8 / パリティ無 / ストップ1
 *
 * 操作 (Python版のキー割り当てに対応):
 *   ボタンA 短押し  = Space  開始 / 再開
 *   ボタンA 長押し  = 右矢印 早送り (この立を終わりにして次へ)
 *   ボタンB 短押し  = Enter  中断 (5声)
 *   ボタンB 長押し  = —      中断中に待機へ戻す / 設定画面(MAKEUP/SHOOTOFF)を回す
 */

// M5StickC Plus を使う場合は include を入れ替えてください
#include <M5StickC.h>
// #include <M5StickCPlus.h>

// ============================================================ 設定
static const uint16_t MOVEUP_SEC   = 10;   // ムーブアップ（準備）時間 [秒]
static const uint16_t SHOOTING_SEC = 180;  // 行射時間 [秒]
static const uint16_t WARN_SEC     = 30;   // 行射残りこの秒数から黄色 [秒]
// 繰り返す「立」の数。0 は無制限(∞)。STANDBY 中にボタンBで
// 1 -> 2 -> 3 -> 4 -> ∞ -> 1 と切り替えられるので、ここは起動時の値。
static const uint16_t REPEAT_DEFAULT = 1;
static const uint16_t REPEAT_MAX     = 4;  // ボタンBで回せる上限 (この次が ∞)

// 補充矢(メイクアップ)とシュートオフの行射時間 [秒]。設定画面でボタンBを短押しする
// たびに、その設定の表を順に回る。最後まで行くと先頭に戻る。
static const uint16_t MAKEUP_SEC_TABLE[]   = {20, 30, 40, 60, 80, 90, 100, 120, 150, 160};
static const uint16_t SHOOTOFF_SEC_TABLE[] = {30, 40, 20};

// 設定画面。ボタンB長押しで、この順に切り替えたあと待機(STANDBY)に戻る。
struct MakeupPreset {
  const char*     name;   // 上段左に "Setting - " に続けて出す名前
  const uint16_t* table;  // 行射時間の表 [秒]
  uint8_t         count;
};
static const MakeupPreset MAKEUP_PRESETS[] = {
  {"MAKEUP",   MAKEUP_SEC_TABLE,   sizeof(MAKEUP_SEC_TABLE)   / sizeof(MAKEUP_SEC_TABLE[0])},
  {"SHOOTOFF", SHOOTOFF_SEC_TABLE, sizeof(SHOOTOFF_SEC_TABLE) / sizeof(SHOOTOFF_SEC_TABLE[0])},
};
static const uint8_t MAKEUP_PRESET_COUNT =
    sizeof(MAKEUP_PRESETS) / sizeof(MAKEUP_PRESETS[0]);

// UD0040 のブザー音色: 1 = ブザー1 (0x01) / 2 = ブザー2 (0x00) / 0 = 鳴らさない
static const uint8_t  BUZZER_KIND_SEL = 1;

// M5StickC 本体側でも鳴らす場合のピン。-1 で鳴らさない。
//   M5StickC Plus : 内蔵ブザーが G2 にあるので 2 を指定する
//   無印 M5StickC : 内蔵ブザーは無い。SPK HAT などを繋いだピン (例: 26) を指定する
// UD0040 のブザーは、この設定とは無関係に BUZZER_KIND_SEL で鳴る。
static const int      BUZZER_PIN  = -1;
static const uint16_t BUZZER_FREQ = 2000;

static const uint8_t  SCREEN_ROTATION = 1;   // 3 または 1 で横向き（上下逆なら 1）
// 画面の明るさ。M5StickC ライブラリの ScreenBreath() は 7..12 で、12 が最大
// (それ以上を渡しても 12 に丸められる)。0..100 を取るライブラリを使っていて
// 画面が暗い場合は 100 にしてください。 → 100じゃないと暗いので 100にする。
static const uint8_t  SCREEN_BRIGHT   = 100;
static const uint32_t LONG_PRESS_MS   = 800; // 長押しの判定時間

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
  STANDBY,        // 待機（ここで繰り返し回数を設定できる）
  MAKEUP_SETTING, // 設定画面（補充矢・シュートオフの行射時間を選ぶ）
  MOVEUP,         // ムーブアップ
  SHOOTING,       // 行射
  HALT_MOVEUP,    // 中断（ムーブアップ中）
  HALT_SHOOTING,  // 中断（行射中、時間はそのまま停止）
  FINISHED        // 行射終了
};

static State    state       = STANDBY;
static uint16_t roundNo     = 1;
static uint16_t repeatCount = REPEAT_DEFAULT;  // 0 = ∞
static uint16_t durationSec = MOVEUP_SEC;
static uint32_t elapsedMs   = 0;      // 停止中はここに固定される
static uint32_t runOrigin   = 0;
static bool     running     = false;  // false = 停止中

// 補充矢・シュートオフを1立だけ行うモード。ムーブアップ+行射が終わると解除され、
// 繰り返し回数などのもとの設定に戻る。
static bool     makeupMode  = false;
static uint8_t  presetNo    = 0;         // MAKEUP_PRESETS の位置 (0=MAKEUP, 1=SHOOTOFF)
static uint8_t  presetIndex[MAKEUP_PRESET_COUNT] = {0};  // 設定ごとに選んでいる秒数の位置

static uint32_t beepAnchor  = 0;
static uint8_t  beepCount   = 0;
static bool     beepBlink   = false;
static bool     beepArmed   = false;

// 今の設定 (MAKEUP か SHOOTOFF)。
static const MakeupPreset& preset() {
  return MAKEUP_PRESETS[presetNo];
}

// 今選んでいる補充矢・シュートオフの行射時間。
static uint16_t makeupSec() {
  return preset().table[presetIndex[presetNo]];
}

// この立の行射時間。メイクアップ中だけ選んだ時間になる。
static uint16_t shootingSec() {
  return makeupMode ? makeupSec() : SHOOTING_SEC;
}

static uint32_t currentElapsedMs(uint32_t now) {
  return running ? elapsedMs + (now - runOrigin) : elapsedMs;
}

static uint32_t totalMs() {
  return (uint32_t)durationSec * 1000UL;
}

static uint32_t remainingMs(uint32_t now) {
  const uint32_t done = currentElapsedMs(now);
  return (done >= totalMs()) ? 0 : (totalMs() - done);
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
    durationSec = shootingSec();
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
  if (makeupMode) {      // メイクアップは1立だけ。終わったらもとの設定に戻る
    makeupMode = false;
    finish(now);         // 3声
    return;
  }
  if (repeatCount == 0 || roundNo < repeatCount) {
    if (roundNo < 0xFFFF) roundNo++;   // ∞ のときに一周しないようにする
    startMoveup(now);  // 2声
  } else {
    finish(now);       // 3声
  }
}

// 待機に戻す。繰り返し回数の設定はそのまま残す。
static void resetToStandby() {
  roundNo     = 1;
  makeupMode  = false;
  state       = STANDBY;
  durationSec = MOVEUP_SEC;
  elapsedMs   = 0;
  running     = false;
  silence();
}

// no 番目の設定画面に入る（切り替えも同じ）。無音。
// 中央には選んでいる行射時間が出るので、そのまま設定の表示になる。
static void enterSetting(uint8_t no) {
  presetNo    = no;
  makeupMode  = true;
  state       = MAKEUP_SETTING;
  durationSec = makeupSec();
  elapsedMs   = 0;
  running     = false;
  silence();
}

// 選んでいる設定の行射時間を、その表の順に回す。
// MAKEUP は 20 -> 30 -> ... -> 160 -> 20、SHOOTOFF は 30 -> 40 -> 20 -> 30。
static void cycleMakeupSec() {
  uint8_t& idx = presetIndex[presetNo];
  idx = (uint8_t)((idx + 1) % preset().count);
  durationSec = makeupSec();
  elapsedMs   = 0;
}

// 繰り返し回数を 1 -> 2 -> 3 -> 4 -> ∞ -> 1 と回す。
static void cycleRepeat() {
  if (repeatCount == 0)              repeatCount = 1;
  else if (repeatCount >= REPEAT_MAX) repeatCount = 0;   // ∞
  else                                repeatCount++;
}

// --- ボタン操作 (対応する状態以外では何も起きない) ---
static void keySpace(uint32_t now) {   // ボタンA 短押し = 開始 / 再開
  switch (state) {
    case STANDBY:                                              // 2声
    case MAKEUP_SETTING: startMoveup(now); break;               // 10秒のムーブアップから
    case HALT_MOVEUP:                                          // 無音。ここでリセット
      // メイクアップ中は、もとの待機ではなくメイクアップ待機に戻す
      state       = makeupMode ? MAKEUP_SETTING : STANDBY;
      durationSec = makeupMode ? makeupSec() : MOVEUP_SEC;
      elapsedMs   = 0;
      break;
    case HALT_SHOOTING: startShooting(now, true); break;       // 1声
    case FINISHED:      resetToStandby(); break;               // 無音
    default: break;
  }
}

static void keyRight(uint32_t now) {   // ボタンA 長押し = 早送り
  if (state == MOVEUP || state == SHOOTING) completeRound(now);
}

static void keyEnter(uint32_t now) {   // ボタンB 短押し = 繰り返し回数 / 中断
  if (state == STANDBY) {
    cycleRepeat();                   // 無音。待機中だけ回数を設定できる
  } else if (state == MAKEUP_SETTING) {
    cycleMakeupSec();                // 無音。設定画面では行射時間を選ぶ
  } else if (state == MOVEUP) {
    elapsedMs = currentElapsedMs(now);
    running   = false;
    state     = HALT_MOVEUP;
    triggerBeep(BUZZER_ALARM, now);  // 5声
  } else if (state == SHOOTING) {
    elapsedMs = currentElapsedMs(now);
    running   = false;
    state     = HALT_SHOOTING;
    triggerBeep(BUZZER_ALARM, now);  // 5声
  }
}

// ボタンB 長押し = 中断中は待機へ戻す。それ以外は設定画面を
//   Setting - MAKEUP -> Setting - SHOOTOFF -> STANDBY -> ... と回す。
static void keyReset() {
  if (state == HALT_MOVEUP || state == HALT_SHOOTING) {
    resetToStandby();
  } else if (state == STANDBY || state == FINISHED) {
    enterSetting(0);                             // 最初の設定 (MAKEUP) へ
  } else if (state == MAKEUP_SETTING) {
    if (presetNo + 1 < MAKEUP_PRESET_COUNT) enterSetting(presetNo + 1);  // 次の設定へ
    else                                    resetToStandby();            // 一周したら待機へ
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
// 上段に状態と「立」、中央に残り秒数、下段に残時間バーを出す。
// 合図が鳴っている間は枠を出す。中央の数字は、置ける範囲でいちばん大きくする。
static TFT_eSprite spr = TFT_eSprite(&M5.Lcd);
static int16_t W = 160, H = 80;

// 信号の色。緑は白文字が読めるよう、原色より濃くしてある。
static const uint16_t COLOR_RED    = 0xF800;  // #FF0000
static const uint16_t COLOR_GREEN  = 0x04A8;  // #009946
static const uint16_t COLOR_YELLOW = 0xFFE0;  // #FFFF00
// メイクアップ待機だけは黒地。通常の待機(赤)と一目で区別できるようにするため。
static const uint16_t COLOR_BLACK  = 0x0000;  // #000000

// 上段(状態・立)と下段(バー)の高さ。中央の数字はその残りに収める。
static const int16_t TOP_H = 18;
static const int16_t BAR_H = 6;
static const int16_t BAR_MARGIN = 2;
static const int16_t BOTTOM_H = BAR_H + BAR_MARGIN * 2;

// 数字に使うフォントの候補。「フォント番号, 拡大率」で、描画される高さは
//   8x2=150 / 7x2=96 / 8x1=75 / 4x3=78 / 6x1=48 / 7x1=48 / 4x2=52 / ...
// 大きい順に並べる。ただし高さがほぼ同じなら、拡大でギザギザにならない
// 等倍のフォント (6x1, 7x1) を、拡大した 4x2 より先に置いている。
struct FontChoice { uint8_t font; uint8_t size; };
static const FontChoice FONT_CANDIDATES[] = {
  {8, 2}, {7, 2}, {8, 1}, {4, 3}, {6, 1}, {7, 1}, {4, 2}, {2, 2}, {4, 1}, {2, 1}
};

static uint8_t  bigFont  = 7;
static uint8_t  bigSize  = 1;
static uint16_t sizedFor = 0xFFFF;  // このフォントを選んだときの基準値

// maxValue を表示しても中央の帯に収まる、いちばん大きなフォントを選ぶ。
// フェーズごとに一度だけ決めるので、桁が減っても数字の大きさは変わらない。
static void chooseFont(uint16_t maxValue) {
  char text[8];
  snprintf(text, sizeof(text), "%u", (unsigned)maxValue);
  const int16_t availH = H - TOP_H - BOTTOM_H;

  bigFont = 2;
  bigSize = 1;
  for (uint8_t i = 0; i < sizeof(FONT_CANDIDATES) / sizeof(FONT_CANDIDATES[0]); i++) {
    const FontChoice c = FONT_CANDIDATES[i];
    spr.setTextSize(c.size);
    // 高さを先に見る。ビルドに含まれていないフォントは高さ 0 になるので、
    // その場合は幅テーブルを引きにいかずに飛ばす。
    const int16_t h = spr.fontHeight(c.font);
    if (h <= 0 || h > availH) continue;
    const int16_t w = spr.textWidth(text, c.font);
    if (w > 0 && w <= W - 6) {
      bigFont = c.font;
      bigSize = c.size;
      break;
    }
  }
  spr.setTextSize(1);
}

static const char* stateLabel() {
  static char buf[24];
  switch (state) {
    case STANDBY:       return "STANDBY";    // 待機
    case MAKEUP_SETTING:                     // 設定画面 (Setting - MAKEUP など)
      snprintf(buf, sizeof(buf), "Setting - %s", preset().name);
      return buf;
    case MOVEUP:        return "MOVE UP";    // ムーブアップ
    case SHOOTING:      return "SHOOTING";   // 行射
    case HALT_MOVEUP:
    case HALT_SHOOTING: return "HALT";       // 中断
    default:            return "FINISHED";   // 行射終了
  }
}

// 無制限(∞)の印。フォントに無いので、円を2つ並べて自前で描く。
static const int16_t INF_R  = 5;
static const int16_t INF_W  = INF_R * 4;
static const int16_t INF_CY = 8;   // 上段(フォント2、高さ16)の中心

static void drawInfinity(int16_t rightX, uint16_t color) {
  const int16_t cx2 = rightX - INF_R;
  const int16_t cx1 = cx2 - INF_R * 2;
  spr.drawCircle(cx1, INF_CY, INF_R,     color);
  spr.drawCircle(cx1, INF_CY, INF_R - 1, color);
  spr.drawCircle(cx2, INF_CY, INF_R,     color);
  spr.drawCircle(cx2, INF_CY, INF_R - 1, color);
}

// 上段右の「立」。分母は繰り返し回数で、0 のときは ∞ を描く。
// 待機中は設定できることが分かるように分母だけ点滅させる。
static void drawRounds(uint32_t now, uint16_t fg, uint16_t bg) {
  const bool hideDen = (state == STANDBY) && ((now / 400) % 2 == 1);

  spr.setTextColor(fg, bg);
  spr.setTextDatum(TR_DATUM);

  int16_t x = W - 4;
  if (repeatCount == 0) {
    if (!hideDen) drawInfinity(x, fg);
    x -= INF_W;
  } else {
    char den[8];
    snprintf(den, sizeof(den), "%u", (unsigned)repeatCount);
    if (!hideDen) spr.drawString(den, x, 0, 2);
    x -= spr.textWidth(den, 2);
  }

  char num[12];
  snprintf(num, sizeof(num), "%u/", (unsigned)roundNo);
  spr.drawString(num, x, 0, 2);
}

// 補充矢・シュートオフの行射中に、上段右へ「立」の代わりに出す名前と設定秒数。
// 例: MAKEUP(30s) / SHOOTOFF(40s)。設定画面は上段左がそのまま Setting - ... なので
// 出さない。出すものがあれば true。
static bool makeupTagText(char* buf, size_t n) {
  if (!makeupMode || state == MAKEUP_SETTING) return false;
  snprintf(buf, n, "%s(%us)", preset().name, (unsigned)makeupSec());
  return true;
}

// 設定画面の中央「[ 20 ]sec」。数字の枠は3桁ぶんで固定してあるので、2桁でも
// 3桁でも [ ] の位置は動かない。数字はその枠の中で中央揃えにする。
static const int16_t SET_GAP_IN  = 4;   // [ と数字の間
static const int16_t SET_GAP_OUT = 6;   // ] と sec の間
static const uint8_t SETTING_NUM_FONTS[] = {6, 4};  // 数字に使う候補 (大きい順)

// 「[ 000 ]sec」を組んだときの全体の幅。slotW に数字3桁ぶんの枠の幅を返す。
static int16_t settingGroupW(uint8_t numFont, uint8_t subFont, int16_t* slotW) {
  *slotW = spr.textWidth("000", numFont);
  return spr.textWidth("[", subFont) + SET_GAP_IN + *slotW + SET_GAP_IN
       + spr.textWidth("]", subFont) + SET_GAP_OUT + spr.textWidth("sec", subFont);
}

static void drawSettingValue(uint16_t fg, uint16_t bg) {
  const int16_t availH = H - TOP_H - BOTTOM_H;
  const int16_t cy     = TOP_H + availH / 2;

  // 数字はフォント2から始めて、全体が横に収まるならより大きいものに差し替える。
  // 括弧と sec は数字より一段小さいフォントで添える。
  uint8_t numFont = 2, subFont = 2;
  int16_t slotW   = 0;
  int16_t groupW  = settingGroupW(numFont, subFont, &slotW);
  for (uint8_t i = 0; i < sizeof(SETTING_NUM_FONTS) / sizeof(SETTING_NUM_FONTS[0]); i++) {
    const uint8_t f = SETTING_NUM_FONTS[i];
    const uint8_t t = (f > 4) ? 4 : 2;
    // ビルドに含まれていないフォントは高さ 0 になるので飛ばす
    if (spr.fontHeight(f) <= 0 || spr.fontHeight(f) > availH) continue;
    if (spr.fontHeight(t) <= 0) continue;
    int16_t w = 0;
    const int16_t g = settingGroupW(f, t, &w);
    if (w > 0 && g <= W - 8) {
      numFont = f;  subFont = t;  slotW = w;  groupW = g;
      break;
    }
  }

  char num[8];
  snprintf(num, sizeof(num), "%u", (unsigned)makeupSec());

  spr.setTextColor(fg, bg);
  int16_t x = (W - groupW) / 2;

  spr.setTextDatum(ML_DATUM);
  spr.drawString("[", x, cy, subFont);
  x += spr.textWidth("[", subFont) + SET_GAP_IN;

  spr.setTextDatum(MC_DATUM);
  spr.drawString(num, x + slotW / 2, cy, numFont);   // 枠の中で中央揃え
  x += slotW + SET_GAP_IN;

  spr.setTextDatum(ML_DATUM);
  spr.drawString("]", x, cy, subFont);
  x += spr.textWidth("]", subFont) + SET_GAP_OUT;
  spr.drawString("sec", x, cy, subFont);
}

// 信号の色。行射だけが緑/黄、設定画面だけが黒で、それ以外は赤。
// 設定画面からスタートしても、ムーブアップは赤に戻る。
static uint16_t signalColor(uint32_t now) {
  if (state == MAKEUP_SETTING) return COLOR_BLACK;
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
  spr.setTextColor(fg, bg);

  // 上段: 状態と、「立」または補充矢・シュートオフの名前と秒数。
  // 状態はフォント2で出すが、文字数が多くて右側と並ばないときは小さい字にする。
  char tag[24];
  const bool    hasTag = makeupTagText(tag, sizeof(tag));
  const int16_t tagW   = hasTag ? spr.textWidth(tag, 1) : 0;
  const int16_t availW = W - 8 - (hasTag ? tagW + 6 : 0);

  const char* label = stateLabel();
  spr.setTextDatum(TL_DATUM);
  if (spr.textWidth(label, 2) <= availW) spr.drawString(label, 4, 0, 2);
  else                                   spr.drawString(label, 4, 4, 1);

  if (hasTag) {
    spr.setTextDatum(TR_DATUM);
    spr.drawString(tag, W - 4, 4, 1);
  } else if (!makeupMode) {
    drawRounds(now, fg, bg);
  }

  // 中央: 設定画面は「[ 20 ]sec」、それ以外は残り秒数
  if (state == MAKEUP_SETTING) {
    drawSettingValue(fg, bg);
  } else if (!blank) {
    char text[8];
    snprintf(text, sizeof(text), "%u", (unsigned)value);
    spr.setTextDatum(MC_DATUM);
    spr.setTextSize(bigSize);
    spr.drawString(text, W / 2, TOP_H + (H - TOP_H - BOTTOM_H) / 2, bigFont);
    spr.setTextSize(1);
  }

  // 下段: 残時間バー
  const int16_t barX = 4, barW = W - 8, barY = H - BAR_H - BAR_MARGIN;
  spr.drawRect(barX, barY, barW, BAR_H, fg);
  if (durationSec > 0) {
    const int16_t fill = (int16_t)((uint32_t)(barW - 2) * remainingMs(now) / totalMs());
    if (fill > 0) spr.fillRect(barX + 1, barY + 1, fill, BAR_H - 2, fg);
  }

  // 合図が鳴っている間は枠を出す
  if (beepActiveAt(now)) {
    spr.drawRect(0, 0, W, H, fg);
    spr.drawRect(1, 1, W - 2, H - 2, fg);
    spr.drawRect(2, 2, W - 4, H - 4, fg);
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

  resetToStandby();
}

void loop() {
  M5.update();

  const uint32_t now = millis();

  // --- 入力 ---
  static bool longFiredA = false;
  if (M5.BtnA.isPressed() && !longFiredA && M5.BtnA.pressedFor(LONG_PRESS_MS)) {
    longFiredA = true;
    keyRight(now);                        // A長押し = 早送り
  }
  if (M5.BtnA.wasReleased()) {
    if (!longFiredA) keySpace(now);       // A短押し = 開始/再開
    longFiredA = false;
  }

  static bool longFiredB = false;
  if (M5.BtnB.isPressed() && !longFiredB && M5.BtnB.pressedFor(LONG_PRESS_MS)) {
    longFiredB = true;
    keyReset();                           // B長押し = 中断中に待機へ戻す
  }
  if (M5.BtnB.wasReleased()) {
    if (!longFiredB) keyEnter(now);       // B短押し = 繰り返し回数 / 中断
    longFiredB = false;
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
