/*
 * ArcheryTimer - M5StickC 用 アーチェリー行射タイマー (RS-485 出力)
 *
 * old_python/snd_data.py の --timer モードを M5StickC に移植したものです。
 * モルテン ハンディータイマーアウトドア UD0040 を RS-485 で駆動しつつ、
 * M5StickC の画面（横向き）にも同じ内容を信号色つきで表示します。
 *
 *   初期画面 --[A]--> ムーブアップ 10秒(赤) --> 行射(緑 / 残り30秒で黄)
 *            --> (次の立 または 行射終了) --> 初期画面に戻る
 *
 * 起動すると Shooting の先頭 (180秒) / 1立 の初期画面になります。
 * 初期画面でボタンBを押すと設定(SETUP)画面(黒)に入り、ボタンBでカーソルが
 *   REPEAT -> MODE -> TIME -> [ Exit SETUP ] -> REPEAT -> ...
 * と移り、ボタンAでその項目が進みます。MODE は Shooting / MakeUp / ShootOff で、
 * MakeUp と ShootOff は必ず1立だけ行います。
 *
 * 通信は仕様どおり固定:
 *   EIA RS-485準拠 / 半二重 / 調歩同期 / 9600bps / スタート1 / データ8 / パリティ無 / ストップ1
 *
 * 操作 (Python版のキー割り当てに対応):
 *   ボタンA 短押し  = Space  開始 / 再開 / 設定の項目を進める
 *   ボタンA 長押し  = 右矢印 早送り (この立を終わりにして次へ)。設定画面では初期画面へ
 *   ボタンB 短押し  = Enter  設定画面へ / カーソル移動 / 中断 (5声)
 *   ボタンB 長押し  = —      初期画面へ戻る (行射中だけは中断)
 */

// M5StickC Plus を使う場合は include を入れ替えてください
#include <M5StickC.h>
// #include <M5StickCPlus.h>

// ============================================================ 設定
static const uint16_t MOVEUP_SEC   = 10;   // ムーブアップ（準備）時間 [秒]
static const uint16_t WARN_SEC     = 30;   // 行射残りこの秒数から黄色 [秒]
// 繰り返す「立」の数。0 は無制限(∞)。STANDBY 中にボタンBで
// 1 -> 2 -> 3 -> 4 -> ∞ -> 1 と切り替えられるので、ここは起動時の値。
static const uint16_t REPEAT_DEFAULT = 1;
static const uint16_t REPEAT_MAX     = 4;  // ボタンBで回せる上限 (この次が ∞)

// 行射時間 [秒]。設定(SETUP)画面の TIME でボタンAを押すたびに、その MODE の表を
// 順に回る。最後まで行くと先頭に戻る。先頭が起動時の値。
// MakeUp と ShootOff は同じ表を使う (選んでいる位置は別々に覚える)。
static const uint16_t SHOOTING_SEC_TABLE[] = {180, 240, 60, 90, 120};
static const uint16_t MAKEUP_SEC_TABLE[]   = {30, 40, 60, 80, 90, 100, 120, 150, 20};

// 3つの MODE。設定画面の MODE でボタンAを押すと、この順に切り替わって先頭に戻る。
struct Mode {
  const char*     title;  // 設定画面の MODE 欄に並べる名前
  const char*     tag;    // 実行中に上段右へ出す名前 (Shooting は「立」を出すので nullptr)
  const uint16_t* table;  // 行射時間の表 [秒]
  uint8_t         count;
};
#define MODE_TABLE(t) t, sizeof(t) / sizeof(t[0])
static const Mode MODES[] = {
  {"Shooting", nullptr,     MODE_TABLE(SHOOTING_SEC_TABLE)},
  {"MakeUp",   "MAKE UP",   MODE_TABLE(MAKEUP_SEC_TABLE)},
  {"ShootOff", "SHOOT OFF", MODE_TABLE(MAKEUP_SEC_TABLE)},
};
static const uint8_t MODE_COUNT = sizeof(MODES) / sizeof(MODES[0]);

// UD0040 のブザー音色: 1 = ブザー1 (0x01) / 2 = ブザー2 (0x00) / 0 = 鳴らさない
static const uint8_t  BUZZER_KIND_SEL = 1;

// 本体の赤色LED。合図(ブザー)が鳴っている間だけ点ける。M5StickC / Plus とも
// G10 で、LOW を出すと点灯する。-1 にすると使わない。
static const int      LED_PIN    = 10;
static const uint8_t  LED_ON     = LOW;

// M5StickC 本体側でも鳴らす場合のピン。-1 で鳴らさない。
//   M5StickC Plus : 内蔵ブザーが G2 にあるので 2 を指定する
//   無印 M5StickC : 内蔵ブザーは無い。RS-485 HAT で HAT 端子 (G0/G26) が
//                   埋まっているので、鳴らすなら別のピンに繋ぐ
// UD0040 のブザーは、この設定とは無関係に BUZZER_KIND_SEL で鳴る。
static const int      BUZZER_PIN  = -1;
static const uint16_t BUZZER_FREQ = 2000;

static const uint8_t  SCREEN_ROTATION = 1;   // 3 または 1 で横向き（上下逆なら 1）
// 画面の明るさ。M5StickC ライブラリの ScreenBreath() は 7..12 で、12 が最大
// (それ以上を渡しても 12 に丸められる)。0..100 を取るライブラリを使っていて
// 画面が暗い場合は 100 にしてください。 → 100じゃないと暗いので 100にする。
static const uint8_t  SCREEN_BRIGHT   = 100;
static const uint32_t LONG_PRESS_MS   = 800; // 長押しの判定時間

// RS-485 HAT の接続先 (M5StickC の底面 HAT 端子)。M5StickC ライブラリの
// examples/Hat/RS485 と同じ割り当て (Serial2.begin(..., 26, 0))。
// Grove の RS-485 ユニットを使う場合は 33 / 32 にする。
static const int  RS485_RX_PIN = 26;   // 受信。このプログラムでは使わない
static const int  RS485_TX_PIN = 0;    // 送信
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

static const uint32_t FRAME_INTERVAL_MS = 85;    // 実機の送信周期
static const uint32_t INIT_RESEND_MS    = 1000;  // 送信開始の宣言を出し直す間隔

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
// タイマーはこれを受け取るまで受信モードにならないので、起動時だけでなく
// 表示が変わるたびと INIT_RESEND_MS ごとに出す。途中でケーブルが抜けても、
// 挿し直せば1秒以内に受信モードに戻る。
static void sendInit() {
  const uint8_t body[3] = {CMD_INIT, 0x00, 0x01};
  uint8_t frame[10];
  RS485.write(frame, buildFrame(body, sizeof(body), frame));
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

// 赤色LED。ブザーと同じタイミングで点け消しする。
static void ledBegin() {
  if (LED_PIN < 0) return;
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, (LED_ON == LOW) ? HIGH : LOW);   // 消灯
}

static void ledSet(bool on) {
  if (LED_PIN < 0) return;
  digitalWrite(LED_PIN, on ? LED_ON : ((LED_ON == LOW) ? HIGH : LOW));
}

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
  SETTING,        // 設定画面（Shooting Time / MAKEUP / Shoot Off の行射時間を選ぶ）
  READY,          // 開始待ち（ここで繰り返し回数を設定できる）
  MOVEUP,         // ムーブアップ
  SHOOTING,       // 行射
  HALT_MOVEUP,    // 中断（ムーブアップ中）
  HALT_SHOOTING,  // 中断（行射中、時間はそのまま停止）
  FINISHED        // 行射終了
};

static State    state       = READY;
static uint16_t roundNo     = 1;
static uint16_t repeatCount = REPEAT_DEFAULT;  // 0 = ∞
static uint16_t durationSec = MOVEUP_SEC;
static uint32_t elapsedMs   = 0;      // 停止中はここに固定される
static uint32_t runOrigin   = 0;
static bool     running     = false;  // false = 停止中

// 選んでいる MODE と、MODE ごとに選んでいる行射時間の位置。
static uint8_t  modeNo    = 0;              // MODES の位置 (0=Shooting)
static uint8_t  modeIndex[MODE_COUNT] = {0};

// 設定(SETUP)画面のカーソル。ボタンBで下に移り、最後まで行くと先頭に戻る。
// ボタンAは、カーソルのある項目を進める (Exit だけは初期画面に出る)。
enum SetupCursor : uint8_t { CUR_REPEAT, CUR_MODE, CUR_TIME, CUR_EXIT, CUR_COUNT };
static uint8_t  setupCursor = CUR_REPEAT;

static uint32_t beepAnchor  = 0;
static uint8_t  beepCount   = 0;
static bool     beepBlink   = false;
static bool     beepArmed   = false;

// 今の設定 (Shooting Time / MAKEUP / Shoot Off)。
static const Mode& mode() {
  return MODES[modeNo];
}

// 今の設定で選んでいる行射時間。そのまま「この立の行射時間」になる。
static uint16_t shootingSec() {
  return mode().table[modeIndex[modeNo]];
}

// この設定で繰り返す「立」の数 (0 = ∞)。補充矢・シュートオフは必ず1立だけで、
// 繰り返し回数は設定できない。Shooting Time の設定値はそのまま残る。
static uint16_t repeatOf() {
  return (modeNo == 0) ? repeatCount : 1;
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
  if (repeatOf() == 0 || roundNo < repeatOf()) {
    if (roundNo < 0xFFFF) roundNo++;   // ∞ のときに一周しないようにする
    startMoveup(now);  // 2声
  } else {
    finish(now);       // 3声
  }
}

// 初期画面(開始待ち)に戻す。設定はそのまま残す。無音。中央は 0。
static void toReady() {
  roundNo     = 1;
  state       = READY;
  durationSec = 0;
  elapsedMs   = 0;
  running     = false;
  silence();
}

// 設定(SETUP)画面に入る。カーソルは先頭の REPEAT から。無音。
static void toSetup() {
  setupCursor = CUR_REPEAT;
  roundNo     = 1;
  state       = SETTING;
  durationSec = 0;
  elapsedMs   = 0;
  running     = false;
  silence();
}

// 選んでいる MODE の行射時間を、その表の順に回す。
static void cycleShootingSec() {
  uint8_t& idx = modeIndex[modeNo];
  idx = (uint8_t)((idx + 1) % mode().count);
}

// MODE を Shooting -> MakeUp -> ShootOff -> Shooting と回す。
static void cycleMode() {
  modeNo = (uint8_t)((modeNo + 1) % MODE_COUNT);
}

// 繰り返し回数を 1 -> 2 -> 3 -> 4 -> ∞ -> 1 と回す。
static void cycleRepeat() {
  if (repeatCount == 0)              repeatCount = 1;
  else if (repeatCount >= REPEAT_MAX) repeatCount = 0;   // ∞
  else                                repeatCount++;
}

// --- ボタン操作 (対応する状態以外では何も起きない) ---
// 設定画面でボタンAを押したとき。カーソルのある項目を進める。
static void setupAdvance() {
  switch (setupCursor) {
    case CUR_REPEAT: if (modeNo == 0) cycleRepeat(); break;  // 回数は Shooting だけ
    case CUR_MODE:   cycleMode();      break;
    case CUR_TIME:   cycleShootingSec(); break;
    default:         toReady();        break;                // [ Exit SETUP ]
  }
}

static void keySpace(uint32_t now) {   // ボタンA 短押し
  switch (state) {
    case SETTING:       setupAdvance(); break;                 // 無音。カーソルの項目を進める
    case READY:         startMoveup(now); break;               // 2声。10秒のムーブアップから
    case HALT_MOVEUP:   startMoveup(now); break;               // 2声。ムーブアップの最初から
    case HALT_SHOOTING: startShooting(now, true); break;       // 1声。止めたところから再開
    case FINISHED:      toReady(); break;                      // 無音。初期画面へ
    default: break;
  }
}

// ボタンA 長押し。ムーブアップ・行射中だけ早送り。設定画面は初期画面へ戻る。
static void keyRight(uint32_t now) {
  if (state == MOVEUP || state == SHOOTING) completeRound(now);  // この立を終わりにして次へ
  else if (state == SETTING)                toReady();           // 設定をやめて初期画面へ
  else                                      keySpace(now);
}

// 行射中・ムーブアップ中を中断して止める (5声)。それ以外の状態では何も起きない。
// ボタンB は短押しでも長押しでも中断できる。
static void keyHalt(uint32_t now) {
  if (state != MOVEUP && state != SHOOTING) return;
  elapsedMs = currentElapsedMs(now);
  running   = false;
  state     = (state == MOVEUP) ? HALT_MOVEUP : HALT_SHOOTING;
  triggerBeep(BUZZER_ALARM, now);  // 5声
}

static void keyEnter(uint32_t now) {   // ボタンB 短押し
  if (state == SETTING) {
    setupCursor = (uint8_t)((setupCursor + 1) % CUR_COUNT);  // 無音。カーソルを次の項目へ
  } else if (state == READY) {
    toSetup();                       // 無音。設定画面へ
  } else {
    keyHalt(now);                    // ムーブアップ・行射中なら中断 (5声)
  }
}

// ボタンB 長押し = 行射中だけ中断 (短押しと同じ)。それ以外は初期画面に戻る。
static void keyReset(uint32_t now) {
  if (state == SHOOTING) keyHalt(now);   // 中断 (5声)
  else                   toReady();      // 設定画面・赤い画面から初期画面へ
}

// 時間切れで自動的に次の状態へ進んでいないか確認する。
static void updateState(uint32_t now) {
  // 行射終了は、0 の点滅が終わったら自動で初期画面に戻る
  if (state == FINISHED && beepArmed && !beepBlinkWindow(now)) {
    toReady();
    return;
  }
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
static int16_t  sizedH   = -1;      // このフォントを選んだときの中央の帯の高さ

// maxValue を表示しても中央の帯に収まる、いちばん大きなフォントを選ぶ。
// フェーズごとに一度だけ決めるので、桁が減っても数字の大きさは変わらない。
static void chooseFont(uint16_t maxValue, int16_t availH) {
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
  switch (state) {
    case MOVEUP:        return "MOVE UP";     // ムーブアップ
    case HALT_MOVEUP:
    case HALT_SHOOTING: return "HALT";        // 中断
    case FINISHED:      return "FINISHED";    // 行射終了
    default:            return nullptr;       // 初期画面・設定・行射中は出さない
  }
}

// 右下に出す、ボタンAの行き先。無ければ nullptr。
static const char* keyHintA() {
  switch (state) {
    case SETTING:
      switch (setupCursor) {
        case CUR_REPEAT: return "REPEAT>>";
        case CUR_MODE:   return "MODE>>";
        case CUR_TIME:   return "TIME>>";
        default:         return "Enter";      // [ Exit SETUP ]
      }
    case READY:         return "START";       // ムーブアップ開始
    case HALT_MOVEUP:   return "START";       // ムーブアップの最初から
    case HALT_SHOOTING: return "RESUME";      // 止めたところから再開
    case FINISHED:      // 0 の点滅が終わると自動で初期画面に戻る
    default:            return nullptr;
  }
}

// --- 白地の札 (ボタンの説明と SETUP の見出しに使う) ---
static const int16_t BADGE_PAD = 3;

static int16_t badgeW(const char* text) {
  return (text == nullptr) ? 0 : (spr.textWidth(text, 1) + BADGE_PAD * 2);
}

static int16_t badgeH() {
  return spr.fontHeight(1) + BADGE_PAD * 2;
}

static void drawBadge(const char* text, int16_t x, int16_t y) {
  if (text == nullptr) return;
  const int16_t w = badgeW(text), h = badgeH();
  spr.fillRect(x, y, w, h, TFT_WHITE);
  spr.setTextColor(TFT_BLACK, TFT_WHITE);
  spr.setTextDatum(MC_DATUM);
  spr.drawString(text, x + w / 2, y + h / 2, 1);
}

// ボタンAの行き先は右下に出す
static void drawKeyHint(const char* text) {
  drawBadge(text, W - 2 - badgeW(text), H - 2 - badgeH());
}

// 無制限(∞)の印。フォントに無いので、円を2つ並べて自前で描く。
static const int16_t INF_R  = 5;
static const int16_t INF_W  = INF_R * 4;
static const int16_t INF_CY = 8;   // 上段(フォント2、高さ16)の中心

static void drawInfinity(int16_t rightX, int16_t cy, int16_t r, uint16_t color) {
  const int16_t cx2 = rightX - r;
  const int16_t cx1 = cx2 - r * 2;
  spr.drawCircle(cx1, cy, r,     color);
  spr.drawCircle(cx1, cy, r - 1, color);
  spr.drawCircle(cx2, cy, r,     color);
  spr.drawCircle(cx2, cy, r - 1, color);
}

// 上段右の「立」。分母は繰り返し回数で、0 のときは ∞ を描く。
static int16_t drawRounds(uint16_t fg, uint16_t bg) {
  spr.setTextColor(fg, bg);
  spr.setTextDatum(TR_DATUM);

  int16_t x = W - 4;
  if (repeatOf() == 0) {
    drawInfinity(x, INF_CY, INF_R, fg);
    x -= INF_W;
  } else {
    char den[8];
    snprintf(den, sizeof(den), "%u", (unsigned)repeatOf());
    spr.drawString(den, x, 0, 2);
    x -= spr.textWidth(den, 2);
  }

  char num[12];
  snprintf(num, sizeof(num), "%u/", (unsigned)roundNo);
  spr.drawString(num, x, 0, 2);
  return (W - 4) - (x - spr.textWidth(num, 2));   // 描いた幅
}

// 上段右。MakeUp / ShootOff はその名前、Shooting は「立」を出す。
static int16_t drawTopRight(uint16_t fg, uint16_t bg) {
  const char* tag = mode().tag;
  if (tag == nullptr) return drawRounds(fg, bg);
  spr.setTextColor(fg, bg);
  spr.setTextDatum(TR_DATUM);
  spr.drawString(tag, W - 4, 4, 1);
  return spr.textWidth(tag, 1);
}

// 設定できるところは点滅させる。隠れている間も位置は変わらない。
static bool blinkOff(uint32_t now) {
  return (now / 400) % 2 == 1;
}

// ============================================================ 設定(SETUP)画面
// 黒地に、REPEAT / MODE / TIME / Exit を並べたもの。ボタンBでカーソルが下に移り、
// ボタンAでその項目が進む。カーソルのある項目は黄色で点滅させる。
static const uint16_t COLOR_CURSOR = COLOR_YELLOW;

// カーソルのある項目の色。点滅で消える間は背景色にして、位置だけ残す。
static uint16_t cursorColor(uint32_t now, uint8_t at, uint16_t fg, uint16_t bg) {
  if (setupCursor != at) return fg;
  return blinkOff(now) ? bg : COLOR_CURSOR;
}

// 文字を左から継ぎ足しながら描く
static void drawRun(const char* text, int16_t* x, int16_t y, uint8_t font, uint16_t color, uint16_t bg) {
  spr.setTextColor(color, bg);
  spr.setTextDatum(ML_DATUM);
  spr.drawString(text, *x, y, font);
  *x += spr.textWidth(text, font);
}

static void drawSetupScreen(uint32_t now, uint16_t fg, uint16_t bg) {
  const int16_t lineH = spr.fontHeight(1);

  // 1行目: 見出しと REPEAT
  drawBadge("SETUP", 4, 1);
  const int16_t topCy = 1 + badgeH() / 2;
  int16_t x = 4 + badgeW("SETUP") + 8;
  char num[24];
  snprintf(num, sizeof(num), "REPEAT: %u / [ ", (unsigned)roundNo);
  drawRun(num, &x, topCy, 1, fg, bg);
  const uint16_t repColor = cursorColor(now, CUR_REPEAT, fg, bg);
  if (repeatOf() == 0) {
    const int16_t r = 3;
    if (repColor != bg) drawInfinity(x + r * 4, topCy, r, repColor);
    x += r * 4;
  } else {
    snprintf(num, sizeof(num), "%u", (unsigned)repeatOf());
    drawRun(num, &x, topCy, 1, repColor, bg);
  }
  drawRun(" ]", &x, topCy, 1, fg, bg);

  // 2行目: MODE の見出し / 3行目: 3つの MODE。選んでいるものを [ ] で囲む
  const int16_t modeY = 1 + badgeH() + 3;
  spr.setTextColor(fg, bg);
  spr.setTextDatum(TL_DATUM);
  spr.drawString("MODE", 4, modeY, 1);

  // 選んでいるものを枠で囲む。文字の位置は選び方で動かない。
  const int16_t listY = modeY + lineH + 4 + lineH / 2;   // ML_DATUM 用に中心を渡す
  x = 5;
  for (uint8_t i = 0; i < MODE_COUNT; i++) {
    const bool     here = (i == modeNo);
    const int16_t  w    = spr.textWidth(MODES[i].title, 1);
    if (here) {
      const uint16_t frame = (setupCursor == CUR_MODE) ? COLOR_CURSOR : fg;
      spr.drawRect(x - 3, listY - lineH / 2 - 2, w + 6, lineH + 4, frame);
    }
    const uint16_t color = here ? cursorColor(now, CUR_MODE, fg, bg) : fg;
    drawRun(MODES[i].title, &x, listY, 1, color, bg);
    x += spr.textWidth(" ", 1) + 2;
  }

  // 4行目: TIME。数字の枠は3桁ぶんで固定なので、桁が変わっても [ ] は動かない
  const int16_t timeY = listY + lineH / 2 + 4 + spr.fontHeight(2) / 2;
  x = 4;
  drawRun("TIME [ ", &x, timeY, 2, fg, bg);
  const int16_t slotW = spr.textWidth("000", 2);
  snprintf(num, sizeof(num), "%u", (unsigned)shootingSec());
  spr.setTextColor(cursorColor(now, CUR_TIME, fg, bg), bg);
  spr.setTextDatum(MC_DATUM);
  spr.drawString(num, x + slotW / 2, timeY, 2);
  x += slotW;
  drawRun(" ] sec", &x, timeY, 2, fg, bg);

  // 5行目: 設定を終わる
  const int16_t exitY = timeY + spr.fontHeight(2) / 2 + 3;
  spr.setTextColor(cursorColor(now, CUR_EXIT, fg, bg), bg);
  spr.setTextDatum(TL_DATUM);
  spr.drawString("[ Exit SETUP ]", 4, exitY, 1);
}

// 信号の色。行射だけが緑/黄、設定画面だけが黒で、それ以外は赤。
// 設定画面からスタートしても、ムーブアップは赤に戻る。
static uint16_t signalColor(uint32_t now) {
  if (state == SETTING) return COLOR_BLACK;
  if (state == SHOOTING) {
    return (remainingSec(now) <= WARN_SEC) ? COLOR_YELLOW : COLOR_GREEN;
  }
  return COLOR_RED;
}

static void render(uint32_t now, uint16_t value, bool blank) {
  const int16_t bandH = H - TOP_H - BOTTOM_H;   // 中央の数字を置ける帯

  if (durationSec != sizedFor || bandH != sizedH) {
    sizedFor = durationSec;
    sizedH   = bandH;
    chooseFont(durationSec, bandH);
  }

  const uint16_t bg = signalColor(now);
  const uint16_t fg = (bg == COLOR_YELLOW) ? TFT_BLACK : TFT_WHITE;

  spr.fillSprite(bg);
  spr.setTextColor(fg, bg);

  if (state == SETTING) {
    drawSetupScreen(now, fg, bg);
  } else {
    // 上段右: 「立」または MODE の名前
    drawTopRight(fg, bg);

    // 上段左: 状態。初期画面は代わりに SETUP (ボタンBの行き先) の札を出す
    if (state == READY) {
      drawBadge("SETUP", 4, 1);
    } else if (stateLabel() != nullptr) {
      spr.setTextColor(fg, bg);
      spr.setTextDatum(TL_DATUM);
      spr.drawString(stateLabel(), 4, 0, 2);
    }

    // 中央: 残り秒数。位置は右下のボタンの有無で変わらない
    if (!blank) {
      char text[8];
      snprintf(text, sizeof(text), "%u", (unsigned)value);
      spr.setTextColor(fg, bg);
      spr.setTextDatum(MC_DATUM);
      spr.setTextSize(bigSize);
      spr.drawString(text, W / 2, TOP_H + bandH / 2, bigFont);
      spr.setTextSize(1);
    }

    // 下段左: この立の行射時間。行射中はバーを出すので、その手前まで
    if (state == READY || state == MOVEUP || state == HALT_MOVEUP || state == HALT_SHOOTING) {
      char text[12];
      snprintf(text, sizeof(text), "%usec", (unsigned)shootingSec());
      spr.setTextColor(fg, bg);
      spr.setTextDatum(BL_DATUM);
      spr.drawString(text, 4, H - 2, 2);
    }

    // 下段: 残時間バー (行射中だけ)
    if (state == SHOOTING && durationSec > 0) {
      const int16_t barX = 4, barY = H - BAR_H - BAR_MARGIN;
      const int16_t barW = W - 8;
      spr.drawRect(barX, barY, barW, BAR_H, fg);
      const int16_t fill = (int16_t)((uint32_t)(barW - 2) * remainingMs(now) / totalMs());
      if (fill > 0) spr.fillRect(barX + 1, barY + 1, fill, BAR_H - 2, fg);
    }
  }

  drawKeyHint(keyHintA());   // 右下: ボタンAの行き先

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
  ledBegin();

  RS485.begin(RS485_BAUD, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);
  sendInit();
  RS485.flush();

  modeNo = 0;     // 起動時は Shooting の先頭 (180秒) / 1立 の初期画面から
  toReady();
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
    keyReset(now);                        // B長押し = 中断 / 待機へ戻す / 設定画面
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
    // 表示が変わるときと、表示が止まっていても1秒ごとに、送信開始の宣言を
    // 出す。ケーブルが抜けて挿し直されても、これでタイマーが受信モードに戻る。
    const uint32_t shown = blank ? 0x10000UL : value;   // 消灯も別の「表示」として見る
    static uint32_t lastShown  = 0xFFFFFFFFUL;
    static uint32_t nextInitMs = 0;
    if (shown != lastShown || (int32_t)(now - nextInitMs) >= 0) {
      lastShown  = shown;
      nextInitMs = now + INIT_RESEND_MS;
      sendInit();
    }
    sendDisplay(value, blank);
  }

  // --- 本体側のブザーと赤色LED (画面に出す白枠と同じタイミング) ---
  {
    static bool toneOn = false;
    const bool want = beepActiveAt(now);
    if (want != toneOn) {
      toneOn = want;
      if (want) buzzerOn(); else buzzerOff();
      ledSet(want);
    }
  }

  // --- 描画 (約20Hz) ---
  static uint32_t nextDrawMs = 0;
  if ((int32_t)(now - nextDrawMs) >= 0) {
    nextDrawMs = now + 50;
    render(now, value, blank);
  }
}
