#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
モルテン ハンディータイマーアウトドア UD0040 (RS-485) 表示データ受信・デコード  v3

■ フレーム構造 (可変長)

    02 | ASCIIヘキサ 2N文字 (= データ本体 N バイト) | 03 | BCC 2文字
   STX |                                          | ETX |

  ・生バイトなのは STX(0x02) / ETX(0x03) のみ。中身は「16進数を表すASCII文字」。
    例: 表示バイト 0xBF は 'B','F' の2文字 (0x42 0x46) として流れてくる。
  ・BCC = STXの次から ETX まで(ETXを含む)の XOR を、2桁の ASCII ヘキサにしたもの。
    表示フレームは全32バイト、ブザーフレームは全16バイト。
  ・本体先頭バイトがコマンド種別。長さはコマンドごとに違う。
        0x01 : 表示更新    (本体14バイト / 全31バイト) 約85ms周期で常時送信
        0x0D : ブザー鳴動  (本体 6バイト / 全15バイト) 鳴っている間だけ送信

■ CMD 0x01 (表示更新)
        [0]      0x01        コマンド種別
        [1]-[7]  7セグ7桁分  未使用桁は 0x00(消灯)。最上位ビット(0x80)が小数点。
                             実機が持つのは d3-d7 の5桁で、[1][2] は常に 0x00
                             (時は1桁までなので10時間以上は設定できない)
        [8]      区切り表示  0x02 = 小数点(コロン)を使う時間表示 / 0x00 = 数値・文字表示
        [9]-[11] 0x00        未使用
        [12]     0x80        得点画面(F-5の動作中)でのみ立つ。他は常に 0x00
        [13]     0x00        未使用
  桁の並びは  d1 d2 d3 . d4 d5 . d6 d7  の HHH:MM:SS 形式。小数点が区切り(コロン)で、
  上位の桁はゼロサプレスされる。区切りの数で単位が決まる。
        .. .. .. .. BF 06 3F -> "0.10"    -> 0分10秒
        .. .. .. 06 BF 3F 3F -> "10.00"   -> 10分00秒
        .. .. 86 3F BF 3F 3F -> "1.00.00" -> 1時間00分00秒
  設定メニュー中は英字が出る。 71 40 66 -> "F-4" / 73 40 06 -> "P-1"

■ 機能(F-1〜F-7) と表示の読み方   ※取扱説明書 UD0040 による
        F-1 ストップウォッチ      H:MM:SS  (最大 9:59:59)
        F-2 ダウンカウントタイマー H:MM:SS  (最大 9:59:59、予鈴10回)
        F-3 アップカウントタイマー H:MM:SS  (サッカー90分計モードあり)
        F-4 プログラムタイマー    H:MM:SS  (P-1〜P-9、繰り返し最大99回)
        F-5 得点                 NN-NN    (最大 99対99)  d3d4 が左、d5 が '-'(0x40)、
                                          d6d7 が右。十の位はゼロサプレス
        F-6 時計                 HH:MM    (24時間表示)  ← 分と秒ではない
        F-7 ショットクロック      SS       (最大99秒、既定12秒)
  F-6 の "10.50" は 10分50秒ではなく 10時50分。同じビット列でも機能によって
  意味が変わるので、直前に流れてきた "F-n" を覚えて解釈を切り替える。

■ CMD 0x0D (ブザー)
        0D 00 01 kk 00 00
        [2]  : 鳴動中フラグ (鳴っている間は常に 0x01)
        [3]kk: 音種  0x01 = ブザー1 / 0x00 = ブザー2
  鳴動中のみ、表示パケットの合間に約85ms周期で送られてくる。鳴動長は約1.9秒。

使い方:
    python rcv_timer.py                     # COM3 から受信
    python rcv_timer.py --port COM4         # ポート指定
    python rcv_timer.py --raw               # イベントを1行ずつスクロール表示
    python rcv_timer.py --8seg              # 7セグを液晶風に描画
    python rcv_timer.py --8seg --style half  # 桁がずれる環境では文字セットを変える
    python rcv_timer.py --replay dump.txt   # 実機なしでログを再生してテスト
    python rcv_timer.py --mode F6           # 起動時の機能が分かっている場合に指定
"""

import argparse
import os
import sys
import time

# ---------------------------------------------------------------------------
# 7セグメント デコード (下位7ビットが字形、0x80 が小数点)
# ---------------------------------------------------------------------------
SEG_MAP = {
    0x00: " ",
    0x3F: "0", 0x06: "1", 0x5B: "2", 0x4F: "3", 0x66: "4",
    0x6D: "5", 0x7D: "6", 0x07: "7", 0x7F: "8", 0x6F: "9",
    0x40: "-", 0x08: "_",
    # 設定メニュー等で出る文字 (F と P と - は実データで確認済み)
    0x71: "F", 0x73: "P", 0x77: "A", 0x7C: "b", 0x39: "C", 0x5E: "d",
    0x79: "E", 0x76: "H", 0x38: "L", 0x54: "n", 0x5C: "o", 0x50: "r",
    0x78: "t", 0x3E: "U", 0x6E: "y", 0x30: "I", 0x1C: "u", 0x37: "N",
}

STX = 0x02
ETX = 0x03
BCC_LEN = 2
MAX_BODY_HEX = 64            # 安全弁 (本体32バイト相当)

CMD_DISPLAY = 0x01
CMD_BUZZER = 0x0D

SEPARATOR_MODE = 0x02        # CMD01 [8] がこの値なら時間表示
SCORE_FLAG_INDEX = 12        # CMD01 [12] の bit7 が得点画面フラグ
BUZZER_TIMEOUT = 0.3         # この秒数 0x0D が来なければ鳴動終了とみなす


def decode_digit(value: int):
    dp = bool(value & 0x80)
    return SEG_MAP.get(value & 0x7F, "?"), dp


def decode_groups(body: bytes):
    """
    7桁を区切り(小数点)でグループ分けする。
        00 00 86 3F BF 3F 3F -> ['  1', '00', '00']  (1時間00分00秒)
        00 00 00 06 BF 3F 3F -> ['   10', '00']      (10分00秒)
    """
    groups = []
    current = []
    for value in body[1:8]:
        char, dp = decode_digit(value)
        current.append(char)
        if dp:
            groups.append("".join(current))
            current = []
    groups.append("".join(current))
    return groups


def decode_display(body: bytes) -> str:
    """表示そのままの文字列。例: '0.10' / '1.00.00' / 'F-4'"""
    out = []
    for value in body[1:8]:
        char, dp = decode_digit(value)
        out.append(char)
        if dp:
            out.append(".")
    return "".join(out).strip()


def to_seconds(groups):
    """区切りで分けた表示を秒に換算する。換算できなければ None。"""
    fields = [g.strip() for g in groups]
    if any(f and not f.isdigit() for f in fields):
        return None
    values = [int(f) if f else 0 for f in fields]
    if len(values) == 3:                          # H:MM:SS
        return values[0] * 3600 + values[1] * 60 + values[2]
    if len(values) == 2:
        if len(fields[-1]) == 1:                  # SS.T (1/10秒表示)
            return values[0] + values[1] / 10.0
        return values[0] * 60 + values[1]         # M:SS / MM:SS
    return None


def format_seconds(seconds) -> str:
    if isinstance(seconds, float):
        return f"{seconds:.1f} 秒"
    hours, rest = divmod(int(seconds), 3600)
    minutes, secs = divmod(rest, 60)
    text = f"{hours}:{minutes:02d}:{secs:02d}" if hours else f"{minutes}:{secs:02d}"
    return f"{text}  ({int(seconds)} 秒)"


FUNCTION_NAMES = {
    "F-1": "ストップウォッチ", "F-2": "ダウンカウントタイマー",
    "F-3": "アップカウントタイマー", "F-4": "プログラムタイマー",
    "F-5": "得点", "F-6": "時計", "F-7": "ショットクロック",
}


def describe_display(body: bytes, mode=None) -> str:
    """CMD01 を人間向けの1行にする。mode は直近に表示された 'F-n'。"""
    display = decode_display(body)
    if not display:
        return "(消灯)"

    if display in FUNCTION_NAMES:                  # 機能選択中
        return f"{display} ({FUNCTION_NAMES[display]})"
    if display.replace(" ", "") in ("LO", "L0"):   # 電池残量低下警告
        return f"{display}  ← 電池残量低下"
    if display.startswith("P-"):                   # F-4 のプログラム番号
        return f"{display} (プログラム{display[2:]})"
    if set(display) <= {"-", " "} and "-" in display:
        return f"{display}  (未設定)"

    is_score = mode == "F-5" or (len(body) > SCORE_FLAG_INDEX
                                 and body[SCORE_FLAG_INDEX] & 0x80)
    if is_score:                                   # 得点 NN-NN
        if "-" in display:
            left, right = (x.strip() for x in display.split("-", 1))
            return f"{display:<9}{left or '0'} 対 {right or '0'}"
        return display
    if mode == "F-7":                              # ショットクロック (秒のみ)
        return f"{display:<9}{display.strip()} 秒" if display.isdigit() else display

    if body[8] == SEPARATOR_MODE:
        groups = decode_groups(body)
        if mode == "F-6":                          # 時計 HH:MM
            fields = [g.strip() for g in groups]
            if len(fields) == 2 and all(f.isdigit() or not f for f in fields):
                hour = int(fields[0] or 0)
                minute = int(fields[1] or 0)
                return f"{display:<9}{hour}時{minute:02d}分"
        seconds = to_seconds(groups)
        if seconds is not None:
            return f"{display:<9}{format_seconds(seconds)}"
    return display


BUZZER_KIND = {0x01: "ブザー1", 0x00: "ブザー2"}


# ---------------------------------------------------------------------------
# 液晶風の7セグメント描画 (--8seg)
# ---------------------------------------------------------------------------
# 点灯/消灯に使う文字のプリセット。
# 桁をそろえるには、両者の East Asian Width が同じ字である必要がある。
#   ■ (U+25A0) は "A"(Ambiguous) なので、PowerShell の既定フォントでは半角1セル、
#   MS ゴシック等では全角2セルになる。一方 全角スペース(U+3000) は "F" で常に2セル。
#   この組み合わせだと環境によって桁がずれるため、既定は "W"(Wide) 固定の ⬛ を使う。
SEG_STYLES = {
    # ■(U+25A0) と □(U+25A1) はどちらも EAW="A"。同じ幅クラス同士なので、
    # フォントが半角に描いても全角に描いても必ずそろう。既定。
    "square": ("■", "□"),
    # ￭(U+FFED) は文字どおりの半角黒四角 EAW="H"。半角スペース(Na)と組んで1セル。
    "half":   ("￭", " "),
    "shade":  ("█", "▓"),      # どちらも EAW="A"
    "wide":   ("⬛", "　"),     # どちらも2セル固定 (W + F)
    "double": ("■■", "　"),    # ■を2つ並べて2セルにする
    "ascii":  ("#", "."),      # 半角のみ。フォント非依存
}
SEG_ON, SEG_OFF = SEG_STYLES["square"]

# セグメントのビット割り当て
#        a(0x01)
#   f(0x20)  b(0x02)
#        g(0x40)
#   e(0x10)  c(0x04)
#        d(0x08)      dp(0x80)
SEG_A, SEG_B, SEG_C, SEG_D, SEG_E, SEG_F, SEG_G, SEG_DP = (
    0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80)


def render_seven_segment(values, on=SEG_ON, off=SEG_OFF):
    """
    7セグのバイト列を5行のブロック文字に描く。1桁ぶんは 4列 + 空き1列 + 小数点1列。

        ■■■■
        ■　　■
        ■■■■
        ■　　■
        ■■■■　■
    """
    def bar(bit, value):
        return (on if value & bit else off) * 4

    def sides(left_bit, right_bit, value):
        return ((on if value & left_bit else off) + off * 2
                + (on if value & right_bit else off))

    def middle(value):
        # 中段は g セグメントの行。g が消えているときは、上下の縦棒が
        # つながって見えるように f/e と b/c を描く。
        if value & SEG_G:
            return on * 4
        left = on if value & (SEG_F | SEG_E) == (SEG_F | SEG_E) else off
        right = on if value & (SEG_B | SEG_C) == (SEG_B | SEG_C) else off
        return left + off * 2 + right

    rows = ["", "", "", "", ""]
    for value in values:
        cell = [
            bar(SEG_A, value),
            sides(SEG_F, SEG_B, value),
            middle(value),
            sides(SEG_E, SEG_C, value),
            bar(SEG_D, value),
        ]
        dot = on if value & SEG_DP else off
        for i in range(5):
            rows[i] += cell[i] + off + (dot if i == 4 else off)
    return rows


def render_screen(body, header, footer, all_digits=False, on=SEG_ON, off=SEG_OFF):
    """表示パケットから、画面まるごとの文字列を組み立てる。"""
    if body is None:
        digits = [0x00] * 5
    else:
        # 実機は d3-d7 の5桁。上位2桁が使われている機種では7桁すべて描く。
        start = 1 if (all_digits or body[1] or body[2]) else 3
        digits = list(body[1:8][start - 1:])
    return [header, ""] + render_seven_segment(digits, on, off) + ["", footer]


def describe_buzzer(body: bytes) -> str:
    if len(body) < 4 or not body[2]:
        return "ブザー停止"
    kind = BUZZER_KIND.get(body[3], f"種別0x{body[3]:02X}")
    return f"ブザー鳴動中 ({kind})"


# ---------------------------------------------------------------------------
# パケット組み立て
# ---------------------------------------------------------------------------
def calc_bcc(body_ascii: bytes) -> int:
    bcc = 0
    for byte in body_ascii:
        bcc ^= byte
    return bcc


def verify_bcc(body_ascii: bytes, received: bytes) -> bool:
    """
    BCC を検証する。
    BCC = XOR(STXの次 〜 ETX まで、ETXを含む) を2桁のASCIIヘキサにしたもの。
    無損失ログ406レコードで確定済み。
    旧 rcv_data.py のログは上位1文字しか残っていないので、その場合は上位のみ照合。
    """
    if not received:
        return True
    expected = f"{calc_bcc(body_ascii) ^ ETX:02X}"
    text = received.decode("ascii", "replace").upper()
    if len(text) >= 2:
        return text[:2] == expected
    return text[:1] == expected[:1]


class PacketParser:
    """1バイトずつ食わせると、完成したパケットを返すステートマシン。"""

    def __init__(self):
        self.reset()

    def reset(self):
        self.buffer = bytearray()
        self.bcc = bytearray()
        self.in_frame = False
        self.after_etx = False

    def feed(self, byte: int):
        """完成したら (本体bytes, 本体ASCII, BCC) を返す。"""
        if self.after_etx:
            # ETX の直後は BCC を規定文字数だけ集める。
            # 途中で STX が来た場合は BCC が1文字の機種なので打ち切って再スタート。
            if byte == STX:
                result = self._finish()
                self.buffer = bytearray()
                self.bcc = bytearray()
                self.in_frame = True
                self.after_etx = False
                return result
            self.bcc.append(byte)
            if len(self.bcc) >= BCC_LEN:
                result = self._finish()
                self.reset()
                return result
            return None

        if byte == STX:                 # ノイズや取りこぼしからの復帰も兼ねる
            self.buffer = bytearray()
            self.bcc = bytearray()
            self.in_frame = True
            return None

        if not self.in_frame:
            return None

        if byte == ETX:
            self.after_etx = True
            return None

        self.buffer.append(byte)
        if len(self.buffer) > MAX_BODY_HEX:
            self.reset()
        return None

    def flush(self):
        """入力終了時、ETX まで受信済みのパケットを取り出す (replay 用)。"""
        if self.after_etx:
            result = self._finish()
            self.reset()
            return result
        return None

    def _finish(self):
        body_ascii = bytes(self.buffer)
        bcc = bytes(self.bcc)
        self.after_etx = False
        if len(body_ascii) < 2 or len(body_ascii) % 2:
            return None
        try:
            body = bytes.fromhex(body_ascii.decode("ascii"))
        except (ValueError, UnicodeDecodeError):
            return None
        return body, body_ascii, bcc


# ---------------------------------------------------------------------------
# 表示処理
# ---------------------------------------------------------------------------
class Monitor:
    def __init__(self, args):
        self.args = args
        self.mode = args.mode
        self.display = None
        self.buzzer = None
        self.buzzer_at = 0.0
        self.buzzer_start = 0.0
        self.bcc_ng = 0
        self.unknown = 0
        self.last_body = None          # 直近の表示パケット (--8seg の再描画用)
        self.screen_drawn = False
        self.on, self.off = SEG_STYLES[args.style]
        if args.on is not None:
            self.on = args.on
        if args.off is not None:
            self.off = args.off

    def handle(self, packet, now):
        body, body_ascii, bcc = packet
        if not verify_bcc(body_ascii, bcc):
            self.bcc_ng += 1
            if self.args.raw:
                self.emit(f"[BCC NG] {body_ascii.decode('ascii', 'replace')}")
            return

        cmd = body[0]
        if cmd == CMD_DISPLAY and len(body) >= 9:
            raw = decode_display(body)
            if raw in FUNCTION_NAMES:          # 機能が切り替わったので解釈も追従する
                self.mode = raw
            text = describe_display(body, self.mode)
            changed = text != self.display or self.last_body != body
            self.last_body = body
            if text != self.display or self.args.all or (self.args.seg8 and changed):
                self.display = text
                self.emit_state(body)
        elif cmd == CMD_BUZZER:
            state = describe_buzzer(body)
            if state != self.buzzer:
                if self.buzzer is None:
                    self.buzzer_start = now
                self.buzzer = state
                self.emit_state(body)
            self.buzzer_at = now
        else:
            self.unknown += 1
            if self.args.raw:
                self.emit(f"[未知のコマンド 0x{cmd:02X}] {body.hex(' ').upper()}")

    def tick(self, now):
        """ブザーパケットが途切れたら鳴動終了とみなす。"""
        if self.buzzer and now - self.buzzer_at > BUZZER_TIMEOUT:
            length = self.buzzer_at - self.buzzer_start
            self.buzzer = None
            self.emit(f"ブザー停止 (鳴動 {length:.2f} 秒)" if self.args.raw else None)
            self.emit_state(None)

    def emit_state(self, body):
        if self.args.seg8:
            self.draw_screen()
            return
        label = f"[ {self.mode or 'タイマー'} ] "
        line = label + f"{self.display or '----':<26}"
        if self.buzzer:
            line += "  ♪ " + self.buzzer
        if self.args.raw and body is not None:
            line += "   | " + body.hex(" ").upper()
        self.emit(line)

    def draw_screen(self):
        """液晶風に画面全体を描き直す。"""
        function = FUNCTION_NAMES.get(self.mode)
        header = f"  {self.mode or '機能不明'}" + (f"  {function}" if function else "")
        footer = "  " + (self.display or "----")
        if self.buzzer:
            footer += "    ♪ " + self.buzzer
        lines = render_screen(self.last_body, header, footer,
                              self.args.all_digits, self.on, self.off)
        body = "\n".join(line.rstrip() + "\033[K" for line in lines)
        if self.screen_drawn:
            sys.stdout.write(f"\033[{len(lines)}A")     # 描いた行数だけ戻る
        else:
            self.screen_drawn = True
        sys.stdout.write(body + "\n")
        sys.stdout.flush()

    def emit(self, line):
        if line is None:
            return
        if self.args.raw:
            print(line)
        else:
            print(f"\r{line:<70}", end="", flush=True)


# ---------------------------------------------------------------------------
# 入力
# ---------------------------------------------------------------------------
def run_serial(args, monitor):
    import serial

    ser = serial.Serial(
        port=args.port,
        baudrate=args.baud,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        rtscts=args.rtscts,
        timeout=0.05,
    )
    print(f"{args.port} でタイマーデータの受信を待機中... (Ctrl+C で終了)")

    parser = PacketParser()
    try:
        while True:
            chunk = ser.read(ser.in_waiting or 1)
            now = time.monotonic()
            for byte in chunk:
                packet = parser.feed(byte)
                if packet:
                    monitor.handle(packet, now)
            monitor.tick(now)
    except KeyboardInterrupt:
        print("\n終了します。")
    finally:
        ser.close()


def run_replay(args, monitor):
    """rcv_data.py が保存したログを読み込んで、実機なしで動作確認する。"""
    import re
    from datetime import datetime

    parser = PacketParser()
    pattern = re.compile(r"\[(.*?)\]\s*(.*)")
    base = None
    now = 0.0
    with open(args.replay, encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            match = pattern.match(line)
            if match:
                stamp = datetime.strptime(match.group(1), "%Y-%m-%d %H:%M:%S.%f")
                base = base or stamp
                now = (stamp - base).total_seconds()
                hex_part = match.group(2)
            else:
                hex_part = line
            try:
                data = bytes(int(x, 16) for x in hex_part.split())
            except ValueError:
                continue
            monitor.tick(now)
            for byte in data:
                packet = parser.feed(byte)
                if packet:
                    monitor.handle(packet, now)
            if args.delay:
                time.sleep(args.delay)
    packet = parser.flush()
    if packet:
        monitor.handle(packet, now)
    monitor.tick(now + BUZZER_TIMEOUT + 1)
    print()


def enable_ansi():
    """Windows のコンソールで ANSI エスケープを有効にする。"""
    if os.name != "nt":
        return
    try:
        import ctypes
        kernel32 = ctypes.windll.kernel32
        handle = kernel32.GetStdHandle(-11)
        mode = ctypes.c_uint32()
        kernel32.GetConsoleMode(handle, ctypes.byref(mode))
        kernel32.SetConsoleMode(handle, mode.value | 0x0004)
    except Exception:
        os.system("")


def main():
    ap = argparse.ArgumentParser(description="モルテン タイマー表示 受信プログラム")
    ap.add_argument("--port", default="COM3", help="シリアルポート (既定: COM3)")
    ap.add_argument("--baud", type=int, default=9600, help="ボーレート (既定: 9600)")
    ap.add_argument("--no-rtscts", dest="rtscts", action="store_false",
                    help="RTS/CTS フロー制御を使わない")
    ap.add_argument("--raw", action="store_true", help="イベントを1行ずつ生データ付きで表示")
    ap.add_argument("--all", action="store_true", help="変化のないパケットも出力")
    ap.add_argument("--replay", metavar="LOGFILE", help="ログファイルを再生 (動作確認用)")
    ap.add_argument("--delay", type=float, default=0.0, help="再生時のウェイト秒")
    ap.add_argument("--8seg", dest="seg8", action="store_true",
                    help="7セグ表示を液晶風のブロック文字で描画する")
    ap.add_argument("--style", choices=list(SEG_STYLES), default="square",
                    help="--8seg の文字セット (既定: square = ■/□)。"
                         "ずれる場合は half / ascii を試す")
    ap.add_argument("--on", default=None, help="点灯セグメントの文字を直接指定")
    ap.add_argument("--off", default=None, help="消灯セグメントの文字を直接指定")
    ap.add_argument("--all-digits", action="store_true",
                    help="--8seg で未使用の上位2桁も含めて7桁すべて描画する")
    ap.add_argument("--mode", choices=sorted(FUNCTION_NAMES), default=None,
                    help="開始時の機能 (F-1〜F-7)。以後 F-n を受信すると自動追従")
    args = ap.parse_args()

    if args.seg8:
        enable_ansi()
    monitor = Monitor(args)
    if args.replay:
        run_replay(args, monitor)
    else:
        run_serial(args, monitor)
    if monitor.bcc_ng or monitor.unknown:
        print(f"BCCエラー {monitor.bcc_ng} 件 / 未知コマンド {monitor.unknown} 件",
              file=sys.stderr)


if __name__ == "__main__":
    main()
