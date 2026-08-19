#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
モルテン ハンディータイマー (UD0040) へのシリアルデータ送信プログラム

通信設定は仕様どおり固定:
    EIA RS-485準拠 / 半二重 / 調歩同期 / 9600bps / スタート1 / データ8 / パリティ無 / ストップ1

■ フレーム構造
    STX(0x02) + 本体を16進にしたASCII文字列 + ETX(0x03) + BCC 2文字
    BCC = XOR(STXの次 〜 ETXまで、ETXを含む) を2桁の大文字16進にしたもの

■ 本体
    0E 00 01                              送信開始の宣言 (INIT / 全10バイト)
    01 [7セグ7桁] [区切り] 000 [得点] 0    表示更新 (全32バイト)
    0D 00 01 [音種] 00 00                 ブザー鳴動 (全16バイト)

  7セグは dp-g-f-e-d-c-b-a。最上位ビットが小数点で、時間表示では区切り(コロン)。
  桁の並びは d1 d2 d3 . d4 d5 . d6 d7 の HHH:MM:SS 形式 (実機は右5桁)。

使い方:
    python snd_data.py --time 3:00                  # 3:00 を表示し続ける
    python snd_data.py --time 1:02:03
    python snd_data.py --score 12-3                 # 得点 12対3
    python snd_data.py --text "F-4"                 # 表示文字を直接指定
    python snd_data.py --buzzer 1                   # ブザーを鳴らす
    python snd_data.py --countdown 3:00 --buzzer-at-end 1
    python snd_data.py --init-only                  # INIT だけ送る
    python snd_data.py capture_rec.txt              # 記録した通信をそのまま再生
    python snd_data.py --frame "02 ... 03 30 35"    # 16進を直接送る

  送信の最初に INIT フレームを1回自動で送ります (--no-init で抑止)。
  実機は静止画面でも 85ms ごとに送り続けているので、既定では送出を継続します。
  Ctrl+C で停止、--duration で秒数を指定できます。
"""

import argparse
import os
import sys
import time

# --- 通信設定 (仕様どおり固定) ---
BAUDRATE = 9600
STX = 0x02
ETX = 0x03
FRAME_INTERVAL = 0.085           # 実機の送信周期

# --- コマンド ---
CMD_DISPLAY = 0x01
CMD_BUZZER = 0x0D
CMD_INIT = 0x0E
INIT_BODY = bytes([CMD_INIT, 0x00, 0x01])

DIGITS = 7                       # プロトコル上の桁数 (実機が持つのは右5桁)
SEPARATOR_MODE = 0x02            # [8] 区切り(コロン)を使う時間表示
SCORE_FLAG = 0x80                # [12] 得点画面フラグ

# 7セグメントの字形 (下位7ビット。0x80 が小数点)
CHAR_TO_SEG = {
    " ": 0x00,
    "0": 0x3F, "1": 0x06, "2": 0x5B, "3": 0x4F, "4": 0x66,
    "5": 0x6D, "6": 0x7D, "7": 0x07, "8": 0x7F, "9": 0x6F,
    "-": 0x40, "_": 0x08,
    "A": 0x77, "b": 0x7C, "c": 0x58, "C": 0x39, "d": 0x5E,
    "E": 0x79, "F": 0x71, "G": 0x3D, "h": 0x74, "H": 0x76,
    "i": 0x04, "I": 0x30, "J": 0x0E, "L": 0x38, "n": 0x54,
    "O": 0x3F, "o": 0x5C, "P": 0x73, "q": 0x67, "r": 0x50,
    "S": 0x6D, "t": 0x78, "u": 0x1C, "U": 0x3E, "V": 0x7E,
    "W": 0x6A, "y": 0x6E, "Z": 0x49,
}

# 表示のデコード(逆引き)用。segment値 -> 文字。
# O(0x3F)=0 / S(0x6D)=5 のように数字と同じ形になる文字があるため、
# 数字を必ず優先させる (末勝ちの辞書内包表記を、数字を最後に上書きする順で作る)。
_LETTERS_FIRST = {k: v for k, v in CHAR_TO_SEG.items() if not k.isdigit()}
_DIGITS_LAST = {k: v for k, v in CHAR_TO_SEG.items() if k.isdigit()}
SEG_TO_CHAR = {v: k for k, v in {**_LETTERS_FIRST, **_DIGITS_LAST}.items()}


# ---------------------------------------------------------------------------
# フレームの組み立て
# ---------------------------------------------------------------------------
def calc_bcc(data: bytes) -> int:
    bcc = 0
    for byte in data:
        bcc ^= byte
    return bcc


def build_frame(body: bytes) -> bytes:
    """本体バイト列から、送出できる1フレームを組み立てる。"""
    ascii_body = body.hex().upper().encode("ascii")
    bcc = calc_bcc(ascii_body) ^ ETX
    return bytes([STX]) + ascii_body + bytes([ETX]) + f"{bcc:02X}".encode("ascii")


def check_frame(data: bytes):
    """フレームのBCCを検証する。(判定, 説明) を返す。"""
    if not data or data[0] != STX:
        return False, "STXで始まっていない"
    if ETX not in data:
        return False, "ETXが無い"
    index = data.index(ETX)
    received = data[index + 1:].decode("ascii", "replace").upper()
    if not received:
        return False, "BCCが無い"
    expected = f"{calc_bcc(data[1:index]) ^ ETX:02X}"
    if len(received) >= 2:
        return received[:2] == expected, f"BCC={received[:2]} 期待={expected}"
    return received[0] == expected[0], f"BCC上位={received[0]} 期待={expected}"


def encode_cells(cells):
    """
    [(文字, 小数点), ...] を右詰めで7バイトの7セグ列にする。
    その文字が無ければ大文字/小文字を入れ替えて探し、それでも無ければ
    "_" (0x08) で代用する (エラーにはしない)。
    """
    if len(cells) > DIGITS:
        raise ValueError(f"桁数が多すぎます ({len(cells)} > {DIGITS})")
    out = [0x00] * (DIGITS - len(cells))
    for char, dot in cells:
        seg = CHAR_TO_SEG.get(char)
        if seg is None:
            seg = CHAR_TO_SEG.get(char.swapcase())     # 大文字/小文字のペアで代替
        if seg is None:
            seg = CHAR_TO_SEG["_"]                     # それでも無ければ "_"
        out.append(seg | (0x80 if dot else 0x00))
    return out


def cells_from_text(text: str):
    """'1.00.00' や 'F-4' を桁のリストにする。'.' は直前の桁の小数点。"""
    cells = []
    for char in text:
        if char == ".":
            if not cells:
                raise ValueError("先頭に小数点は置けません")
            cells[-1] = (cells[-1][0], True)
        else:
            cells.append((char, False))
    return cells


def cells_from_seconds(total_seconds: int):
    """
    秒数を実機と同じ桁割りにする。上位桁はゼロサプレスされる。
      59分59秒まで : M:SS / MM:SS   (分の桁に区切り)
      1時間以上    : H:MM:SS        (時と分の桁に区切り)
    """
    total_seconds = int(total_seconds)
    if not 0 <= total_seconds <= 9 * 3600 + 59 * 60 + 59:
        raise ValueError("0秒〜9:59:59 の範囲で指定してください")
    hours, rest = divmod(total_seconds, 3600)
    minutes, seconds = divmod(rest, 60)

    cells = []
    if hours:
        cells.append((str(hours), True))            # 時 (1桁) + 区切り
        text = f"{minutes:02d}"
    else:
        text = str(minutes)                         # 分は前ゼロを詰めない
    for i, char in enumerate(text):
        cells.append((char, i == len(text) - 1))    # 分の最下位に区切り
    cells += [(char, False) for char in f"{seconds:02d}"]
    return cells


def cells_from_plain_number(n: int):
    """区切り(コロン)を使わない、数字だけの表示。ショットクロックと同じ形式。"""
    if not 0 <= n <= 99999:
        raise ValueError("0〜99999 の範囲で指定してください")
    return [(c, False) for c in str(n)]


def cells_from_score(left: int, right: int):
    """得点表示 NN-NN。十の位はゼロサプレス。"""
    for value in (left, right):
        if not 0 <= value <= 99:
            raise ValueError("得点は 0〜99 です")
    cells = [(c, False) for c in f"{left:2d}"]
    cells.append(("-", False))
    cells += [(c, False) for c in f"{right:2d}"]
    return cells


def build_display_body(cells, separator=False, score=False):
    body = bytearray(14)
    body[0] = CMD_DISPLAY
    body[1:8] = bytes(encode_cells(cells))
    body[8] = SEPARATOR_MODE if separator else 0x00
    body[12] = SCORE_FLAG if score else 0x00
    return bytes(body)


def frame_for_init() -> bytes:
    """送信開始の宣言フレーム。02 30 45 30 30 30 31 03 37 37"""
    return build_frame(INIT_BODY)


def frame_for_seconds(total_seconds: int) -> bytes:
    return build_frame(build_display_body(cells_from_seconds(total_seconds),
                                          separator=True))


def frame_for_plain_seconds(n: int) -> bytes:
    """
    'M:SS' のような区切りを使わず、秒数をそのまま数字で表示するフレーム。
    120秒を渡すと '2.00'(2分00秒)ではなく '120' とそのまま表示する。
    """
    return build_frame(build_display_body(cells_from_plain_number(n), separator=False))


def frame_for_blank() -> bytes:
    """全消灯 (空白) の表示フレーム。0秒点滅の消灯側に使う。"""
    return build_frame(build_display_body([]))


def frame_for_score(left: int, right: int) -> bytes:
    return build_frame(build_display_body(cells_from_score(left, right),
                                          score=True))


def frame_for_text(text: str) -> bytes:
    cells = cells_from_text(text)
    separator = any(dot for _, dot in cells)
    return build_frame(build_display_body(cells, separator=separator))


def frame_for_buzzer(kind: int) -> bytes:
    """kind: 0x01=ブザー1 / 0x00=ブザー2"""
    return build_frame(bytes([CMD_BUZZER, 0x00, 0x01, kind, 0x00, 0x00]))


# ---------------------------------------------------------------------------
# 入力の解釈
# ---------------------------------------------------------------------------
def parse_time(text: str) -> int:
    """'3:00' / '1:02:03' / '180' を秒数にする。"""
    text = text.strip()
    if ":" not in text:
        return int(text)
    parts = [int(p) for p in text.split(":")]
    if len(parts) == 2:
        return parts[0] * 60 + parts[1]
    if len(parts) == 3:
        return parts[0] * 3600 + parts[1] * 60 + parts[2]
    raise ValueError(f"時間の書き方が不正です: {text}")


def parse_score(text: str):
    """'12-3' / '12:3' / '12 3' を (左, 右) にする。"""
    for sep in ("-", ":", ",", " "):
        if sep in text:
            left, right = text.split(sep, 1)
            return int(left), int(right)
    raise ValueError(f"得点の書き方が不正です: {text} (例: 12-3)")


def load_records(path):
    """record_data.py の <経過秒>\\t<16進> を [(経過秒, bytes), ...] にする。"""
    records = []
    header = []
    with open(path, encoding="utf-8") as f:
        for lineno, line in enumerate(f, 1):
            line = line.strip()
            if not line:
                continue
            if line.startswith("#"):
                header.append(line)
                continue
            parts = line.split("\t") if "\t" in line else line.split(None, 1)
            try:
                records.append((float(parts[0]),
                                bytes.fromhex(parts[1].replace(" ", ""))))
            except (ValueError, IndexError):
                print(f"  {lineno}行目を読み飛ばしました: {line[:40]}", file=sys.stderr)
    return records, header


# ---------------------------------------------------------------------------
# アーチェリー用タイマー (--timer) : キーボード操作の対話モード
# ---------------------------------------------------------------------------
# --timer "ムーブアップ秒,シューティング秒,黄色警告秒" の3つで1ラウンド(「立」)を定義する。
#   例: --timer "10,180,30"
#       ムーブアップ 10秒 (信号=赤)
#       → シューティング 180秒 (信号=緑、残り30秒を切ったら信号=黄)
#   これを --repeat で指定した回数(立)繰り返す。
#
# 操作:
#   Space  : 「開始」(初回、またはムーブアップ中断からの再開) /
#            「再開」(シューティング中断からの再開、1声鳴らしてシューティングに戻る)。
#            実行中は何もしない (一時停止機能は無し)。
#   右矢印 : 早送り。現在のラウンドを終了扱いにして3声鳴らし、次のラウンドへ進む。
#            全ラウンド終了後(中断表示)に押すと最初から読み込み直す。
#   Enter  : 中断。シューティング中なら時間をそのまま停止、ムーブアップ中なら
#            ムーブアップの最初に戻す。いずれも表示は「中断」/信号=赤にして5声鳴らす。
#   Ctrl+C : 終了
#
# ブザーの単位は「0.7秒鳴動→0.3秒インターバル」で共通。回数だけ場面ごとに違う。
#   ムーブアップ開始(予告) : 2音
#   シューティング開始     : 1音
#   再開 (Enter中断から)   : 1音
#   早送り / 全ラウンド終了 : 3音
#   中断 (Enterキー)       : 5音
# 音色は --buzzer で指定した1種類 (0=鳴らさない。ただし合図の"タイミング"自体は
# 画面表示 [*Beep] や0秒点滅として残る) を全ての合図で共通に使う。
BEEP_ON = 0.7
BEEP_OFF = 0.3


def beep_pattern(count: int):
    """0.7秒鳴動→0.3秒インターバルを count 回繰り返すパターンを作る。"""
    pattern = []
    for _ in range(count):
        pattern += [(True, BEEP_ON), (False, BEEP_OFF)]
    return pattern


BUZZER_READY = beep_pattern(2)     # ムーブアップ開始の予告
BUZZER_START = beep_pattern(1)     # シューティング開始 / 再開
BUZZER_SKIP = beep_pattern(3)      # 早送り (このラウンドを終えて次へ進む合図)
BUZZER_FINISH = beep_pattern(3)    # 全ラウンド終了
BUZZER_ALARM = beep_pattern(5)     # Enterキーによる中断


def on_intervals(pattern):
    """(鳴動するか,秒数) の並びを、鳴動している時間帯 [(開始,終了), ...] にする。"""
    t = 0.0
    result = []
    for on, length in pattern:
        if on:
            result.append((t, t + length))
        t += length
    return result


class KeyPoll:
    """
    キー入力を非ブロッキングで拾う。押されたキーを 'SPACE'/'ENTER'/'RIGHT' の
    いずれかの文字列にして返す。それ以外のキーは無視する。
    Windows は msvcrt、それ以外は termios を使う。POSIX では端末モードを
    元に戻す必要があるので with 文で使う。
    """

    def __init__(self):
        self.windows = (os.name == "nt")
        self._old_settings = None
        self._fd = None

    def __enter__(self):
        if not self.windows:
            import termios
            import tty
            self._fd = sys.stdin.fileno()
            self._old_settings = termios.tcgetattr(self._fd)
            tty.setcbreak(self._fd)
        return self

    def __exit__(self, *exc):
        if not self.windows and self._old_settings is not None:
            import termios
            termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old_settings)

    def poll(self):
        """今たまっているキー入力を全部読み、意味のあるものだけリストで返す。"""
        if self.windows:
            return self._poll_windows()
        return self._poll_posix()

    def _poll_windows(self):
        import msvcrt
        keys = []
        while msvcrt.kbhit():
            ch = msvcrt.getwch()
            if ch in ("\x00", "\xe0"):            # 矢印キー等の拡張キーの前置き
                ch2 = msvcrt.getwch()
                if ch2 == "M":                      # 右矢印
                    keys.append("RIGHT")
            elif ch == " ":
                keys.append("SPACE")
            elif ch in ("\r", "\n"):
                keys.append("ENTER")
        return keys

    def _poll_posix(self):
        import select
        keys = []
        while select.select([sys.stdin], [], [], 0)[0]:
            ch = sys.stdin.read(1)
            if ch == " ":
                keys.append("SPACE")
            elif ch in ("\r", "\n"):
                keys.append("ENTER")
            elif ch == "\x1b" and select.select([sys.stdin], [], [], 0)[0]:
                ch2 = sys.stdin.read(1)
                if ch2 == "[" and select.select([sys.stdin], [], [], 0)[0]:
                    if sys.stdin.read(1) == "C":     # ESC [ C = 右矢印
                        keys.append("RIGHT")
        return keys


# 信号の色 (ANSI)。●自体は常に同じ1文字なので、色を変えても桁はずれない。
_ANSI_RED = "\x1b[31m●\x1b[0m"
_ANSI_GREEN = "\x1b[32m●\x1b[0m"
_ANSI_YELLOW = "\x1b[33m●\x1b[0m"


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


class TimerEngine:
    """
    アーチェリー用タイマーの状態機械。図(ボタンと状態遷移)のとおりに実装している。

    状態:
        開始前(PRE_START)         : 読み込み直後。ムーブアップの秒数を表示して待機
        ムーブアップ(MOVEUP)       : 実行中。入場時 2声
        行射(SHOOTING)            : 実行中。色は残り時間で自動的に緑/黄
                                     入場時、残り>黄色警告秒なら1声、すでに黄色域なら無音
        中断(INTERRUPTED_MOVEUP)  : ムーブアップ中にEnter。入場5声。値は中断した瞬間のまま
        中断(INTERRUPTED_SHOOTING): 行射中にEnter。入場5声。値は中断した瞬間のまま(時間停止)
        行射終了(FINISHED)        : 全ラウンド終了。入場3声、0/空白の点滅つき

    遷移 (図の矢印そのまま):
        開始前         --[Space]--> ムーブアップ (2声)
        ムーブアップ    --(自動,時間切れ)--> 行射 (1声)
        ムーブアップ    --[Enter]--> 中断(ムーブアップ) (5声)
        ムーブアップ    --[→]--> ラウンド完了処理
        行射           --(自動,時間切れ)--> ラウンド完了処理
        行射           --[Enter]--> 中断(行射) (5声、時間はそのまま停止)
        行射           --[→]--> ラウンド完了処理
        中断(ムーブアップ) --[Space]--> 開始前 (無音。ここでムーブアップの長さにリセットされる)
        中断(行射)         --[Space]--> 行射 (再開。色に応じて1声/無音)
        行射終了       --[Space]--> 開始前 (無音、ラウンド数も1に戻る)
        (上記以外のキーは、その状態では何も起きない)

        ラウンド完了処理: まだ繰り返しが残っていればムーブアップへ(2声)、
                          最後のラウンドなら行射終了へ(3声)。
                          早送りで来ても自然終了で来ても、この処理は同じ。
    """

    PRE_START = "PRE_START"
    MOVEUP = "MOVEUP"
    SHOOTING = "SHOOTING"
    INTERRUPTED_MOVEUP = "INTERRUPTED_MOVEUP"
    INTERRUPTED_SHOOTING = "INTERRUPTED_SHOOTING"
    FINISHED = "FINISHED"

    def __init__(self, moveup, shooting, warn, repeat, buzzer_kind):
        self.moveup_duration = moveup
        self.shooting_duration = shooting
        self.warn = warn
        self.repeat = repeat
        self.buzzer_kind = buzzer_kind         # None (無音) / 0x01 / 0x00
        self.round = 1                         # 「立」。1〜repeat の範囲に収まる
        self.state = self.PRE_START
        self.duration = moveup                 # 表示用。開始前はムーブアップの長さを見せる
        self.elapsed = 0.0
        self.run_origin = None                 # None = 停止中 (経過時間は elapsed に固定)
        self.beep_anchor = None
        self.beep_intervals = []
        self.beep_frame = None
        self.beep_blink = False
        self.blink_intervals = []
        self.blink_total = 0.0
        self._display_cache = {}
        self._blank_frame = frame_for_blank()

    # --- 経過時間 ---------------------------------------------------------
    def current_elapsed(self, now):
        if self.run_origin is not None:
            return self.elapsed + (now - self.run_origin)
        return self.elapsed

    def remaining(self, now):
        return max(0, self.duration - int(self.current_elapsed(now)))

    # --- 合図 ---------------------------------------------------------
    def trigger_beep(self, pattern, kind, now, blink=False):
        self.beep_anchor = now
        # 鳴動する時間帯そのものは、音を出す/出さない (--buzzer 0) に関わらず常に持っておく。
        # 「本来鳴るはずの時間帯」を画面表示 (*Beep) と点滅の両方で使うため。
        self.beep_intervals = on_intervals(pattern)
        self.beep_frame = frame_for_buzzer(kind) if kind is not None else None
        self.beep_blink = blink
        self.blink_intervals = on_intervals(pattern) if blink else []
        self.blink_total = self.blink_intervals[-1][1] if self.blink_intervals else 0.0

    def silence(self):
        """予定されていた合図を打ち切って無音にする。"""
        self.beep_anchor = None
        self.beep_intervals = []
        self.beep_frame = None
        self.beep_blink = False
        self.blink_intervals = []
        self.blink_total = 0.0

    def beep_frame_at(self, now):
        """実際に送信するブザーフレーム。--buzzer 0 のときは常に None。"""
        if self.beep_frame is None or self.beep_anchor is None:
            return None
        rel = now - self.beep_anchor
        for start, end in self.beep_intervals:
            if start <= rel < end:
                return self.beep_frame
        return None

    def beep_active_at(self, now):
        """--buzzer 0 でも、本来なら鳴っているはずの時間帯かどうか (画面表示用)。"""
        if self.beep_anchor is None:
            return False
        rel = now - self.beep_anchor
        return any(start <= rel < end for start, end in self.beep_intervals)

    # --- 状態遷移の内部処理 ---------------------------------------------------------
    def _start_moveup(self, now):
        self.state = self.MOVEUP
        self.duration = self.moveup_duration
        self.elapsed = 0.0
        self.run_origin = now
        self.trigger_beep(BUZZER_READY, self.buzzer_kind, now)        # 2声

    def _start_shooting(self, now, resuming=False):
        self.state = self.SHOOTING
        if not resuming:
            self.duration = self.shooting_duration
            self.elapsed = 0.0
        self.run_origin = now
        # 新規開始・再開のどちらも1声。緑/黄色は関係ない
        # (時間経過で緑->黄色に切り替わるだけのときは、この関数を通らないので無音のまま)
        self.trigger_beep(BUZZER_START, self.buzzer_kind, now)

    def _finish(self, now):
        self.state = self.FINISHED
        self.duration = 0
        self.elapsed = 0.0
        self.run_origin = None
        self.trigger_beep(BUZZER_FINISH, self.buzzer_kind, now, blink=True)   # 3声+点滅

    def _complete_round(self, now):
        """ムーブアップ+行射の1ラウンド(「立」)が終わった。早送りでも自然終了でも同じ処理。"""
        if self.round < self.repeat:
            self.round += 1
            self._start_moveup(now)        # 2声
        else:
            self._finish(now)              # 3声

    def _reset_to_pre_start(self):
        self.round = 1
        self.state = self.PRE_START
        self.duration = self.moveup_duration
        self.elapsed = 0.0
        self.run_origin = None
        self.silence()

    # --- キー操作 (図の矢印に無いキーは、その状態では何もしない) -----------------------
    def handle_key(self, key, now):
        if key == "SPACE":
            if self.state == self.PRE_START:
                self._start_moveup(now)                            # 2声
            elif self.state == self.INTERRUPTED_MOVEUP:
                self.state = self.PRE_START                        # 無音。ここでリセットされる
                self.duration = self.moveup_duration
                self.elapsed = 0.0
            elif self.state == self.INTERRUPTED_SHOOTING:
                self._start_shooting(now, resuming=True)           # 1声 or 無音
            elif self.state == self.FINISHED:
                self._reset_to_pre_start()                         # 無音
        elif key == "RIGHT":
            if self.state in (self.MOVEUP, self.SHOOTING):
                self._complete_round(now)
        elif key == "ENTER":
            if self.state == self.MOVEUP:
                self.elapsed = self.current_elapsed(now)
                self.run_origin = None
                self.state = self.INTERRUPTED_MOVEUP
                self.trigger_beep(BUZZER_ALARM, self.buzzer_kind, now)   # 5声
            elif self.state == self.SHOOTING:
                self.elapsed = self.current_elapsed(now)
                self.run_origin = None
                self.state = self.INTERRUPTED_SHOOTING
                self.trigger_beep(BUZZER_ALARM, self.buzzer_kind, now)   # 5声

    def update(self, now):
        """時間切れで自動的に次の状態へ進んでいないか確認する。"""
        if self.run_origin is None:
            return
        if self.current_elapsed(now) < self.duration:
            return
        if self.state == self.MOVEUP:
            self._start_shooting(now)          # 1声
        elif self.state == self.SHOOTING:
            self._complete_round(now)          # 2声 or 3声

    # --- 表示 ---------------------------------------------------------
    def label_and_color(self, now):
        if self.state == self.PRE_START:
            return "開始前", _ANSI_RED
        if self.state == self.MOVEUP:
            return "ムーブアップ", _ANSI_RED
        if self.state == self.SHOOTING:
            color = _ANSI_YELLOW if self.remaining(now) <= self.warn else _ANSI_GREEN
            return "行射", color
        if self.state in (self.INTERRUPTED_MOVEUP, self.INTERRUPTED_SHOOTING):
            return "中断", _ANSI_RED
        return "行射終了", _ANSI_RED

    def display_value(self, now):
        return self.remaining(now)

    def display_frame(self, now):
        if self.state == self.FINISHED and self.beep_blink and self.beep_anchor is not None:
            rel = now - self.beep_anchor
            if rel < self.blink_total:
                active = any(start <= rel < end for start, end in self.blink_intervals)
                return self._zero_frame() if active else self._blank_frame
        value = self.display_value(now)
        if value not in self._display_cache:
            self._display_cache[value] = frame_for_plain_seconds(value)
        return self._display_cache[value]

    def _zero_frame(self):
        if 0 not in self._display_cache:
            self._display_cache[0] = frame_for_plain_seconds(0)
        return self._display_cache[0]

    def display_text(self, now):
        """実際にタイマー本体へ送っている表示フレームを、そのままデコードした文字列。"""
        body = bytes.fromhex(self.display_frame(now)[1:-3].decode())
        return "".join(SEG_TO_CHAR.get(v & 0x7F, "?") for v in body[1:8])

    def status_line(self, now):
        label, color = self.label_and_color(now)
        beep = "  *Beep" if self.beep_active_at(now) else ""
        return (f"{color}[{self.display_text(now):>7}]  "
                f"{label:<10} {self.round}/{self.repeat}立{beep}")


def send_timer(moveup, shooting, warn, repeat, buzzer, ser, args):
    """
    アーチェリー用タイマーをキーボード操作で動かす。
        moveup, shooting, warn : ムーブアップ秒 / シューティング秒 / 黄色警告秒
        repeat                 : 「立」の回数
        buzzer                 : "0"=鳴らさない / "1"=ブザー1 / "2"=ブザー2
    """
    kind = None if buzzer == "0" else (0x01 if buzzer == "1" else 0x00)
    engine = TimerEngine(moveup, shooting, warn, repeat, kind)

    enable_ansi()
    send_init(ser, args)
    print(f"  ムーブアップ{moveup}秒 → 行射{shooting}秒 (黄色{warn}秒)   "
          f"{repeat}立   "
          f"{'ブザーなし' if kind is None else f'ブザー{buzzer} (0.7秒鳴動/0.3秒間隔)'}")
    print("  [Space]開始/再開   [→]早送り   [Enter]中断   [Ctrl+C]終了")

    with KeyPoll() as keys:
        origin = time.perf_counter()
        tick = 0
        try:
            while True:
                now = time.perf_counter()
                for key in keys.poll():
                    engine.handle_key(key, now)
                engine.update(now)

                buzzer_frame = engine.beep_frame_at(now)
                if buzzer_frame is not None:
                    write(ser, buzzer_frame)
                write(ser, engine.display_frame(now))

                if not args.quiet:
                    sys.stdout.write(f"\r  {engine.status_line(now):<80}")
                    sys.stdout.flush()

                tick += 1
                delay = origin + tick * FRAME_INTERVAL - time.perf_counter()
                if delay > 0:
                    time.sleep(delay)
        except KeyboardInterrupt:
            print("\n終了します。")
    flush(ser)


# ---------------------------------------------------------------------------
# 送信
# ---------------------------------------------------------------------------
def open_port(port_name):
    import serial
    return serial.Serial(
        port=port_name,
        baudrate=BAUDRATE,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=0.1,
        write_timeout=2.0,
    )


def write(ser, data):
    if ser is not None:
        ser.write(data)


def flush(ser):
    if ser is not None:
        ser.flush()


def send_init(ser, args):
    """送信開始の宣言フレームを1回送る。"""
    if args.no_init:
        return
    frame = frame_for_init()
    write(ser, frame)
    flush(ser)
    time.sleep(FRAME_INTERVAL)


def send_stream(frame_at, ser, args, duration=None, label=""):
    """
    実機と同じ 85ms 周期で送り続ける。
    frame_at(経過秒) が、その瞬間に送るフレームのリストを返す。
    None を返すか duration を過ぎたら終了。
    """
    origin = time.perf_counter()
    tick = sent = 0
    try:
        while True:
            now = time.perf_counter() - origin
            frames = frame_at(now)
            if frames is None or (duration is not None and now > duration):
                break
            for frame in frames:
                write(ser, frame)
                sent += 1
            if not args.quiet:
                sys.stdout.write(f"\r  {now:7.2f} 秒   {sent:6d} フレーム   {label}   ")
                sys.stdout.flush()
            tick += 1
            delay = origin + tick * FRAME_INTERVAL - time.perf_counter()
            if delay > 0:
                time.sleep(delay)
    except KeyboardInterrupt:
        print("\n中断しました。")
    flush(ser)
    print(f"\n送信完了: {sent} フレーム")


def send_static(frame, ser, args, label=""):
    """同じフレームを送り続ける (実機は静止画面でも送り続けている)。"""
    send_init(ser, args)
    send_stream(lambda now: [frame], ser, args, args.duration, label)


def send_countdown(total_seconds, ser, args):
    """カウントダウンを実演する。0になったらブザーフレームも流す。"""
    buzzer_frame = None
    if args.buzzer_at_end is not None:
        buzzer_frame = frame_for_buzzer(0x01 if args.buzzer_at_end == "1" else 0x00)
    cache = {}

    def frame_at(now):
        remaining = max(0, total_seconds - int(now))
        if remaining <= 0 and now > total_seconds + args.buzzer_seconds:
            return None
        if remaining not in cache:
            cache[remaining] = frame_for_seconds(remaining)
        frames = []
        if buzzer_frame is not None and remaining <= 0:
            frames.append(buzzer_frame)      # 実機は表示の合間にブザーを挟む
        frames.append(cache[remaining])
        return frames

    print(f"  カウントダウン {total_seconds} 秒を送出します。")
    send_init(ser, args)
    send_stream(frame_at, ser, args, None, "カウントダウン")


def send_records(records, ser, args):
    """記録された経過秒どおりの間隔で送出する。"""
    if not records:
        print("送るレコードがありません。", file=sys.stderr)
        return
    send_init(ser, args)
    base = records[0][0]
    count = loop = 0
    try:
        while True:
            origin = time.perf_counter()
            for elapsed, data in records:
                delay = (elapsed - base) / args.speed - (time.perf_counter() - origin)
                if delay > 0:
                    time.sleep(delay)
                write(ser, data)
                count += 1
                if not args.quiet:
                    sys.stdout.write(f"\r  {elapsed - base:8.2f} 秒   "
                                     f"{count:6d} フレーム   "
                                     f"{data[:16].hex(' ').upper()}")
                    sys.stdout.flush()
            flush(ser)
            loop += 1
            if not args.loop:
                break
            if not args.quiet:
                print(f"\n  --- {loop} 巡目 完了、先頭に戻ります ---")
    except KeyboardInterrupt:
        print("\n中断しました。")
    print(f"\n送信完了: {count} フレーム")


def send_raw(path, ser):
    """生バイト列を、ボーレートなりの速度で流す。"""
    with open(path, "rb") as f:
        data = f.read()
    print(f"  生バイト列 {len(data)} バイトを送出します。")
    chunk = 64
    interval = chunk * 10 / BAUDRATE          # 1バイト=10ビット換算
    for i in range(0, len(data), chunk):
        write(ser, data[i:i + chunk])
        time.sleep(interval)
    flush(ser)


# ---------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description="モルテンタイマー データ送信")
    ap.add_argument("recfile", nargs="?", default=None,
                    help="record_data.py が作った *_rec.txt")
    ap.add_argument("--port", default="COM3", help="シリアルポート (既定: COM3)")
    ap.add_argument("--raw-bin", default=None, help="生バイト列 *_raw.bin を流す")
    ap.add_argument("--frame", default=None,
                    help='16進文字列をそのまま送る 例: "02 30 31 ... 03 30 35"')

    build = ap.add_argument_group("フレームを組み立てて送る")
    build.add_argument("--time", default=None,
                       help='時間表示 例: "3:00" "1:02:03" "180"')
    build.add_argument("--score", default=None, help='得点表示 例: "12-3"')
    build.add_argument("--text", default=None,
                       help='表示文字を直接指定 例: "F-4" "P-1" "1.00.00"')
    build.add_argument("--buzzer", choices=["0", "1", "2"], default=None,
                       help="単独指定: ブザーを鳴らす (0は無効。単独では意味を持たない)。"
                            "--timer と併用: 予告/開始/終了で使う音色を指定 "
                            "(0=ブザーなし、既定は1)")
    build.add_argument("--countdown", default=None,
                       help='カウントダウンを実演する 例: "3:00"')
    build.add_argument("--timer", default=None,
                       help='アーチェリー用タイマー(キーボード操作) '
                            '"ムーブアップ秒,シューティング秒[,黄色警告秒]" '
                            '例: "10,180,30" (黄色警告省略時は "10,180" で0秒扱い)。'
                            'Space=開始/再開、右矢印=早送り、'
                            'Enter=中断(5声)、Ctrl+Cで終了')
    build.add_argument("--repeat", type=int, default=1,
                       help="--timer を繰り返す回数 (既定: 1)")
    build.add_argument("--buzzer-at-end", choices=["1", "2"], default=None,
                       help="--countdown が0になったらブザーも鳴らす")
    build.add_argument("--buzzer-seconds", type=float, default=1.9,
                       help="ブザーを鳴らす長さ (既定: 1.9秒 = 実機と同じ)")
    build.add_argument("--duration", type=float, default=None,
                       help="送り続ける秒数 (既定: Ctrl+C まで)")
    build.add_argument("--init-only", action="store_true",
                       help="INIT フレームだけ送って終了する")
    build.add_argument("--no-init", action="store_true",
                       help="開始時の INIT フレームを送らない")

    replay = ap.add_argument_group("記録ファイルの再生")
    replay.add_argument("--speed", type=float, default=1.0, help="再生速度の倍率")
    replay.add_argument("--loop", action="store_true", help="繰り返し再生する")
    replay.add_argument("--from", dest="start", type=float, default=None,
                        help="この経過秒から送る")
    replay.add_argument("--to", dest="end", type=float, default=None,
                        help="この経過秒まで送る")
    replay.add_argument("--check", action="store_true", help="送信前にBCCを検証する")

    ap.add_argument("--dry-run", action="store_true", help="ポートを開かず内容だけ表示")
    ap.add_argument("--quiet", action="store_true", help="画面表示を減らす")
    args = ap.parse_args()

    modes = (args.recfile, args.raw_bin, args.frame, args.time, args.score,
             args.text, args.buzzer, args.countdown, args.timer,
             args.init_only or None)
    if not any(modes):
        ap.error("recfile / --raw-bin / --frame / --time / --score / --text / "
                 "--buzzer / --countdown / --timer / --init-only "
                 "のいずれかを指定してください")

    ser = None
    if args.dry_run:
        print("--dry-run: 実際には送信しません。")
    else:
        try:
            ser = open_port(args.port)
        except ImportError:
            print("pyserial が必要です:  pip install pyserial", file=sys.stderr)
            return 1
        except Exception as err:
            print(f"{args.port} を開けません: {err}", file=sys.stderr)
            if "PermissionError" in repr(err) or "アクセス" in str(err):
                print("  他のソフトが同じポートを掴んでいないか確認してください。",
                      file=sys.stderr)
            return 1

    try:
        # --- 組み立てて送る ---
        try:
            if args.init_only:
                send_init(ser, args)
                return 0
            if args.countdown is not None:
                send_countdown(parse_time(args.countdown), ser, args)
                return 0
            if args.timer is not None:
                parts = [p.strip() for p in args.timer.split(",")]
                if len(parts) not in (2, 3):
                    raise ValueError('--timer は "ムーブアップ秒,シューティング秒'
                                     '[,黄色警告秒]" で指定してください '
                                     '(例: "10,180,30" または "10,180")')
                moveup, shooting = (parse_time(p) for p in parts[:2])
                warn = parse_time(parts[2]) if len(parts) == 3 else 0
                if args.repeat < 1:
                    raise ValueError("--repeat は1以上を指定してください")
                buzzer = args.buzzer if args.buzzer is not None else "1"
                send_timer(moveup, shooting, warn, args.repeat, buzzer, ser, args)
                return 0

            frame = label = None
            if args.time is not None:
                seconds = parse_time(args.time)
                frame, label = frame_for_seconds(seconds), f"時間 {seconds} 秒"
            elif args.score is not None:
                left, right = parse_score(args.score)
                frame, label = frame_for_score(left, right), f"得点 {left}-{right}"
            elif args.text is not None:
                frame, label = frame_for_text(args.text), f"表示 {args.text!r}"
            elif args.buzzer is not None:
                if args.buzzer == "0":
                    print("  --buzzer 0 は単独では鳴らすものがありません。"
                          "--timer と組み合わせて使ってください。")
                    return 0
                frame = frame_for_buzzer(0x01 if args.buzzer == "1" else 0x00)
                label = f"ブザー{args.buzzer}"
                if args.duration is None:
                    args.duration = args.buzzer_seconds

            if frame is not None:
                ok, note = check_frame(frame)
                print(f"  {label}: {frame.hex(' ').upper()}")
                print(f"  BCC {'OK' if ok else 'NG'} ({note}) / {len(frame)} バイト")
                send_static(frame, ser, args, label)
                return 0
        except ValueError as err:
            print(f"エラー: {err}", file=sys.stderr)
            return 1

        # --- そのまま送る ---
        if args.frame:
            data = bytes.fromhex(args.frame.replace(" ", ""))
            ok, note = check_frame(data)
            print(f"  {data.hex(' ').upper()}   BCC {'OK' if ok else 'NG'} ({note})")
            write(ser, data)
            flush(ser)
            return 0

        if args.raw_bin:
            send_raw(args.raw_bin, ser)
            return 0

        records, header = load_records(args.recfile)
        for line in header:
            print("  " + line)
        if args.start is not None:
            records = [r for r in records if r[0] >= args.start]
        if args.end is not None:
            records = [r for r in records if r[0] <= args.end]
        span = (records[-1][0] - records[0][0]) if records else 0.0
        print(f"  {len(records)} レコード / {span:.2f} 秒ぶん / 速度 x{args.speed}")

        if args.check:
            ng = sum(1 for _, data in records if not check_frame(data)[0])
            for elapsed, data in records:
                ok, note = check_frame(data)
                if not ok:
                    print(f"  BCC NG  {elapsed:9.3f}  {data.hex(' ').upper()}  {note}")
            print(f"  BCC 検証: NG {ng} 件 / 全 {len(records)} 件")
            print(f"  レコード長の種類: {sorted({len(d) for _, d in records})}")

        send_records(records, ser, args)
        return 0
    finally:
        if ser is not None:
            ser.close()


if __name__ == "__main__":
    sys.exit(main())