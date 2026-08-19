#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
モルテン ハンディータイマー (UD0040) のシリアルデータ 無損失記録プログラム

snd_data.py で送信するための元データを作る。
既存の rcv_data.py と違い、

  ・パケットの解釈を一切しない (BCCの長さもフレーム長も決め打ちしない)
  ・「表示が変化したときだけ記録」をしない (同じ内容でも全部残す)
  ・受信した全バイトをそのまま残す

ので、取りこぼしが起きない。

■ 出力する2種類のファイル (どちらも同時に書き出す)

  1) <基準名>_raw.bin
       受信したバイト列そのもの。一切加工しない生ログ。
       解析をやり直したいとき、この1本があれば何でも再現できる。

  2) <基準名>_rec.txt
       開始時刻からの経過秒 + そのまま送出できる16進文字列。
       1レコード = STX(0x02) から「次の STX の直前」まで。
       フレーム長を仮定せず STX で切るだけなので、ETX の後ろに何バイト
       続いていても全部そのまま入る (BCCが1文字か2文字かもこれで分かる)。

       # で始まる行はヘッダ・コメント。データ行は次の形式:
           <経過秒> <TAB> <16進文字列>
           0.000000	023031303030303030303042463033463032...0337

使い方:
    python record_data.py                        # COM3 から記録
    python record_data.py --port COM4 --seconds 60
    python record_data.py --name test1           # 出力ファイル名を指定
    python record_data.py --spaced               # 16進を空白区切りで読みやすく
    Ctrl+C で終了 (それまでの内容は既にディスクに書かれている)
"""

import argparse
import os
import sys
import time
from datetime import datetime

STX = 0x02


class RecordSplitter:
    """
    バイト列を STX 区切りのレコードに切り分ける。
    フレーム長も終端も仮定しないので、どんな未知のコマンドが来ても落とさない。
    """

    def __init__(self):
        self.buffer = bytearray()
        self.start_time = 0.0
        self.started = False

    def feed(self, byte: int, now: float):
        """1バイト投入。レコードが確定したら (経過秒, bytes) を返す。"""
        if byte == STX:
            result = None
            if self.buffer:
                result = (self.start_time, bytes(self.buffer))
            self.buffer = bytearray([byte])
            self.start_time = now
            self.started = True
            return result
        if not self.started:
            # 最初の STX より前に届いたバイト。捨てずに前置きとして貯めておく
            if not self.buffer:
                self.start_time = now
            self.buffer.append(byte)
            return None
        self.buffer.append(byte)
        return None

    def flush(self):
        """残っているレコードを取り出す (終了時)。"""
        if self.buffer:
            result = (self.start_time, bytes(self.buffer))
            self.buffer = bytearray()
            return result
        return None


def format_record(elapsed: float, data: bytes, spaced: bool) -> str:
    text = data.hex(" ").upper() if spaced else data.hex().upper()
    return f"{elapsed:.6f}\t{text}\n"


def main():
    ap = argparse.ArgumentParser(description="モルテンタイマー 無損失記録")
    ap.add_argument("--port", default="COM3", help="シリアルポート (既定: COM3)")
    ap.add_argument("--baud", type=int, default=9600, help="ボーレート (既定: 9600)")
    ap.add_argument("--no-rtscts", dest="rtscts", action="store_false",
                    help="RTS/CTS フロー制御を使わない")
    ap.add_argument("--name", default=None,
                    help="出力ファイルの基準名 (既定: yyyymmddhhmmss)")
    ap.add_argument("--outdir", default=".", help="出力先ディレクトリ")
    ap.add_argument("--seconds", type=float, default=None,
                    help="この秒数で自動終了する")
    ap.add_argument("--spaced", action="store_true",
                    help="16進を空白区切りで書く (読みやすさ優先)")
    ap.add_argument("--quiet", action="store_true", help="画面表示を減らす")
    args = ap.parse_args()

    try:
        import serial
    except ImportError:
        print("pyserial が必要です:  pip install pyserial", file=sys.stderr)
        return 1

    started_at = datetime.now()
    base = args.name or started_at.strftime("%Y%m%d%H%M%S")
    os.makedirs(args.outdir, exist_ok=True)
    raw_path = os.path.join(args.outdir, f"{base}_raw.bin")
    rec_path = os.path.join(args.outdir, f"{base}_rec.txt")

    ser = serial.Serial(
        port=args.port,
        baudrate=args.baud,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        rtscts=args.rtscts,
        timeout=0.02,
    )

    splitter = RecordSplitter()
    total_bytes = 0
    total_records = 0

    print(f"記録開始: {args.port} {args.baud}bps rtscts={args.rtscts}")
    print(f"  生バイト列 : {raw_path}")
    print(f"  送出用ログ : {rec_path}")
    print("Ctrl+C で終了します。")

    with open(raw_path, "wb", buffering=0) as raw_file, \
            open(rec_path, "w", encoding="utf-8") as rec_file:

        rec_file.write("# molten timer capture v1\n")
        rec_file.write(f"# start={started_at.strftime('%Y-%m-%d %H:%M:%S.%f')}\n")
        rec_file.write(f"# port={args.port} baud={args.baud} bytesize=8 "
                       f"parity=N stopbits=1 rtscts={args.rtscts}\n")
        rec_file.write("# record = STX から次の STX の直前まで (長さは仮定しない)\n")
        rec_file.write("# format: <経過秒>\\t<16進文字列>\n")
        rec_file.flush()

        origin = time.perf_counter()
        try:
            while True:
                chunk = ser.read(ser.in_waiting or 1)
                now = time.perf_counter() - origin
                if chunk:
                    raw_file.write(chunk)          # まず生ログに退避
                    total_bytes += len(chunk)
                    for byte in chunk:
                        record = splitter.feed(byte, now)
                        if record:
                            rec_file.write(format_record(*record, args.spaced))
                            rec_file.flush()       # 落ちても残るよう都度保存
                            total_records += 1
                    if not args.quiet:
                        sys.stdout.write(
                            f"\r  {now:8.2f} 秒   {total_bytes:8d} バイト   "
                            f"{total_records:6d} レコード")
                        sys.stdout.flush()
                if args.seconds is not None and now >= args.seconds:
                    break
        except KeyboardInterrupt:
            pass
        finally:
            record = splitter.flush()
            if record:
                rec_file.write(format_record(*record, args.spaced))
                total_records += 1
            rec_file.flush()
            ser.close()

    elapsed = time.perf_counter() - origin
    print(f"\n記録終了: {elapsed:.2f} 秒 / {total_bytes} バイト / "
          f"{total_records} レコード")
    print(f"  {raw_path}\n  {rec_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
