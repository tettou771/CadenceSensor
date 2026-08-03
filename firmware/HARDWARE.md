# cadence sensor ハードウェア

## テスト基板: RideFormTracker PCB **v1.0**（board = `cadence_rft`）

本番基板ができるまではこれを流用する。実体は
`2026/2026-02-28_RideFormTracker/apps/v2/HARDWARE.md` の v1.1 と同じ基板なので、
詳細（電源・充電まわり・SWD トラブルシュート）はそちらが一次資料。
FindMyTag の `findmy_rft` とも同じ基板で、ピンも配置も同一。

ここには **ケイデンスセンサーが実際に使うピンだけ** 載せる。

### 使うピン

| nRF52840 | パッド | 信号 | 接続先 |
|---|---|---|---|
| P0.06 | 22 | SPI MOSI | LSM6DSV SDA/SDI |
| P0.04 | 20 | SPI MISO | LSM6DSV SDO/SA0 |
| P0.08 | 24 | SPI SCLK | LSM6DSV SCL/SPC |
| P1.09 | 26 | SPI CS | LSM6DSV CS |
| P0.26 | 19 | IMU_INT1 | LSM6DSV INT1（wake-up） |
| P0.05 | 21 | IMU_INT2 | LSM6DSV INT2（未使用） |
| P0.28 | 13 | LED_R | RGB LED 赤（共通アノード＝アクティブ LOW） |
| P0.31 | 12 | LED_G | RGB LED 緑 |
| P0.30 | 14 | LED_B | RGB LED 青 |
| P0.00/P0.01 | 17/18 | XL1/XL2 | 32.768 kHz 水晶 |
| D+ / D- | 35/34 | USB | USB-C |
| SWDIO/SWCLK | 51/53 | SWD | U5 ヘッダ（初回書き込み用） |

### v1.0 と v1.1 の違い

このファームから見て**違うのは RGB LED のピンだけ**。SPI・両 INT・水晶・USB・SWD は
v1.0/v1.1 共通（RideFormTracker の HARDWARE.md で "same on v1.0 and v1.1" と
明記されている）。

| 信号 | **v1.0（いまこれ）** | v1.1 |
|---|---|---|
| LED_R | **P0.28**（パッド 13） | P0.30（パッド 14） |
| LED_G | **P0.31**（パッド 12） | P0.29（パッド 10） |
| LED_B | **P0.30**（パッド 14） | P0.31（パッド 12） |

v1.1 では P0.28 が磁気センサーの I2C SDA に変わっているので、ここは本当に
リビジョン依存。とはいえ間違えても失うのは LED だけで、ケイデンス検出は LED を
一切使わないし、そもそも USB 接続時しか点かない。切り替えは
`boards/nordic/cadence_rft/cadence_rft.dts` の `leds` ノード3行。

### 使わないもの（DTS で明示的に disable）

| 部品 | 理由 |
|---|---|
| W25Q256JVP (QSPI フラッシュ) | 本番基板には載せない。ファームが依存しないよう最初から切る |
| IIS2MDC (磁気センサー, I2C) | ケイデンスには不要 |
| CryptoCell (CC310) | 使わない。エントロピーは `&rng` から取る |
| NFCT | 使わない（P0.09/P0.10 を GPIO に開放） |

> **注意**: `&cryptocell` を disable すると、nrf52840.dtsi のデフォルトの
> `zephyr,entropy` が宙に浮いてビルドが落ちる。DTS の `chosen` で
> `zephyr,entropy = &rng;` を指定してある。

### 32.768 kHz 水晶は必須

CSC Measurement には「最後に1回転した時刻」が 1/1024 秒単位で乗り、サイコンは
**それを微分して RPM を出す**。つまり表示されるケイデンスの精度は、このクロックの
精度そのもの。内蔵 RC は校正しても ±500 ppm、水晶は ±20 ppm。
`cadence_rft_defconfig` の `CONFIG_CLOCK_CONTROL_NRF_K32SRC_XTAL=y` はこのため。

### バッテリ電圧が読めない

BAT_CHECK が P0.09 に繋がっているが、**nRF52840 の AIN は P0.02–P0.05 と
P0.28–P0.31 だけ**なので P0.09 では ADC が取れない。v1.0/v1.1 共通の設計ミス。

そのためファームは Battery Service で常に 100% を返している。本番基板では AIN0/1/3
(P0.02 / P0.03 / P0.05) のどれかに回すこと。回せば `ble_csc_set_battery()` を
呼ぶだけで済む。

### GPIO 電圧

MDBT50Q は UICR の REGOUT0 が未設定（＝消去済み）だと GPIO が 1.8 V で動く。
LSM6DSV の VDD_IO 下限を割るし、共通アノード LED も引けない。
`boards/nordic/cadence_rft/board.c` が起動時に 3.3 V を書いて 1 回だけリブートする。

チップ消去（APPROTECT 解除の mass erase を含む）のあと最初の 1 回だけ、
起動が 2 回走るのはこのため。異常ではない。

なお board.c はブートローダー側にもリンクされるので、`sysbuild/mcuboot.conf` に
`CONFIG_REBOOT=y` が要る（FindMyTag ではシリアルリカバリが REBOOT を連れてきていた
ので気付かれていなかった）。

---

## 取り付け

センサーは**クランクアームに固定する**。向きは問わない —— ファームが回転面を
自動判定する（AC エネルギーが一番小さい軸をスピンドル軸とみなす）。

効くのは向きより**位置**で、スピンドルからの距離 r が遠心力 w²r を決める:

| 位置 | r | 120 rpm での遠心力 |
|---|---|---|
| ペダル寄り | 0.17 m | 2.7 g |
| クランク中ほど | 0.09 m | 1.4 g |
| スピンドル近く | 0.03 m | 0.5 g |

**スピンドルに近いほど楽**（遠心力が小さく、DC トラッカーの仕事が減る）。ただし
近すぎても得はしない —— 信号は遠心力ではなく重力なので、r に関係なく 1 g のまま。
±8 g レンジは r = 0.17 m・150 rpm（4.3 g + 重力）でも飽和しない値として選んでいる。

取り付けたら USB コンソールの `c` でライブ表示を見ながらクランクを回して、
`amp` が 1000 mg 付近に落ち着くことを確認する。ここが大きくズレる場合、
センサーが回転面に対して傾いているか、固定が緩んでいる。

---

## 本番基板（未設計）

方針:

- moriyama / EggDrop 基板くらいシンプルに。**外部フラッシュなし**、磁気センサーなし
- 加速度センサーは LSM6DSV 系を継続
- MDBT50Q (nRF52840) 継続、USB-C 有り（SMP で更新するため）
- SWD は初回書き込み用にパッドだけあればよい

決まっていないこと:

- ピンアサイン全般（SPI / INT / LED）
- 電源（コイン電池 or LiPo + 充電）。クランクに付けるので**軽さと防水**が効く
- バッテリ電圧の分圧先 → **AIN のあるピンに繋ぐこと**（上記参照）
- クランクへの固定方法（バンド / 両面テープ / 3Dプリントのクランプ）

ピンが決まったら `boards/nordic/cadence_tag/` を追加する。手順は README の
「本番基板を足すとき」を参照。
