# cadence sensor firmware

nRF52840 (MDBT50Q) + LSM6DSV の自転車ケイデンスセンサー。クランクアームに貼って、
標準の BLE Cycling Speed and Cadence プロファイルで回転数を出す。サイコンやスマホの
アプリからは**普通のケイデンスセンサーとして見える**（専用アプリ不要）。

- センサー構成は `2026-07-29_FindMyTag`（同じ nRF52840、同じ LSM6DSV ドライバ）
- 加速度センサーと基板は `2026-02-28_RideFormTracker` の PCB v1.1 をそのまま流用
- 書き込みは `TrussC/apps/tdk/JAVELYTICS_Viewer_2/firmware` と同じ方式
  （初回だけ SWD、以降は**アプリ動作中に** USB 経由で mcumgr SMP）

---

## いま動くこと

- クランク回転の検出（重力ベクトルの回転を追う。原理は後述）
- BLE CSC (0x1816) で crank revolution data を notify
- Battery Service / Device Information Service
- 駐輪中は advertise を止め、**重力の向きが50°以上変わったときだけ**起床（6D）
  - 電車・車載・押し歩きでは起きない（クランクはぶら下がるので向きが変わらない）
  - 60秒静止でセンサー自身が加速度計を 1.875 Hz に落とす（ハードウェア機能）
- USB-CDC コンソールで状態確認・ライブケイデンス表示
- MCUboot + SMP で、SWD なしのファーム更新

**実機で確認済み**（RideFormTracker PCB v1.0）: 書き込み、IMU 認識、実走でのケイデンス出力、
BLE でのスキャン、USB 経由の SMP 更新、6D の arming、駐輪時の 1.875 Hz 降格。
残っている未確認は「未検証・注意」を参照。

---

## 最初の一回

```bash
# 1. NCS ワークスペース（FindMyTag / JAVELYTICS と同じリビジョンなので使い回しでOK）
./setup_workspace.command          # 既存ワークスペースがあればスキップ

# 2. build.env を作って NCS_ROOT を指す
cp build.env.example build.env
$EDITOR build.env

# 3. ビルド＋SWD 書き込み（CMSIS-DAP / DAPLink プローブが要る）
./tools/flash.command --build
```

SWD が要るのは**この一回だけ**。ここで焼かれるのが MCUboot で、以降はそれが USB 更新を
引き受ける。

```bash
./tools/flash_usb.command --build   # ビルド → USB で流し込み → confirm → 読み戻し
```

`flash_usb.command` は confirm まで必ずやる。MCUboot は「テスト」フラグのままの
イメージを一回だけ起動して次の電源投入で元に戻すので、confirm を忘れたファームは
**次にバイクに乗った時点で消える**。

### 3つの書き込み経路の違い

| | `tools/flash.command` | `tools/flash_usb.command` | `tools/recover_usb.command` |
|---|---|---|---|
| 必要なもの | CMSIS-DAP プローブ | USB ケーブルだけ | USB ケーブルだけ |
| 書くもの | `merged.hex`（MCUboot + app） | `zephyr.signed.bin` | `zephyr.signed.bin` |
| 話す相手 | pyocd → SWD | **動作中アプリ**の2本目 CDC 上の SMP | **ブートローダ**の DFU 窓 |
| 電源入れ直し | 不要 | 不要 | **必要**（1秒の窓を狙う） |
| いつ | 新品基板の一回目、文鎮化からの復旧 | それ以外全部 | 他人の MCUboot が既に載ってる基板 |

**RideFormTracker の基板に初めて焼くときは `recover_usb.command` が使える**（プローブ不要）。
あの基板に載っている RFT ファームの MCUboot は、シリアルリカバリ有効・パーティション配置
同一・署名なしと、こちらと条件が揃っているため。焼いた後は RFT の MCUboot の上に
このアプリが乗る形になり、以降は `flash_usb.command` も使えるようになる。

`flash_usb.command` を前提が揃う前に実行すると SMP のタイムアウトが延々出るだけなので、
`flash_usb.py` は失敗時に USB の VID/PID を読んで「この基板はまだこのファームじゃない」
と言うようにしてある。

FindMyTag は MCUboot のシリアルリカバリ（電源入れ直して 1 秒の窓に流し込む）だったが、
こっちは JAVELYTICS と同じで **SMP エンドポイントがアプリ側にある**。電源を入れ直す
必要も、タイミングを狙う必要もない。代わりにアプリが完全に死ぬと USB では戻せないので、
そのときは SWD（`tools/flash.command`）。

---

## ケイデンスをどうやって測っているか

クランクに付いたセンサーから見ると、**重力ベクトルが 1 回転につき 1 周する**。それが信号。
残りは全部それを見えなくしているもので、`src/cadence.h` に分離の理屈が、
`src/cadence.c` に実装がある。要点だけ:

| 邪魔なもの | 大きさ | 分離できる理由 |
|---|---|---|
| 遠心力 w²r | **120 rpm で 2.7 g**（重力の約3倍） | 回転するセンサー座標系では常に軸を指す＝DC。DC トラッカーが引く |
| 接線加速度 r·dw/dt | 数百 mg | ペダリングのムラ由来でクランク周波数の**2倍**。低域通過で減らす（消せはしない） |
| 路面振動 | 広帯域 | 低域通過。あとセンサーの ODR より**速く**読むことで折り返しを防ぐ |
| 取り付け向き | — | 仮定しない。AC エネルギーが小さい軸をスピンドル軸と判定して自動で決める |

DC トラッカーと低域通過の時定数は**秒ではなくクランク周期に対して**決めている。これが
効く: 遠心力が一番大きく一番速く変化するのは高ケイデンスのときで、そこでちょうど
フィルタも速くなる。固定 8 秒だと 4 秒のスプリントで 68 回転中 16 回落とす（実測）。

### アルゴリズムの検証

実機がないので、**実際の `src/cadence.c` をそのままホストでコンパイルして**
合成波形を通してある（`test/zephyr/` が Zephyr API のスタブ）。コピーではなく
出荷するコードそのものを回しているので、ここで通ったことは実装の性質になる。

```bash
./test/run.sh              # 18シナリオ
./test/run.sh -t "SPRINT"  # 1シナリオを毎サンプルトレース
```

30〜150 rpm の定常、取り付け向き3種、路面ノイズ、短いクランク、低ケイデンスの
グラインド、スプリント、立ち漕ぎスタート、そして「回転ではない揺れ」。
各シナリオは内部カウントと、**サイコン側が notify されたバイト列から計算する RPM**の
両方で確認している。実測結果は `test/test_cadence.c` の冒頭。

定常はすべて誤差0、偽カウント0。残差は過渡のみ（2秒で60→140rpm のスプリントで
73回転中-3、立ち漕ぎスタートで検出開始までの約2秒分）。CSC のフィールドは累積値なので
恒久的にはズレない。

---

## BLE から見えるもの

```
0x1816 Cycling Speed and Cadence
  0x2A5B CSC Measurement    notify   flags(1) revs(2) event_time(2)
  0x2A5C CSC Feature        read     0x0002 = crank revolution data
  0x2A5D Sensor Location    read     5 = left crank
0x180F Battery Service
0x180A Device Information
```

- notify は**1回転ごと**＋止まっていても 1 秒ごと。後者がないとサイコンは
  ケイデンス 0 を表示できず、最後の数字で固まる
- ペアリングしない（`CONFIG_BT_SMP=n`）。CSC は暗号化を要求しないので、OS の
  ペアリング一覧に出ずに繋がる
- アドレスは FICR の DEVICEID から作る固定値。毎回変わるとサイコンが「別のセンサー」
  として扱うので、電池交換のたびに登録し直しになる
- 名前は `CAD-XXXX`（DEVICEID 下位16bit）。2個以上作ったときの区別用

ホスト側の確認ツール:

```bash
python3 tools/cadence_ble.py          # サイコンと同じ計算で RPM を出す
```

---

## USB コンソール

USB を挿したときだけ enumerate する（電池運用時は USB を上げない）。
`/dev/cu.usbmodem*` の**若い方**がコンソール（もう片方は SMP）。

```
h = help   s = status   c = live cadence   a = accel   r = reboot
```

`c` のライブ表示が取り付け確認用。クランクを回して:

- `plane` が2軸に落ち着く（スピンドル軸が正しく除外されている）
- `amp` が **1000 mg 付近**（これは重力そのものなので、大きくズレていたら
  取り付けが回転面に対しておかしい）
- `rpm` が実際の回転と合う

---

## フラッシュ配置

Partition Manager が DTS の `fixed-partitions` を上書きするので `pm_static.yml` で固定。
FindMyTag と同じ配置にしてあるので、同じ基板を行き来させても NVS の位置がズレない。

| 領域 | アドレス | サイズ |
|---|---|---|
| mcuboot | 0x00000 | 128 KB |
| mcuboot_primary (app) | 0x20000 | 416 KB |
| mcuboot_secondary | 0x88000 | 416 KB |
| nvs_storage | 0xf0000 | 64 KB |

現状の使用量: **app 177 KB / 416 KB、MCUboot 24 KB / 128 KB、RAM 46 KB / 256 KB**。

MCUboot はシリアルリカバリを積んでいないぶん FindMyTag より小さい（52→24 KB）。
128 KB は余っているが、配置を動かすと deploy 済み基板と互換が切れるのでそのまま。

---

## 本番基板を足すとき

いまの `boards/nordic/cadence_rft/` は**テスト用**（RideFormTracker PCB v1.1 を流用）。
クランクに付ける本番基板は外部フラッシュ・磁気センサーなしのシンプル構成を想定。

1. `boards/nordic/cadence_tag/` を作る（`cadence_rft/` をコピーしてピンを直す）
2. `board.yml` / `Kconfig.<name>` / `<name>_defconfig` の名前を合わせる
3. `pm_static.yml` は 1 MB フラッシュならそのまま
4. `build.env` の `BOARD` を変える

ファーム側で基板依存なのは DTS だけ（`lsm6dsv` ノード、LED、`imu-int1-gpios`、
2本の CDC）なので `src/` は触らなくていいはず。

---

## 未検証・注意

- **6D が回転で実際に発火するかは未確認。** arming は実機で読み戻し検証まで通って
  いるが、基板を回して `motion evt` が増えるところは見ていない。ここが効かないと
  駐輪から復帰できない（USB 挿せば起きるので詰みはしない）。
- **実走ログとホスト検証の突き合わせが未了。** `test/test_cadence.c` の信号モデルは
  物理から立てたもので、実走データと比べたわけではない。走行中に `c` のライブ表示を
  録って `amp` が 1000 mg 付近か、`plane` が安定しているかを見ると裏が取れる。
- `src/lsm6dsv.c` のレジスタマップは **ST 公式の `lsm6dsv16b_reg.h` と照合済み**
  （RideFormTracker の `modules/hal/st/sensor/stmemsc/` にある）。当初 FindMyTag から
  引き継いだ定義に 2 箇所の誤りがあった: `FUNCTIONS_ENABLE` の `interrupts_enable` は
  bit3 ではなく **bit7**、`WAKE_UP_DUR` の `WAKE_THS_W` は **この世代には存在しない**
  （重みは `INACTIVITY_DUR` に移動し、全スケール非依存の絶対 mg 値になった）。
  前者のせいで INT1 は長らく一度も発火していなかった。**データシートではなくこの
  ヘッダを一次資料にすること。**
- **バッテリ残量は 50% 固定を送っている**（デバッグ用の目印）。100% だと「読めていない
  ので既定値が出ている」のと区別がつかないため。この基板は BAT_CHECK が P0.09 に
  繋がっていて nRF52840 では ADC が取れない（HARDWARE.md 参照）。本番基板では
  AIN0/1/3 (P0.02/0.03/0.05) に回すこと。回せば `ble_csc_set_battery()` を
  呼ぶだけで済む。
- Sensor Location は「左クランク」固定。右に付けるなら `ble_csc.c` の
  `SENSOR_LOCATION_LEFT_CRANK` を 6 に。表示だけの話で動作には影響しない。
- MCUboot は署名なし（`SB_CONFIG_BOOT_SIGNATURE_TYPE_NONE`）。
- ホスト検証の信号モデルは自分で立てたもの。実走ログと突き合わせたわけではないので、
  実機が動いたら `c` のライブ表示を録って `test/test_cadence.c` の想定と比べるとよい。


---

## 消費電力と起床の設計

駐輪中に何もしないことが電池寿命のほぼ全てを決める（1日1時間走行なら**23時間が待機**）。

| 仕組み | 担当 | 効果 |
|---|---|---|
| 6D 50° で起床 | センサー | 振動では起きない。クランクが回ったときだけ |
| 60秒静止で 1.875 Hz | センサー | 待機の加速度計電流を大幅に削減 |
| サンプラーは INT1 待ちのみ | ファーム | 定期起床を廃止。駐輪中は完全に停止 |

**定期起床を廃止した代わりの安全策**が2つある。片方だけでは危ないので必ずセットで:

1. `lsm6dsv_arm_orientation_wake()` が書いた設定を**読み戻して検証**し、一致しなければ
   `-EIO`。`main()` はそれを見て `CADENCE_HOLD_FAULT` でサンプラーを**寝かせない**。
   壊れ方が「静かに永久停止」ではなく「電池は食うが動く」になる
2. **USB を挿せば必ず起きる**（`CADENCE_HOLD_USB`）。人間が手動で叩き起こす経路が常に残る

`tools/power_budget.py` に見積もりモデルがある。ただし**センサー電流は推定値**で、
LiPo の自己放電と保護回路も効くので、絶対値より比率を見ること:

```bash
python3 tools/power_budget.py --capacity-mah 200
```
