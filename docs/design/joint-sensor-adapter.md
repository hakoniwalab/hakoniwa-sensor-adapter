# 関節のセンサアダプタの口（設計）

ロボットの関節に付くセンサ（エンコーダと関節トルク）の口を決めます。
下調べ（シミュレータと実機の比較）は [joint-sensor-survey.md](joint-sensor-survey.md) です。今のセンサの口（[force-torque-adapter.md](force-torque-adapter.md) など）と同じ考え方で作ります。

この文書は、設定の値の意味と、誤差を足す順番と式まで決めます。同じ設定・同じ真値・同じ種なら、どの実装でも同じ意味の値を出すためです。

## 1. 目的と範囲

- 1 つの関節の角度・速度・トルクを測るセンサの口です。値は ROS の `sensor_msgs/JointState` の 1 関節分に似せます。
- 1 つのインスタンスは 1 つの関節です。同じ機種の関節が何本あっても、同じ設定ファイルを使えます（乱数はセンサの名前ごとに別）。
- 回転の関節を前提にします（単位は rad、rad/s、N·m）。

## 2. ホストとアダプタの受け持ち

```text
ホスト：物理の真値（関節の角度・速度、アクチュエータの力、関節を通るトルク）
                                                        │
アダプタ（物理の刻みごと）：                              ▼
        遅れ
アダプタ（測る周期の境目ごと）：
        角度：たわみ → ガタ → ずれ → 周期の誤差 → ノイズ → カウントに丸める → 1 回転で戻す → 関節の角度
        速度：真値か、丸めた角度の差分 → 単位で丸める
        トルク：トルクセンサか電流からの推定 → 目盛りの誤差・ずれ → ノイズ → 頭打ち → 分解能
                                                        │
ホスト：                                                 ▼
        制御器・ログ・PDU
```

| 受け持ち | 中身 |
|---|---|
| ホスト | 物理の真値。減速機は物理に入れない（関節の角度は、関節側の値） |
| アダプタ | 4 章の誤差のモデル。モーター側で測るときの減速比・ガタ・たわみも、アダプタが受け持つ |

## 3. 決めたこと（原則）

### 3.1 真値

ホストは、物理の刻みごとに、次の値を渡します。

| 渡すもの | 単位 | 意味 |
|---|---|---|
| 角度 q | rad | 関節の角度（物理の関節の値） |
| 速度 q̇ | rad/s | 関節の角速度 |
| アクチュエータの力 τ_act | N·m | その関節を回すアクチュエータの力の合計（MuJoCo なら `qfrc_actuator`） |
| 関節を通るトルク τ_tr | N·m | 関節の軸まわりに、親の側が子の側に及ぼすトルク（関節に付けたトルクセンサが測るもの）。静止して支えているときは τ_act と同じ |

ホストが関節を通るトルクを計算できないときは、τ_act を渡し、能力の表で知らせます（トルクセンサの値が摩擦を含まなくなる）。

### 3.2 時刻、乱数、リセット、ログ

今のセンサの口と同じです。

- 測る周期は、物理の刻みの整数倍に丸めます（ずれるときは警告）。時刻 0 から周期の境目で測り、その間は前の値を返します。
- 乱数は、種・機体・センサの名前から、特性ごとに別の列（`offset`、`accuracy`、`noise`、`torque_unit`、`torque_noise`）を作ります。
- `reset()` で、遅れの履歴、ガタの状態、増分式の始めの角度、速度の履歴、作るときに引いた個体の誤差、乱数の列を、作ったときに戻します。

### 3.3 値の印（flags）

- 頭打ち：共通のビット `kSensorFlagSaturated`。トルクが範囲の端で切られたときに立てます。

## 4. 誤差のモデル（式）

N は減速比（関節側で測るときは 1）、C は 1 回転あたりのカウント、T は測る周期。U(a, b) は一様分布、N(0, σ) は正規分布。

### 4.1 物理の刻みごと

| 段 | 式・決まり |
|---|---|
| 遅れ | `delay` だけ前の真値を使う（物理の刻みに切り捨て。履歴が足りない間は時刻 0 の真値） |

### 4.2 角度（測る周期の境目ごと）

遅れのあとの真値 q、τ_tr から、次の順に計算します。

| 段 | 式・決まり |
|---|---|
| たわみ | モーター側で、剛性 K > 0 のとき：θ = q + τ_tr / K。それ以外は θ = q（関節側の角度で表す） |
| ガタ | モーター側で、ガタの幅 b > 0 のとき、遊びの状態 p で：θ − p > b/2 なら p = θ − b/2、θ − p < −b/2 なら p = θ + b/2、それ以外は p のまま。最初の値で p = θ。以降は θ の代わりに p を使う |
| ずれ | 絶対値式：θ ← θ + o。o は作るときに U(−O, O) |
| 軸の角度 | s = N · θ（エンコーダの軸の角度） |
| 周期の誤差 | s ← s + A · sin(s + ψ)。A は作るときに U(0, `accuracy`)、ψ は U(0, 2π) |
| 増分式 | s ← s − s₀。s₀ は作ったあと・リセットのあとに最初に測った s |
| ノイズ | s ← s + N(0, σ_c · 2π / C)（σ_c はカウント単位） |
| カウント | c = round(s · C / 2π)（整数。0.5 は 0 から遠いほうへ） |
| 1 回転で戻す | `wrap` が `single_turn` のとき、c ← c mod C（0 から C−1） |
| 角度の値 | 角度の出力 = c · 2π / (C · N) |

`encoder` を書かなければ、角度は q そのまま、カウントは 0 です。

### 4.3 速度

| `method` | 式 |
|---|---|
| `true`（既定） | v = q̇（遅れのあとの真値） |
| `difference` | v = (c_k − c_{k−w}) · 2π / (C · N · w · T)。c は 1 回転で戻す前のカウント、w は `window`。まだ w 個前の値がないときは、あるうちで一番古い値と、その間の時間で割る。最初の値は 0 |

- `lsb` > 0 のとき、v ← lsb · round(v / lsb)。

### 4.4 トルク

| 段 | 式・決まり |
|---|---|
| 測るもの | `source` が `sensor`：x = τ_tr。`current`：x = τ_act + F_c · sgn(q̇) + F_v · q̇（電流は、摩擦の分も含めて流れるため。sgn(0) = 0） |
| 目盛りの誤差・ずれ | y = x · (1 + g) + b。g は作るときに U(−S, S)、b は U(−B, B) |
| ノイズ | y ← y + N(0, σ) |
| 頭打ち | \|y\| > R なら ±R で切り、頭打ちの印を立てる |
| 分解能 | q_τ > 0 なら y ← q_τ · round(y / q_τ) |

`torque` を書かなければ、トルクは出しません（`has_effort` が false）。

## 5. 設定

角度は数（rad）か、単位つきの文字列（`"0.25deg"`、`"15arcmin"`、`"18arcsec"`）で書けます。速度の単位は数（rad/s）か `"0.229rpm"`、割合は `"2%"`（範囲 R に対する割合）です。

| キー | 意味 | 省略したとき |
|---|---|---|
| `type` | `"joint"` | 必須 |
| `update_rate` | 測る周期 [Hz] | 必須 |
| `delay` | 遅れ [s] | 0 |
| `encoder.side` | `"joint"` か `"motor"` | `"joint"` |
| `encoder.gear_ratio` | 減速比 N（`motor` のときだけ使う） | 1 |
| `encoder.counts_per_rev` か `encoder.bits` | 1 回転あたりのカウント C（`bits` なら 2^bits） | 必須（`encoder` を書くとき） |
| `encoder.kind` | `"absolute"` か `"incremental"` | `"absolute"` |
| `encoder.wrap` | `"none"` か `"single_turn"` | `"none"` |
| `encoder.offset` | 絶対値式のずれの上限 O（関節側の角度） | 0 |
| `encoder.accuracy` | 周期の誤差の振幅の上限（エンコーダの軸の角度） | 0 |
| `encoder.backlash` | ガタの幅 b（関節側の角度。`motor` のときだけ） | 0 |
| `encoder.stiffness` | ねじりの剛性 K [N·m/rad]（`motor` のときだけ） | 0（たわみなし） |
| `encoder.noise_counts` | ノイズ σ_c [カウント] | 0 |
| `velocity.method` | `"true"` か `"difference"` | `"true"` |
| `velocity.window` | 差分の窓 w [サンプル] | 1 |
| `velocity.lsb` | 速度の単位 | 0（丸めない） |
| `torque.source` | `"sensor"` か `"current"` | 必須（`torque` を書くとき） |
| `torque.range` | 範囲 R [N·m] | 必須（`torque` を書くとき） |
| `torque.scale_error` | 目盛りの誤差の上限 S | 0 |
| `torque.bias` | ずれの上限 B（数か範囲の %） | 0 |
| `torque.noise` | ノイズ σ [N·m] | 0 |
| `torque.resolution` | 分解能 q_τ [N·m] | 0 |
| `torque.friction` | `current` のときの摩擦（`coulomb` [N·m]、`viscous` [N·m·s/rad]） | 0 |

データシートの「正確さ：最大トルクの ±2 %」は、`torque.bias` に `"2%"` と書きます。知らないキー、範囲の外の値、知らない単位は、作るときにエラーにします。

例（Feetech STS3215。SO-101 の関節）：

```json
{
  "type": "joint",
  "update_rate": 100,
  "encoder": {
    "side": "joint",
    "bits": 12,
    "kind": "absolute"
  },
  "velocity": { "method": "difference", "window": 1, "lsb": "0.01465rpm" }
}
```

STS3215 の速度の単位は 1 ステップ/s（50 ステップ/s = 0.732 rpm なので、1 ステップ/s ≈ 0.01465 rpm）です。STS3215 のデータシートは速度の作り方を書いていないので、`method` は仮に `difference` にしています。

## 6. 口の形

| 名前 | 中身 |
|---|---|
| `JointSensorAdapterSetup` | 作るときにホストが渡すもの（共通の `SensorAdapterSetup`） |
| `JointSensorTruth` | 物理の刻みごとの真値：時刻、角度、速度、アクチュエータの力、関節を通るトルク、関節を通るトルクを計算できたか |
| `JointSensorSample` | 1 刻みの出力：新しく測ったか、測った時刻、角度、速度、トルク、トルクを出すか、カウント、印 |
| `JointSensorAdapterCapabilities` | 持っている機能の宣言 |
| `IJointSensorAdapter` | `step`、`capabilities`。`reset`、`log_columns`、`log_values` は共通の `ISensorAdapter` から受け継ぐ |

## 7. 能力の表

| 項目 | 意味 |
|---|---|
| `delay` | 遅れ |
| `encoder_counts` | カウントに丸める |
| `motor_side` | モーター側で測る（減速比） |
| `backlash`、`stiffness` | ガタ、たわみ |
| `offset`、`cyclic_error`、`noise` | 角度のずれ、周期の誤差、ノイズ |
| `incremental`、`wrap` | 増分式、1 回転で戻す |
| `velocity_from_difference`、`velocity_lsb` | 差分で作る速度、速度の単位で丸める |
| `torque_sensor`、`torque_from_current` | トルクセンサ、電流からの推定 |
| `torque_errors` | トルクの目盛りの誤差・ずれ・ノイズ・頭打ち・分解能 |
| `deterministic_with_seed` | 同じ種・同じ入力・同じ呼び順で同じ値 |
| `thread_safe_across_instances` | 別のインスタンスを別のスレッドから同時に呼べる |

## 8. ホストの側でやること

- 物理エンジンから、関節の角度・速度、アクチュエータの力、関節を通るトルクを取り出します。MuJoCo なら、関節を通るトルクは、子の体の `cfrc_int` を関節の位置に移して関節の軸に写したもの（`mj_rnePostConstraint` で計算）です。
- 何本の関節のセンサも、関節ごとに 1 つのインスタンスで動かします。
- 出力を、関節の状態の PDU（`sensor_msgs/JointState`）に、関節の順にまとめて載せます。
