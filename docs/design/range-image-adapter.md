# 距離と画像のセンサアダプタの口（設計）

距離計、LiDAR（2D・3D）、深度カメラ、RGB カメラの口を決めます。
下調べ（Gazebo・Project AirSim・箱庭の比較）は [range-image-survey.md](range-image-survey.md) です。5 つのセンサの口（[imu-adapter.md](imu-adapter.md)、[mag-baro-gps-adapter.md](mag-baro-gps-adapter.md)）と同じ考え方で作ります。

## 1. 目的と範囲

- 用途ごとに口を分けます。

  | 口 | 用途（ドローン） | 似せる ROS の型 |
  |---|---|---|
  | `IRangefinderSensorAdapter` | 地面からの高さ、着陸、地形に沿った飛行、近くの障害物 | `sensor_msgs/Range` |
  | `ILidarSensorAdapter` | 2D：水平の障害物、壁沿いの飛行。3D：障害物を避ける、地図を作る | `sensor_msgs/LaserScan`、`sensor_msgs/PointCloud2` |
  | `IDepthCameraSensorAdapter` | 前の障害物、近い距離の形 | `sensor_msgs/Image`（32FC1）、`sensor_msgs/CameraInfo` |
  | `IRgbCameraSensorAdapter` | 見る、物を見つける、マーカーを見て着陸する | `sensor_msgs/Image`（rgb8）、`sensor_msgs/CameraInfo` |

- 2D と 3D の LiDAR は、同じ口です。縦の段が 1 つなら 2D です。
- 口は ROS に依存しません。データ型は ROS の型に似せ、フィールドの名前と意味をそろえます。ROS や箱庭の PDU への変換は、口の外（ホスト）で行います。
- 単位はメートル、角度はラジアンです。

## 2. ホストとアダプタの受け持ち

5 つのセンサとの違いは、**真値が「世界そのもの」**であることです。距離も画像も、世界にある物で決まります。そこで、世界を測る部分（レイを飛ばす、描く）はホストが受け持ち、アダプタはそれを呼び出します。

```text
ホスト：センサの姿勢（物理） ──────────────┐
ホスト：世界を測る部分（レイ・描画） ◀────┐ │
                                       │ ▼
アダプタ：測る周期・走査の並び → レイや描画を頼む → 範囲・ノイズ・分解能・当たらないときの扱い → 出力
                                                                                           │
ホスト：                                                                                    ▼
        ROS・PDU への変換 → 自動操縦・ログ・表示
```

| 受け持ち | 中身 | 理由 |
|---|---|---|
| ホスト | センサの姿勢（取り付けの姿勢を当てはめた後）、世界にレイを飛ばす、世界を描く、自分の機体を当たりから除く | 物理の世界を持っているのはホスト。アダプタは物理エンジンを知らずに済む |
| アダプタ | 測る周期、走査の並び（角度、段、回転）、範囲、ノイズ、分解能、当たらないときの扱い、レンズ、カメラの内部の値 | センサの性質そのもの。データシートの値で書くところ |
| ホスト | 出力を ROS や PDU に変える | 口の外の話 |

## 3. 決めたこと（原則）

### 3.1 世界を測る部分は、ホストが渡す

ホストは、作るときに「世界を測る部分」をアダプタへ渡します。

- **`IRayCaster`**（レイを飛ばす）：センサの軸で書いたレイの束（始点と向き）を受け取り、今の刻みのセンサの姿勢から世界へ飛ばします。レイごとに、当たったか、距離、当たった物の番号、当たった面の向き（センサの軸）を返します。
- **`IImageRenderer`**（描く）：カメラの内部の値（画像の大きさ、視野、near・far）を受け取り、今の刻みのセンサの姿勢から、RGB または深度の画像を描いて返します。

箱庭ドローン PRO は、レイを MuJoCo の `mj_multiRay` で、描画を MuJoCo の描画で実装します。物理の当たり判定の世界を測ります（見た目は安く、当たり判定は実物の大きさで作る、という箱庭の決まりと合います）。

アダプタは、必要なほうを能力の表（`requires_ray_caster`、`requires_renderer`）で宣言します。ホストが持っていなければ、作るときに断ります。

### 3.2 真値：センサの姿勢と時刻

| 渡すもの | 座標 | 単位 |
|---|---|---|
| 時刻 | — | ns |
| センサの位置 | 世界 NWU | m |
| センサの姿勢（世界 NWU からセンサの軸への回転） | 四元数 | — |

- センサの軸は FLU（x 前、y 左、z 上）です。距離のセンサは x の向きが正面です。
- 姿勢は、出力に付けて返すためと、アダプタが点群を世界の座標でも出すために渡します。レイを飛ばすのはホストなので、アダプタが姿勢を使って計算する必要はありません。

### 3.3 時刻と走査（点ごとの時刻）

- 5 つのセンサと同じく、物理の刻みごとに `step` を呼びます（`sim_time_ns` = 0, dt, 2dt, …）。
- **回転しながら測る LiDAR は、その刻みに測るぶんのレイだけを飛ばします。** たとえば 10 Hz で回る LiDAR を 1 ms の刻みで動かすと、1 刻みに 1 周の 1/100 を測ります。1 周がそろったところで出力を `updated = true` にします。こうすると、機体が動いている間の歪みが自然に出ます。Gazebo にも Project AirSim にもない性質です。
- 点ごとに、走査の始まりからの時刻の差（`time_offset`）を持ちます。
- 一瞬で全部を測る形（`scan_mode = instant`）も選べます。Gazebo・Project AirSim と同じ測り方で、比べるときや軽くしたいときに使います。
- カメラは、撮影時刻の刻みで 1 枚を描きます（露出の時間による歪みは扱いません）。

### 3.4 当たらないとき・範囲の外

- 既定は ROS の REP 117 の決まりです。遠すぎる・当たらないときは +inf、最短より近いときは −inf です。
- 設定で「範囲の最大の値を入れる」形も選べます。
- ノイズは、範囲の中の値にだけ足し、足した後に範囲へ切り詰めます。切り詰めたときは `kSensorFlagSaturated` を立てます。

### 3.5 ノイズ・分解能

- 距離のノイズは、距離の範囲ごとに書けます。書き方は 2 通りです。
  - σ（m）
  - 距離の %（データシートの「測った距離の 3 %」など）
- 分解能（量子化）は、実際に効かせます。Gazebo の `range/resolution` はどこにも効きませんが、この口では効きます。
- 深度カメラのノイズも、同じ書き方で距離ごとに書けます。
- RGB のノイズは、画素ごとのガウス（平均と σ）から始めます。

### 3.6 深度の意味とカメラの軸

- 深度カメラは、次のどちらを出すかを設定ではっきり決めます。
  - `planar`：カメラの面からの距離（z の深さ）
  - `perspective`：カメラの点からレイに沿った距離
- 画像の軸は、カメラの光学の軸（x 右、y 下、z 前）です。行は上から下、列は左から右に並べます。センサの軸（FLU）から光学の軸への回転は口で決めた固定のもので、`CameraInfo` と一緒に出します。
- `CameraInfo` は必ず出します。中身は K（3×3）、歪みのモデルと係数、R、P（3×4）、画像の大きさです。

### 3.7 設定

- 5 つのセンサと同じく、アダプタは自分の形式の設定だけを読みます。ホストは自分の設定を渡しません。
- 箱庭のアダプタの設定は、SDF 風の JSON にします。構造は Gazebo の `scan`・`range`・`noise` に寄せ、データシートの距離ごとの精度（hakoniwa-mujoco-robots の形）も書けるようにします。
- AirSim の名前との対応は、次のとおりです（設定を読む道具で書き換えます）。

  | AirSim | 箱庭 |
  |---|---|
  | `number-of-channels` | `scan.vertical.samples` |
  | `points-per-second` ÷ `horizontal-rotation-frequency` ÷ チャンネル数 | `scan.horizontal.samples` |
  | `horizontal-rotation-frequency` | `update_rate` |
  | `horizontal-fov-start-deg`・`horizontal-fov-end-deg` | `scan.horizontal.min_angle`・`max_angle`（ラジアン） |
  | `vertical-fov-lower-deg`・`vertical-fov-upper-deg` | `scan.vertical.min_angle`・`max_angle`（ラジアン） |
  | `range` | `range.max` |
  | 距離計の `min-distance`・`max-distance` | `range.min`・`range.max` |
  | カメラの `width`・`height`・`fov-degrees` | `image.width`・`image.height`・`horizontal_fov`（ラジアン） |
  | `DepthPlanar`・`DepthPerspective` | `depth.type = planar`・`perspective` |

- データシートの値（ミリメートル、%）は、設定を読むところでメートルに直します。

設定の例（3D LiDAR）：

```json
{
  "type": "lidar",
  "update_rate": 10,
  "scan_mode": "rotating",
  "scan": {
    "horizontal": { "samples": 1800, "min_angle": -3.14159265, "max_angle": 3.14159265 },
    "vertical":   { "samples": 16,   "min_angle": -0.26179939, "max_angle": 0.26179939 }
  },
  "range": { "min": 0.1, "max": 100.0, "resolution": 0.002 },
  "accuracy": [
    { "range": { "min": 0.1, "max": 1.0 },   "stddev": 0.03 },
    { "range": { "min": 1.0, "max": 100.0 }, "percent": 0.3 }
  ],
  "no_return": "inf"
}
```

### 3.8 乱数とリセット

- 5 つのセンサと同じ決まりです。種、機体の名前・番号、センサの名前から、乱数の列を作ります。
- `reset` で、走査の途中の状態、たまった点、乱数の列を、作ったときの状態に戻します。

### 3.9 出力の大きさ

- 点群と画像は大きいので、アダプタが持つバッファへの参照で返します。参照は、次の `step` まで有効です。ホストはその間に写します。
- 大きさの上限は、作るときに決まります（走査の並び、画像の大きさから）。`step` の途中で増えることはありません。

## 4. 口の形

ヘッダは次の 4 つです（`include/hakoniwa/sensor_adapter/`）。

| ヘッダ | 中身 |
|---|---|
| `range_image_sensor_adapter.hpp` | 共通：`Ray`・`RayHit`・`IRayCaster`、`RenderRequest`・`RenderResult`・`IImageRenderer`、`RangeImageAdapterSetup`、`SensorPoseTruth`、`CameraInfo` |
| `rangefinder_sensor_adapter.hpp` | 距離計：`RangeSample`、能力の表、`IRangefinderSensorAdapter` |
| `lidar_sensor_adapter.hpp` | LiDAR：`LidarPoint`、`LidarScan`、能力の表、`ILidarSensorAdapter` |
| `camera_sensor_adapter.hpp` | 深度と RGB：`DepthImage`、`RgbImage`、能力の表、`IDepthCameraSensorAdapter`、`IRgbCameraSensorAdapter` |

センサの軸から光学の軸への回転は、`v_optical = R(q) v_sensor` で q = (0.5, 0.5, −0.5, 0.5) です（前 → +z、左 → −x、上 → −y）。

### 4.1 世界を測る部分（ホストが実装する）

```cpp
struct Ray {
    Vector3 origin;     // センサの軸 [m]（ふつうは 0。円錐やレーザーの間隔のぶんずらす）
    Vector3 direction;  // センサの軸、単位ベクトル
};
struct RayHit {
    bool hit{false};
    double distance{0.0};   // [m]
    std::int32_t object_id{-1};
    Vector3 normal{};       // 当たった面の向き、センサの軸
};
class IRayCaster {
public:
    virtual ~IRayCaster() = default;
    // 今の刻みのセンサの姿勢から、rays を max_distance まで飛ばす。自分の機体には当たらない。
    virtual bool cast(const Ray* rays, std::size_t count, double max_distance, RayHit* hits,
                      std::string& error) = 0;
};

struct RenderRequest {
    std::uint32_t width{0}, height{0};
    double horizontal_fov{0.0};  // [rad]
    double near_clip{0.0}, far_clip{0.0};  // [m]
    bool rgb{false};
    bool depth{false};           // planar（カメラの面からの距離）[m]
};
struct RenderResult {
    const std::uint8_t* rgb{nullptr};  // rgb8、行は上から下
    const float* depth{nullptr};        // [m]、範囲の外は +inf
};
class IImageRenderer {
public:
    virtual ~IImageRenderer() = default;
    // 今の刻みのセンサの姿勢から描く。結果は次の render まで有効。
    virtual bool render(const RenderRequest& request, RenderResult& result, std::string& error) = 0;
};
```

### 4.2 作るときに渡すもの

```cpp
struct RangeImageAdapterSetup : SensorAdapterSetup {
    IRayCaster* ray_caster{nullptr};       // ホストが持たなければ nullptr
    IImageRenderer* renderer{nullptr};     // 同上
};
```

### 4.3 真値

```cpp
struct SensorPoseTruth {
    std::uint64_t sim_time_ns{0};
    Vector3 position_world{};                 // NWU [m]
    Quaternion orientation_world_to_sensor{};
};
```

### 4.4 出力

```cpp
// 距離計（sensor_msgs/Range に似せる）
enum class RadiationType : std::uint8_t { kUltrasound = 0, kInfrared = 1, kLaser = 2 };
struct RangeSample {
    bool updated{false};
    std::uint64_t sample_time_ns{0};
    RadiationType radiation_type{RadiationType::kLaser};
    double field_of_view{0.0};   // [rad]
    double min_range{0.0}, max_range{0.0};  // [m]
    double range{0.0};           // [m]、REP 117（±inf）
    double variance{0.0};        // [m²]（設定から分かるとき。分からなければ 0）
    std::uint32_t flags{0};
};

// LiDAR（sensor_msgs/LaserScan と PointCloud2 に似せる）
struct LidarPoint {
    float x, y, z;         // センサの軸 [m]
    float intensity;       // 能力の表で宣言。持たなければ 0
    std::uint16_t ring;    // 縦の段の番号
    float time_offset;     // 走査の始まりからの時刻 [s]
    std::int32_t object_id;
};
struct LidarScan {
    bool updated{false};
    std::uint64_t sample_time_ns{0};    // 走査の始まり
    SensorPoseTruth pose_at_end{};      // 走査が終わったときのセンサの姿勢
    // LaserScan の形（行 = 縦の段、列 = 横）。2D は縦が 1 段
    double angle_min{0.0}, angle_max{0.0}, angle_increment{0.0};
    double vertical_angle_min{0.0}, vertical_angle_max{0.0}, vertical_angle_increment{0.0};
    std::uint32_t horizontal_count{0}, vertical_count{0};
    double time_increment{0.0}, scan_time{0.0};  // [s]
    double range_min{0.0}, range_max{0.0};       // [m]
    const float* ranges{nullptr};       // vertical_count × horizontal_count、REP 117
    const float* intensities{nullptr};  // 同じ並び
    // PointCloud2 の形（当たった点と、選べば当たらない点）
    const LidarPoint* points{nullptr};
    std::uint32_t point_count{0};
    std::uint32_t flags{0};
};

// カメラ（sensor_msgs/CameraInfo に似せる）
struct CameraInfo {
    std::uint32_t width{0}, height{0};
    std::string distortion_model;   // "plumb_bob" など。歪みなしは空
    std::vector<double> d;          // 歪みの係数
    std::array<double, 9> k{};      // 3×3
    std::array<double, 9> r{};      // 3×3
    std::array<double, 12> p{};     // 3×4
    Quaternion sensor_to_optical{0.5, 0.5, -0.5, 0.5}; // センサの軸（FLU）から光学の軸（x 右、y 下、z 前）
};
enum class DepthType : std::uint8_t { kPlanar = 0, kPerspective = 1 };
struct DepthImage {
    bool updated{false};
    std::uint64_t sample_time_ns{0};
    DepthType depth_type{DepthType::kPlanar};
    std::uint32_t width{0}, height{0};
    const float* data{nullptr};     // [m]、行は上から下、REP 117
    CameraInfo info{};
    std::uint32_t flags{0};
};
struct RgbImage {
    bool updated{false};
    std::uint64_t sample_time_ns{0};
    std::uint32_t width{0}, height{0};
    std::uint32_t step{0};          // 1 行のバイト数
    const std::uint8_t* data{nullptr};  // rgb8、行は上から下
    CameraInfo info{};
    std::uint32_t flags{0};
};
```

### 4.5 アダプタの口

```cpp
class IRangefinderSensorAdapter : public ISensorAdapter {
public:
    virtual bool step(const SensorPoseTruth& truth, RangeSample& out, std::string& error) = 0;
    virtual RangefinderAdapterCapabilities capabilities() const = 0;
};
class ILidarSensorAdapter : public ISensorAdapter {
public:
    virtual bool step(const SensorPoseTruth& truth, LidarScan& out, std::string& error) = 0;
    virtual LidarAdapterCapabilities capabilities() const = 0;
};
class IDepthCameraSensorAdapter : public ISensorAdapter {
public:
    virtual bool step(const SensorPoseTruth& truth, DepthImage& out, std::string& error) = 0;
    virtual DepthCameraAdapterCapabilities capabilities() const = 0;
};
class IRgbCameraSensorAdapter : public ISensorAdapter {
public:
    virtual bool step(const SensorPoseTruth& truth, RgbImage& out, std::string& error) = 0;
    virtual RgbCameraAdapterCapabilities capabilities() const = 0;
};
```

作る関数は、5 つのセンサと同じく、アダプタの実装の側に置きます（例：`create_mujoco_lidar_sensor_adapter(const RangeImageAdapterSetup&, std::string& error)`）。

## 5. 能力の表

| 能力 | 距離計 | LiDAR | 深度 | RGB | 意味 |
|---|---|---|---|---|---|
| `requires_ray_caster` | ○ | ○ | ○ | | レイを飛ばす部分が要る |
| `requires_renderer` | | | ○ | ○ | 描く部分が要る（深度は、レイか描画のどちらか） |
| `noise` | ○ | ○ | ○ | ○ | ノイズ |
| `range_dependent_accuracy` | ○ | ○ | ○ | | 距離の範囲ごとの精度（σ と %） |
| `resolution` | ○ | ○ | ○ | | 分解能 |
| `cone` | ○ | | | | 円錐の中の何本かのレイで測る |
| `rotating_scan` | | ○ | | | 回転しながら測る（点ごとの時刻、動きによる歪み） |
| `intensity` | | ○ | | | 強度 |
| `object_id` | | ○ | ○ | | 当たった物の番号 |
| `lens_distortion` | | | ○ | ○ | レンズの歪み |
| `deterministic_with_seed` | ○ | ○ | ○ | ○ | 同じ種・同じ入力なら同じ値 |
| `thread_safe_across_instances` | ○ | ○ | ○ | ○ | 別のインスタンスを別のスレッドで動かしてよい |

## 6. ホスト（箱庭ドローン PRO）の側でやること

- `IRayCaster` を `mj_multiRay` で実装します。自分の機体の物には当たらないようにします（体の除外か、geom のグループで）。物の番号は、MuJoCo の geom から決めた、変わらない番号にします。
- `IImageRenderer` を MuJoCo の描画で実装します。最初は標準の描画（OpenGL）を使い、Filament が MuJoCo の配布版に入ったら、同じ口の裏で差し替えます。
- 出力を今の約束へ変えます。
  - 3D LiDAR：`lidar_points`（PointCloud2）、`lidar_pos`（姿勢）、`DroneService/LiDARScan`
  - 距離計：`Range`
  - 2D LiDAR：`LaserScan`
  - カメラ：`Image`、`CameraInfo`
- 作る順番は、距離計 → 2D LiDAR → 3D LiDAR → 深度カメラ → RGB カメラです。それぞれで、出力を測って確かめます（決めた形の物を置いて、距離・角度・点の位置が真値と合うか）。
