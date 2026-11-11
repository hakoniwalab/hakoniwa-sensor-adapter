# センサの比較

この口（hakoniwa-sensor-adapter）の上で、箱庭のセンサと、ほかのシミュレータのセンサを比べた結果です。

## 比べ方

どの実装も、同じ口のアダプタとして包み、ホストから同じ真値の列を渡して、出力を並べます。物理は同じで、差はセンサの誤差のモデルの差だけになります。

```text
真値の列 ─┬─▶ 箱庭のアダプタ            ─┐
          ├─▶ Gazebo（gz-sensors）のアダプタ ├─▶ 出力を並べる
          └─▶ Project AirSim のアダプタ   ─┘
```

- 比べる軸は 6 つです。ノイズなしの一致、確からしさ、表せる範囲、再現性、速さ、入れやすさ。定義は [評価の軸](drone/evaluation-axes.md) です。
- 合否の基準は決めません。数値と、違いの理由を並べます。
- Gazebo と Project AirSim のアダプタは公開しています（hakoniwa-sensor-adapter-gazebo、hakoniwa-sensor-adapter-airsim）。その列は誰でも作り直せます。
- 箱庭の列は、箱庭の製品（箱庭ドローン PRO など）に入っているセンサのモデルで測りました。そのソースは公開していません。

## センサの種類ごとの結果

| センサ | 比べたもの | 結果 |
|---|---|---|
| ドローンのセンサ（IMU、地磁気、気圧、GPS） | 箱庭ドローン PRO のセンサ v2、gz-sensors 10.2.0、Project AirSim の core_sim | [ドローンのセンサ](drone/README.md) |
