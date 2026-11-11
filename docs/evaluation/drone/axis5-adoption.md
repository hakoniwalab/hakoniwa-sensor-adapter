# 軸 5：入れやすさ（2026-11-01）

| 項目 | 箱庭 | Gazebo（gz-sensors 10.2.0） | Project AirSim（core_sim） |
|---|---|---|---|
| 追加の依存 | なし（センサのモデルは nlohmann/json だけを使う） | 実行に 37 ライブラリ・約 29 MB（libc・libstdc++ を除く。Ubuntu 24.04、arm64） | core_sim の依存の nng 1.11（2.x はヘッダが変わってビルドできない）・msgpack-cxx・Eigen・nlohmann/json。nng はリンクされるが、通信の口は開かない |
| 入れ方 | 箱庭ドローン PRO と一緒にビルドされる | Ubuntu は apt で 562 パッケージ（イメージ 1.48 GB）。macOS・Windows は conda-forge（pixi）で約 1.8 GB・245 パッケージ。Homebrew だと 89 パッケージを足し、protobuf・abseil を上げる | 固定したコミットから、使う 140 ファイル（約 1 MB）を取ってきて sha256 で照合し、49 ファイルをコンパイルする（静的ライブラリ約 6 MB）。リポジトリには入れない |
| ビルドのつまずき | なし | apt の gz-sensors の開発パッケージが、必須の `libgz-common7-profiler-dev` を連れてこない。IMU だけでも、CMake の設定が gz-rendering と OpenGL の開発ファイルを要求する | センサの作り方が非公開（`Robot` だけが作れる）なので、同じ名前の小さな作り手を置く。単体では呼ばれないシーン側の関数 16 個に代役が要る。msgpack を Boost なしで使う指定が要る。地磁気と気圧の設定の getter は、宣言だけで中身がない |
| ビルドできる OS | macOS・Linux・Windows（箱庭ドローン PRO と同じ） | macOS・Linux で確認。Windows は conda-forge にパッケージがある（ビルドは確かめていない） | macOS で確認（conda-forge のパッケージは Linux・Windows にもある。ビルドは確かめていない） |
| 実行時の副作用 | なし | センサを読み込むと、通信（gz-transport）のスレッドが立ち、マルチキャストで相手を探す。気圧はメッセージを待つ（最大 2 秒）。同じ PC のプロセスどうしで、通信の名前がぶつかりうる（軸 3） | 通信の口は開かない。アダプタ 1 つにつき、core_sim のメッセージの配送のスレッドが 2 本立つ（センサの値はそこを通らないので、待っているだけ） |
| プロセスの中の共有の状態 | なし（インスタンスごとに独立） | 乱数がプロセスに 1 つ | なし（インスタンスごとに独立）。種はいつも同じ |
| ピークのメモリ（軸 4 の測定のプロセス） | 26.4 MB | 49.6 MB | 25.9 MB |
