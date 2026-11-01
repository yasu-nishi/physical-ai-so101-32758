# カメラの設置位置がsimと違う場合

simと同じ場所に設置する必要はない。固定カメラごとにレンズ・位置・向きを校正し、画素を共通のロボット基準座標へ変換する。AIはそのカメラの実画像から対象と可視の辺を選ぶ。カメラを動かしたら外部校正を取り直す。ピクセルやsimの校正値を実機へコピーしない。

2026-11-01に位置・向き・画角を変えたMuJoCo 4ケースを追加した。その後、実機のUSB通信・関節読取りと机上カメラ画像のMCP取得まで確認した。実機の幾何校正と動作は未検証。以下の数値は実機の精度を示さない。最新の接続状態は[実機接続記録](REAL_CONNECTION.md)を参照。simの既存設定は保持する。

実RGBでは校正下書きを使ってスポンジ1個を確認した。実測を聞く前の640×360での推定は誤差区間が広く棄却し、その後の実測30mmとの差は5.70mmだった。1280×720の再撮影では参考計算29.60mmとなったが、同じ物体の実測を知った後の選択で、独立した未知高さの成功試行ではない。実機精度の検証フラグは変更していない。詳細と制約は[実RGBの高さ確認](RGB_REAL.md#実rgbでの高さ確認2026-11-01)を参照。以下のsim試行とは別集計。

さらに高さ20mmと事前に報告された硬い箱を撮影し、左右の角から19.24mm・19.77mmを計算した。値は推定後の比較にだけ使用したが、ホストには事前に既知なのでブラインド評価ではない。同じ1画像の2辺であり、独立した2試行には数えない。カメラ・支持面・モデルの検証フラグは変更していない。

## 追加した機能

- sim設定の`camera_override`で`position_m`、`target_m`、`up`、`fovy_deg`を指定。物体を動かさずカメラだけを変更し、レンダリングとMCPの校正を一致させる。既存の設定には影響しない。
- `calibrate_rgb.py height`で校正下書きと写真から高さ・上面中心を確認。JSONと選択位置を示すPNGを保存する。ロボット接続、動作、検証フラグの変更は行わない。
- `real_check.py --joints-only`でカメラを開かず既存校正との照合と関節読取りだけを実行。位置目標・トルク設定を書き込まない。失敗も`inspection_error.json`に保存する。
- `doctor`はmacOSのWCHを含むUSBシリアルを列挙し、実行時の環境変数を尊重する。デバイス固有IDや個人情報は診断出力へ含めない。

MuJoCoのカメラ変更はコンパイル前の設定だけで、物理データ内の物体位置を書き換えない。既存XMLの`xyaxes`指定を解除してから新しい向きを適用する。[MuJoCoの姿勢指定](https://github.com/google-deepmind/mujoco/blob/main/include/mujoco/mjspec.h)、[OpenCVのカメラ校正](https://docs.opencv.org/4.13.0/d9/d0c/group__calib3d.html)を参照。

## 実機で動かさず確認する

USBが認識されたら、まず関節だけを読む。出力は既存結果を上書きしない新しいディレクトリにする。

```sh
uv run --locked python -m so101_mcp doctor
uv run --locked python scripts/real_check.py --joints-only \
  --config config/physical_ai/real_staged.json --output artifacts/joints01.local
```

カメラだけの撮影にはロボットUSBは不要。`.env`でロボット作業台を撮るカメラのindexと解像度を確認してから撮る。

```sh
uv run --locked python scripts/calibrate_rgb.py capture \
  --output artifacts/camera01.local/raw.png
```

[RGB校正手順](RGB_REAL.md)で内部校正と机に対する外部校正を作り、`verified=false`の下書きのまま次へ進める。印刷板の実寸・基準位置・向きは実測する。手で測った台面の点と画像から求めた位置も照合する。

```sh
uv run --locked python scripts/calibrate_rgb.py rectify \
  --calibration config/physical_ai/camera_calibration_draft.local.json \
  --image artifacts/camera01.local/raw.png \
  --output artifacts/camera01.local/rectified.png
```

AIまたは操作者が**rectified.pngの元解像度**で同じ垂直辺の下端と上端を選ぶ。次のU/Vは選んだ画素へ置き換える。`--image`には元のraw.pngを渡す。補正済み画像を二重に補正しない。

```sh
uv run --locked python scripts/calibrate_rgb.py height \
  --calibration config/physical_ai/camera_calibration_draft.local.json \
  --image artifacts/camera01.local/raw.png \
  --bottom U_BOTTOM V_BOTTOM --top U_TOP V_TOP \
  --center U_CENTRE V_CENTRE --support-z-m 0 \
  --output artifacts/camera01.local/height01.json
```

上面が隠れているときは`--center`を省略して高さだけを返せる。机上の支持面はZ=0。積み重ねは独立に確認した支持面高さを与える。垂直辺や支持面が不明なら使わない。`height01.preview.png`で選択点を確認する。測定不能なら理由をJSONに残して終了コード2を返す。

ノギス・物差しによる独立した高さとの比較には`--reference-height-mm 実測値 --tolerance-mm 3`を追加する。参考高さは推定器へ渡さず、推定後の比較にだけ用いる。許容誤差を超えても終了コード2。通過しても自動で検証済みにしない。複数の場所・高さで照合し、[関節・手先の対応](STAGED.md)も別に確認する。

実機観測で校正を使うときは現物確認済みの設定だけを`camera_calibration.local.json`へ保存する。動作には従来どおり関節・作業範囲・手先の校正と操作者のローカルARMが必要。手首カメラの動的な外部校正は未実装。

## 今回の結果

同じ2物体・1配置を4つのカメラ位置で観測した。AI判断がすべて終わった後にのみ、別の評価処理で物体の正解状態を参照した。物体ID・寸法・深度を測定APIやAIの対象選択へ渡していない。

| 視点 | 赤の推定高さ | 青の推定高さ | 動作と結果 |
|---|---:|---:|---|
| 横側 | 37.55mm | 38.86mm | 計測のみ、移動なし |
| 低い斜め視点 | 37.11mm | 39.50mm | 計測のみ、移動なし |
| 正面寄り | 37.53mm | 38.41mm | 18指令完了。搬送中2回滑落、最終的に赤は箱内 |
| 真上 | 測定保留 | 測定保留 | 辺が見えず、測定・移動指令とも0件 |

独立評価の初期高さは赤37.67mm、青39.99mm。6測定の平均絶対誤差0.67mm、最大1.58mm。6/6が画素±2pxの区間に入った。同じ2物体の繰返しであり、6個の独立物体ではない。実カメラの誤差、照明、ぼけ、誤対応を含む精度評価ではない。

搬送は**試行1、制御された移載成功0**。初回に机へ落下し、画像から現在位置と高さを再測定して再把持した。再試行でも箱の上で把持が外れた。最終状態では赤が箱内で静止し、青は移動・接触していないが、予定した下降・開放による移載ではない。18指令の正常終了や最終的な箱内配置を、安定した搬送成功と混同しない。保持中の摩擦・把持位置・動作による滑りの改善が残る。

軌道時間合計30.24秒。AI判断待ちも物理計算は進むため、試行全体の時間とは異なる。詳細は[集計JSON](../../artifacts/camera_view_experiments.json)、[CSV](../../artifacts/camera_view_experiments.csv)、`artifacts/runs/camera_*`の全MCP応答・生成Python・画像・映像。

## 再現・録画

```sh
uv run --locked python -m so101_mcp serve --backend sim \
  --config config/physical_ai/camera_views/side.json --seed 402 \
  --transport streamable-http --run-dir artifacts/new_camera_trial
```

`camera_view_session.py`は記録用のMCPブリッジで、対象や画素を自動で選ばない。既存4ケースは上書きしない。追加試行は新しいIDと評価ルールを用意する。`summarize_camera_views.py`は今回の4ケース専用で、進行中に正解状態を読むことを拒否する。

```sh
# 全4ケースが終了した後だけ実行する。
uv run --locked python scripts/summarize_camera_views.py
uv run --locked python scripts/build_camera_review_video.py
uv run --locked python scripts/validate_camera_video.py
```

[確認動画](../../artifacts/camera_view_review.mp4)は1分28秒、実MuJoCo映像をsim時間1倍速で編集し、AI判断待ちをカット。静止画と説明字幕は後編集と明記。2回の落下も残し、`camera_video_edit.json`に元フレームを保存する。検証CLIは全フレームをデコードし、各カット中央の確認用画像を`tmp/camera_review.local`へ出す。字幕と映像を確認してから提出物へ含める。以前の7 runの成功数へ合算しない。既存3ページPDFと2分49秒動画は以前の7 runを説明する資料として保持する。
