# 未知高さの計測と、一段ずつのPython生成

接続先のChatGPT / Codexが、RGBと指示を見て「対象・測る辺・次の移動・再試行・停止」を決める。Pythonはカメラ幾何、robot-only FK/IK、関節補間、制限、停止を担当する。Astraはクライアントで選ぶ候補であり、サーバーは別のLLMを呼ばない。今回の実行はこのCodexセッションによるもので、Astra固有の性能測定ではない。

## 実行時に未知の高さをどう求めるか

必要なのは、校正済みの斜めRGBカメラ、既知の支持面、画像で上下端が見える同じ垂直辺。物体寸法は入力しない。

1. AIが同じ垂直辺の下端と上端を元画像の画素で指定する。
2. Pythonが下端の画素光線を支持面（机ならZ=0）と交差させ、下端のXYを求める。
3. 下端からの鉛直線と、上端画素の光線が最も近づく点から上端Zを計算する。
4. 高さは上端Zから支持面Zを引く。上面中央も見える場合、その画素を求めた上端Zへ投影しXYを得る。
5. 上面中央が隠れる場合は`center_pixel=null`とし、高さだけを得る。架空の中心を入力しない。

`measure_height`は選んだ辺を描いたPNG、推定高さ、画素±2pxによる区間、鉛直辺の整合残差を返す。辺が短い、ほぼ真上から見る、整合残差が大きい場合は拒否。**この区間は統計的な信頼区間ではなく、校正誤差・支持面高さの誤差・対応する辺の取り違えも含まない。** 重ねた台の高さは先に測るが、その誤差の自動伝播は未実装。

深度センサー、MuJoCoの深度バッファ、物体ID、正解位置は使用しない。これは条件を満たす直立直方体の単眼計測であり、任意形状・傾いた辺・隠れた下端から高さが分かる方式ではない。小さな高さ差で区間が重なった実験は、判別保留として動作を止めた。

参考：Criminisi et al., [Single View Metrology](https://www.robots.ox.ac.uk/~vgg/publications/2000/Criminisi00a/)（IJCV, 2000）。本実装は校正済みカメラ・既知支持面・鉛直辺に限定した幾何計算。論文コードの転載はない。

## 観測と生成Pythonを繰り返す

```sh
uv run --locked python -m so101_mcp serve --backend sim \
  --config config/physical_ai/sim_staged.json --seed 136 \
  --transport streamable-http --run-dir artifacts/staged_new
uv run --locked python scripts/mcp_call.py observe --output artifacts/staged_client
```

AIがPNGを表示して辺を選び、以下の大文字部分を実観測で置換する。

```sh
uv run --locked python scripts/mcp_call.py measure_height --args '{"observation_id":"OBS_ID","bottom_pixel":[U0,V0],"top_pixel":[U1,V1],"center_pixel":[UC,VC],"support_z_m":0,"visual_reason":"画像に見える同じ辺の説明"}'
uv run --locked python scripts/mcp_call.py prepare_cartesian_step --args '{"observation_id":"OBS_ID","target_xyz_m":[X,Y,Z],"gripper_open":null,"instruction":"元の指示","visual_reason":"直前画像の根拠","expected_visual_change":"次の画像で確かめる変化"}'
uv run --locked python artifacts/staged_new/programs/PROGRAM_ID.py
```

グリッパーだけなら`target_xyz_m=null`、`gripper_open=0..1`を指定する。開度はmmでも関節角でもない。生成Pythonは1つの動作をMCPへ送り、完了を待って新しいPNGを保存する。**次の動作は自動生成しない。ホストAIがその画像を実際に見てから再び生成する。** Python本文は決定的な雛形で、任意のLLM生成ソースをサーバー内で実行しない。

接近、下降、閉じる、10-15mm程度の試験持上げ、保持確認、搬送、解放、退避の各段階を、その時点の画像から選ぶ。滑り・落下・位置ずれが見えたら搬送を止め、見える位置・高さ・姿勢を測り直して次を決める。再試行回数は成功とは別に記録する。

積上げの試行では、最初の解放で赤が横へ落ちた。再把持後の画像から、保持された部品中心と手先フレームのずれを補正し、積上げを達成した。その補正には「測定済み寸法＋把持時と現在のFKから推定した上面平面」を使った。この平面は新たな高さ実測ではなく、保持姿勢が保たれるという仮定。滑り・回転で崩れるため一般的な物体追跡は未完成。

## ローカルで検査する範囲

| 項目 | sim_staged | real_staged初期値 |
|---|---:|---:|
| 1段のXYZ移動上限 | 60mm | 20mm |
| 1段の関節変化上限 | 45度 | 5度 |
| 関節速度上限 | 30度/s | 10度/s |
| XYZ距離から時間を決める公称速度 | 35mm/s | 10mm/s |
| 開度変化上限 | 0.7 | 0.1 |
| 1段の時間 | 0.8-5秒 | 0.8-5秒 |
| 画像の有効時間 | 120秒、別動作で失効 | 同左 |

XYZ目標は既定の`tool_orientation="down"`では下向きの手先姿勢に対するIKで解く。`tool_orientation="preserve_tilt"`は、現在の手先X軸と机面の角度、およびwrist_roll関節値を維持する。XY移動に伴ってpanが回転するので、完全な姿勢固定ではない。斜めに展開した実機を毎回2度ずつ操作せず、既存のCartesian上限内で1区間のXYZ移動を準備できる。対象の高さや干渉が未確認の最終接近では短い区間にし、各区間の新画像で次を決める。既存のsim場面・物理・下向き動作、realの上限やARMを変更していない。

空の手先の退避など、傾きの変化を許容する場合は`tool_orientation="position_only"`を使える。wrist_rollを実測角に保ち、追加で1個だけ`locked_joints=["elbow_flex"]`のように指定できる。手先X軸の向きの変化は最大5度、または`cartesian_steps.max_tool_axis_change_deg`に設定したより小さい値まで。ロックは指令角を保つ意味で、実測の追従確認や停止処理を無効化しない。指定するのは次の1段だけで、実行後の新画像から次を決める。実機の成立確認は[検証記録](CARTESIAN_VALIDATION.md)を参照する。

軌道は関節空間の補間で、手先が完全な直線を通る保証や、XYZ公称速度を経路全域で厳密に制限する検証はない。関節速度・範囲・時間はローカルで検査する。障害物の自動回避や力制御はないため、空間内の干渉をAIの画像判断だけで完全に防げるとは主張しない。

実機Cartesian移動では各制御周期に実測関節FKを確認し、補間中の名目FKとの誤差3mm超、開始点から指示と逆方向への移動1mm超で失敗・保持・停止ラッチにする。`tracking_tolerance_m`と`reverse_tolerance_m`は実機設定で定義する。これは関節追従の監視であり、カメラ校正の絶対精度・力・接触・障害物の検査ではない。[実機XYZ検証](CARTESIAN_VALIDATION.md)には、導入前の上昇失敗、記録再生による検出、新監視の実機再実行が未検証であることを分けて記録している。

把持中の実測開度は閉指令より大きくなり得る。腕を動かす際は直前の**閉指令**を維持し、実測開度を新しい開指令へ戻さない。simの実測値に限り0.02度の数値許容を設定し、初期補間点を厳密な指令限界へ丸める。目標・補間指令の範囲外を許すものではない。realではこの許容値を追加していない。

`idle_physics=true`によりAI待機中も接触・重力が進む。動作中は計算可能な速度、待機中は概ね実時間で進むため、動画のsim時間と実際の作業時計は異なる。ローカルSTOP、MCP stop、Ctrl-CはホストAIの応答を待たない。

## 実機へ接続するために残る作業

2026-11-01に実機6モーターとの通信、既存校正との一致、実測関節値とRGBのMCP取得を確認した。MCPとローカルSTOPによる待機中の停止ラッチも確認済み。RGB校正は草案と高さ比較まで実施したが、実物座標・FKの対応は未検証。初期姿勢の通電調整は追従監視・電圧監視で停止し、姿勢変更成功と把持・搬送成功は得られていない。`real_staged.json`の`mapping_verified`、`workspace_verified`、`real_kinematics_verified`はfalse。MCPからこれらを変更したりARMしたりする機能はない。[実機接続記録](REAL_CONNECTION.md)、[初期姿勢調整](COMMISSIONING.md)、[写真のみの高さ確認](CAMERA_POSITION.md)を参照。

1. followerの電源・USBと**固定した外部RGBカメラ**を接続し、`.env`にポート、既存robot ID、カメラindexを設定。`scripts/real_check.py --config config/physical_ai/real_staged.json`で読取りだけを確認する。
2. [関節対応](JOINT_MAPPING.md)の符号・ゼロ・開閉端点と運用限界を実測する。`real_staged.json`を`real_staged.local.json`にコピーし実機専用に設定する。
3. [RGB校正手順](RGB_REAL.md)で内部・外部パラメータ、歪み補正、机面の座標系を検証する。作業範囲内の独立した既知点と高さで誤差を測る。
4. 共通角からrobot-only MJCFへの差は`cartesian_steps.model_offset_deg`、モデル基準から机座標への変換は`model_to_work`で設定する。LeRobotとの`mapping.offset_deg`と二重に補正しない。MJCFの`gripperframe`に対応する現物の基準点と軸を測定する。
5. 少なくとも6姿勢、2方向以上と高さ差20mm以上で、関節読値と**モデルから独立に実測した**手先XYZ・tool X軸を記録。下記の読取り専用検査を実行する。合格しても自動で有効化しない。
6. 目視で座標・作業範囲・停止を確認した設定だけ検証フラグをtrueにする。ドライランを確認後、操作者がローカル端末で`--enable-motion`とARMを明示し、空荷・短い移動から把持試験へ進む。simの経由点をそのまま流用しない。

```sh
uv run --locked python scripts/check_kinematics.py \
  --config config/physical_ai/real_staged.local.json \
  --samples config/physical_ai/kinematics_samples.local.json
uv run --locked python -m so101_mcp serve --backend real \
  --config config/physical_ai/real_staged.local.json --transport streamable-http \
  --run-dir artifacts/real_staged_dry
# 現物での確認後、操作者だけが開始する。
uv run --locked python -m so101_mcp serve --backend real \
  --config config/physical_ai/real_staged.local.json --transport streamable-http \
  --enable-motion --run-dir artifacts/real_staged_motion
uv run --locked python -m so101_mcp stop --run-dir artifacts/real_staged_motion
```

検査入力は`{"samples":[{"pose":{"joints_deg":{5関節の名前:実測度数},"gripper_open":実測開度},"reference_position_m":[x,y,z],"reference_tool_x_axis":[dx,dy,dz]}, ...]}`。初期許容は位置3mm・軸3度で、用途に応じ実測で決める。モデル自身のFK出力をreferenceへ入れても現物検証にはならない。

手首カメラを動かす場合は毎姿勢の外部パラメータ更新が必要。現在はhand-eye校正と動的更新を未実装のため、realのCartesian操作ではwristを拒否する。手首カメラだけなら従来の一定観測姿勢＋教示動作に限定する。深度カメラを買えば自動的に完成するという設計でもなく、使う場合は別途校正と実機試験が必要。

全結果は[STAGED_RESULTS.md](STAGED_RESULTS.md)。これらの限定条件を超える未知形状、多数障害物、柔軟物、任意姿勢、多様なロボット作業への汎用対応は未検証。
