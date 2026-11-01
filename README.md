# SO-101の視覚・言語によるロボット作業

## 準備

Python 3.12とuvを使用します。検証環境はApple Silicon搭載Macです。プロジェクトのルートディレクトリで実行してください。

```sh
uv sync --locked
uv run --locked python -m so101_mcp doctor
```

ルートに `.env` を作成し、APIキーを設定します。既存の `.env` がある場合は必要な項目だけ追記してください。

```dotenv
OPENAI_API_KEY=ご自身のAPIキー
OPENAI_MODEL=gpt-6-astra
```

指定モデルを利用できるAPIアカウントが必要です。実際のキーを含む `.env` は共有しないでください。

## シミュレーションの実行

```sh
uv run --locked python -m so101_mcp.api_agent \
  --backend sim \
  --model gpt-6-astra \
  --instruction "赤いブロックを箱に入れて" \
  --output artifacts/api_runs/trial_01.local
```

MuJoCoが起動し、観測、対象の選択、把持、搬送、結果確認を実行します。指示は `--instruction` で変更できます。`--output` には新規または空のディレクトリを指定し、再実行時は `trial_02.local` など別名にしてください。省略すると保存先を自動作成します。

設定を変更する場合は次のファイルを編集します。

- `config/physical_ai/api_agent.json`：モデル、API応答数、動作回数、時間、トークン数の上限。
- `config/physical_ai/sim_staged.json`：物体、カメラ、作業範囲、動作速度。別の設定は `--sim-config` で指定。

実行結果は指定した保存先に出力されます。

| ファイル | 内容 |
|---|---|
| `result.json` | 終了理由、AIの視覚判定、応答数、所要時間 |
| `events.jsonl` | 操作と応答のログ |
| `*.png` | 観測画像・動作プレビュー |
| `sim/programs/` | 生成した1動作分のPythonと軌道 |
| `sim/simulation.mp4` | シミュレーションの録画 |

## 停止

実行端末で `Ctrl-C` を押します。または、別端末から次を実行します。保存先を変更した場合はパスも合わせてください。

```sh
uv run --locked python -m so101_mcp stop \
  --run-dir artifacts/api_runs/trial_01.local/sim
```

再開するときは、新しい保存先で最初から実行します。

## 実機の観測・ドライラン

`.env` に `SO101_FOLLOWER_PORT`、`SO101_FOLLOWER_ID`、`SO101_CAMERA_INDEX` を設定し、使用する機体のLeRobot校正データを用意します。[関節対応](docs/physical_ai/JOINT_MAPPING.md)と[カメラ校正手順](docs/physical_ai/RGB_REAL.md)を確認してから、読み取り確認を実行します。

```sh
uv run --locked python scripts/real_check.py \
  --config config/physical_ai/real_staged.json
uv run --locked python -m so101_mcp serve --backend real \
  --config config/physical_ai/real_staged.json \
  --transport streamable-http --run-dir artifacts/real_dry.local
```

既定は観測・ドライランです。実機を動かすには、機体ごとの校正・制限の確認と、ローカル端末での明示的な動作開始が必要です。[実機の設定・検証手順](docs/physical_ai/STAGED.md)に従ってください。停止には `Ctrl-C`、または上記の停止コマンドで `--run-dir artifacts/real_dry.local` を指定します。

## 結果

| 検証 | 結果 |
|---|---|
| Astra APIによるシミュレーション搬送 | 同一初期配置で2試行。初回は累積トークン上限で中断。上限修正後、赤いブロックを箱へ搬送し、解放後の画像判定と独立した物理評価が一致。動作11回、API応答28回、所要132.75秒。 |
| 未知高さ・段階動作のシミュレーション | 5場面・7試行。高さによる選別を落下なしで完了。積上げは再試行で部分達成し、積んだ部品の取外しは落下後の再把持で最終状態を達成。開発中の停止や高さ判別の保留も記録。API搬送試験とは別集計。 |
| 高さ推定 | 重複配置を除く9対象で、絶対誤差平均1.49 mm、最大4.17 mm。理想的に校正したシミュレーション内の結果。 |
| SO-101実機 | 6モーターの接続、関節値とカメラ画像の取得、一部の関節動作と短いXYZ移動を確認。把持・搬送成功は0件。 |

詳細は[API試行の記録](artifacts/api_live_verification.json)、[未知高さの全試行](docs/physical_ai/STAGED_RESULTS.md)、[実機試験](docs/physical_ai/CARTESIAN_VALIDATION.md)に記載しています。

## 限界

- 高さ推定には、校正済みの斜めカメラ、既知の支持面、上下端が見える垂直な辺が必要です。任意形状や隠れた底面の高さを一般的に求める機能ではありません。
- 成功例は限定した物体・配置・指示での成立確認です。試行数が少なく、一般的な成功率や未知環境での性能を示すものではありません。
- 指令の完了とタスクの成功は別に記録します。シミュレーションの結果を実機の位置精度や搬送成功とは扱いません。
- 実機は手先モーターの入力電圧エラーで停止しました。原因箇所の特定、待機姿勢への復帰、把持・搬送は未完了です。API経由の実機動作も未検証です。
- 力制御、一般的な障害物回避、手首カメラの動的な校正には対応していません。Linux・Windowsでの実行は未検証です。

## ソフトウェアの検証

```sh
uv run --locked python -m pytest -q
```
