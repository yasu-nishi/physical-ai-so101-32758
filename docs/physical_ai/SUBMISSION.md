# 録画・編集・提出前レビュー

締切：**2026年11月2日（月）10:00 JST**。課題指定はpublic GitHubのコード、1-3分のYouTube動画（限定公開以上）、説明PDF。倍速は明記。氏名・所属・メールアドレス・認証情報・社内情報を含めない。

## 現在の推奨提出物

提出コードには[API実行クライアント](API_AGENT.md)を追加済み。`python -m so101_mcp.api_agent --instruction ...`でMuJoCoとAPI判断役を起動できる。実Astra APIで2試行を行い、初回はトークン上限で中断、上限修正後に赤の搬送を完了した。動作11回・API応答28回・約133秒で、画像判断と独立物理評価が一致。PDF5頁へ追加し、既存の会話中Codexによる7 runと区別する。両API試行の画像・ログ・生成Python・原映像は公開用ZIPにも収録する。

- `output/pdf/staged_unknown_height_review.pdf`：目的、構成、AIとPythonの役割、未知高さ、段階生成、全7 runの結果、失敗・復旧。4頁は実機の部分動作と中断、5頁は実Astra APIの2試行。日時・ローカルのファイル位置・日時メタデータを除去済み。
- `submission/astra_api_demo.mp4`：今回の提出推奨動画。実Astra APIでの赤いブロックの搬送を2分36.8秒にまとめた。シミュレーション時間1倍速、API待機を含め全原フレームを保持し、前後に6秒ずつの説明と動作中の字幕を追加した。日時や個人情報を表示しない。
- `submission/staged_unknown_height_demo.mp4`：従来の会話中Codexによる補足動画。未知高さの選別、積上げ失敗からの再試行、取外し時の落下と再把持、判別保留を示す。Astra APIによる実験として扱わない。
- `submission/public_repo/`と`public_repo.zip`：コード、依存定義、設定、出典、実行ログ、PNG、生成Python、未編集映像を含む公開前レビュー用の選別済みフォルダ。
- `submission/review_manifest.json`：全収録ファイルのSHA-256と秘密情報パターン検査。`submission/deliverables.json`：結果・PDF頁数・動画長・検査結果。

追加レビュー資料は[カメラ設置位置の検証](CAMERA_POSITION.md)と`artifacts/camera_view_review.mp4`（1分28秒）。4視点の観測、搬送中の2回の滑落、制御された移載成功0/1を別集計で示す。コード・ログ・原映像とともに公開用ZIPへ収録する。PDF2頁と従来の補足動画は7 runを説明する資料として保持し、API版や追加カメラ結果を成功数へ合算しない。

実機は[接続記録](REAL_CONNECTION.md)に加え、[部分動作と復帰中断](CARTESIAN_VALIDATION.md)、[電圧診断](SUPPLY_DIAGNOSIS.md)を収録する。6モーター、既存校正との一致、実画像・関節値のMCP取得を確認し、手首・開口・共通XYZの一部動作まで実行した。待機復帰は電圧異常で中断、把持・搬送成功は0件。最新診断は全トルクOFFでID6が3.8Vを報告した。予備電源・電圧計と単独直結の比較は実施できず、原因箇所は未特定。動画は実取得のシミュレーション映像で、従来の補足動画の末尾字幕とPDF4頁に実機状況を記載した。API版の主動画では実機搬送を未検証と明記。実画像・校正値・デバイス固有情報は公開用ZIPに含めない。

旧PDF `explanation_PAI_final_review.pdf`、`scenario_suite_review.pdf`と旧動画は作業用の履歴として保存し、旧PDFは公開用ZIPへ収録しない。現在の提出には上記の新しい1本を使う。旧動画を連結すると3分を超える。校正用の`rgb_checkerboard.pdf`は実機準備用で、実寸印刷後に寸法を測る。提出PDFにはカレンダー日時・ローカルのファイル位置を載せず、作成・更新日時のメタデータも除去する。実験の所要時間と依存ソフトのバージョンは残す。

元の作業リポジトリにはローカル設定や過去の学習出力があるため、公開にはallowlistで選別したpublic_repoを使う。.env、実機校正、実カメラ画像、既存動画、個人の絶対パス、Git履歴は収録しない。今回の公開用画像・映像はシミュレーションのみ。

## 新しい実験の録画

1. `sim_staged.json`と新規run-dirでサーバーを起動。録画は標準で有効。初期安定化後から`simulation.mp4`へ25fpsで記録する。
2. [AI_OPERATOR.md](AI_OPERATOR.md)を接続AIへ渡す。毎回新しいRGBを表示し、measure_height、1段Python生成、実行、再観測を繰り返す。実際の指示、画像、推定値、失敗、時間を残す。
3. `report_result`で画像判断を記録し、`staged_session.py finish`またはCtrl-Cで正常終了し動画を閉じる。失敗runを消したり成功例と置換したりしない。
4. 全対象runのホスト判断が終了してからだけ`summarize_staged.py`を実行する。正解状態や初期寸法の再構築をAI判断へ流用しない。

`staged_session.py`は実MCPへの機械的な橋渡し。start / call / move / finishを提供するが、画像の選択や次の動作は決めない。既存runは上書き禁止。再試験はstaged_cases.jsonへ別case_idを追加する。本集計・レポートは収録7 run用なので、新しい試験にはタスクに対応する独立評価条件と集計の更新も必要。

## 編集とPDFの再生成

```sh
uv run --locked python scripts/build_staged_report.py --pdf-only
uv run --locked python -m scripts.build_api_video
uv run --locked python scripts/export_submission.py
```

提出PDFとAPI動画には、同梱したNoto Sans JP Regular / Boldを使う。出典とSIL Open Font License 1.1は `assets/fonts/` と出典一覧に記載。別の日本語TTFを使う場合は `PAI_REPORT_FONT` と `PAI_REPORT_FONT_BOLD` を指定する。PDFはPopplerで全頁をPNGへ描画し、文字・表・画像を目視確認する。

今回の最終検査は`finalize_staged_submission.py`でZIP内と公開フォルダの全ファイルhash、動画長、PDF頁数を照合した。API動画を変更した場合は全フレームを再デコードして字幕・冒頭・末尾・各動作を目視し、`artifacts/api_video_validation.json`を更新する。PDFも再描画し、`artifacts/staged_pdf_validation.json`を更新してからexportと最終検査を行う。補足動画の検査記録は`artifacts/staged_video_validation.json`。目視確認の記録はこの提出版についてのものなので、再編集した資料は再確認する。

提出推奨のAPI動画は、実取得のMuJoCo原映像をAPIの判断待ちも含めて全フレーム保持する。**sim時間1倍速**で再生し、冒頭・末尾に6秒ずつの説明を付ける。右側の動作字幕の区間は、ログのsimulation_time_sとtrajectory_sから求める。字幕は後編集の説明であり、APIの画面やモデル内部の思考過程ではない。

従来の段階生成の補足動画だけはAI判断待ちをカットし、動作部分をsim時間1倍速で使う。編集リストは`submission/staged_video_edit.json`。再生成する場合は`python scripts/build_staged_report.py`を実行する。こちらも原動画には待機中の保持・落下を残している。開発停止や失敗はPDF・集計に含め、成功例だけの成功率を示さない。

## 公開前と提出

公開・アップロード・提出はまだ行っていない。PDF全頁、新動画、公開用フォルダを確認できる状態まで用意する。

1. GitHubに新しいpublicリポジトリを作りpublic_repoの内容を登録。過去の履歴を引き継がず、コミット表示名・公開メールも確認する。
2. PDFを課題指定の`説明資料_PAI最終課題_<omnicampusアカウントID>.pdf`へ改名。未提供のIDや氏名を仮で埋めない。
3. 動画をYouTubeへ限定公開以上でアップロード。タイトルは`PAI最終課題_<omnicampusアカウントID>`。課題指定に従って子ども向け・年齢制限はいずれも「いいえ」、インスタントプレミア公開は使わない。採点終了まで削除・非公開化しない。
4. PDF、GitHub URL、YouTube URLをomnicampusへ提出し、別ブラウザから閲覧を確認する。

YouTube概要欄の文案：

```text
基盤モデルがRGBと日本語の指示から対象と次の1動作を選ぶSO-101システム。
校正済み単眼RGBの見える垂直辺から未知高さを推定し、1段分のPythonを生成・実行して再観測。
MuJoCo / Python / MCP / LeRobot。ROS・新規学習・サーバー内の別LLM呼出しなし。
この動画はgpt-6-astra APIによる「赤いブロックを箱に入れて」の完了試行。
同じ初期配置で2試行。初回は累積トークン上限で中断し、上限修正後に搬送を完了した。
完了試行は動作11回・API応答28回・約133秒。解放後の画像判断と独立した物理評価が一致。
一般的な成功率の評価ではない。未知高さの5シーン・7試行は別の実験として説明PDFに記載。
物体座標を書き換えず接触で把持・移載。AIの選択に物体ID・正解座標・深度を使わない。
直立直方体、既知支持面、可視の上下端、校正済みカメラに限定。任意形状には未対応。
実機6モーター、関節とRGBのMCP取得、一部の短い動作は確認済み。電圧異常で停止し、把持・搬送成功0件。
全トルクOFFでも異常を再現。原因箇所と独立した実機位置精度は未確定。詳細は説明PDF4頁とログに記載。
映像は2分36.8秒、sim時間1倍速。API待機も含めた全原フレームを保持し、説明字幕を後編集した。
モデル資産: Hugging Face lerobot/so101-tabletop-mujoco / MuJoCo Menagerie (Apache-2.0)。
コード: <公開後のGitHub URL>
```
