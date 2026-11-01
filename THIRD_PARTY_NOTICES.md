# 出典・ライセンス・固定バージョン

本課題で追加したコードはルートのApache-2.0ライセンスで配布する。第三者のモデル、コード、依存ライブラリにはそれぞれのライセンスを適用する。モデルを自作したとは主張しない。

## SO-101モデルと卓上環境

- 出典：[Hugging Face / lerobot/so101-tabletop-mujoco](https://huggingface.co/lerobot/so101-tabletop-mujoco)
- 取得リビジョン：`844dee6c058960c2ef42b65e2c68b47a5cf7c4f3`（2026-10-31取得）
- ライセンス：Apache-2.0。環境のモデルカードでwood.pngも同ライセンスと確認。
- 同梱先：`assets/so101_tabletop/`。モデルXML/STL、木目テクスチャ、上流READMEとLICENSEを保持。
- `robotstudio_so101/` は [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie) のコミット `0059d4335f8156206f63a35662313385f7ad6d74` に由来し、TheRobotStudio SO-ARM100の派生。個別LICENSEはそのディレクトリに保持。
- 変更箇所：上流卓上XMLを基に `config/scene.xml` を作成。4個のブロックを2個へ削減し、固定マーカー・カメラ・物理刻みを変更。`so101_mcp/sim.py` のモデル合成方法は上流 `env.py` を参照。ロボットのメッシュと接触ジオメトリは上流を使用し、表示色のみ実行時に変更。
- 独自実装：共通MCP API、非同期ローカル軌道、固定場所IK、画像ラベル、視覚判断の手順、LeRobotの観測専用接続、実機アーム手順、停止ラッチ、独立評価と提出パッケージ。

## 実機ライブラリと依存

[LeRobot](https://github.com/huggingface/lerobot) は既存のv0.5.1を維持。参照コミット `1396b9fab7aecddd10006c33c47a487ffdcb54b4`、Apache-2.0。現在のmainドキュメントとインストール済みソースを照合して、SOFollower / Feetech / OpenCVのAPIを使用した。校正ファイルは利用者のローカル環境に置き、公開物には含めない。

| 依存 | 検証版 | ライセンス（主要部分） |
|---|---|---|
| Python | 3.12.12 | PSF |
| LeRobot | 0.5.1 / 上記commit | Apache-2.0 |
| MuJoCo | 3.17.0 | Apache-2.0 |
| MCP Python SDK | 1.30.0 | MIT |
| NumPy | 2.2.6 | BSD-3-Clause |
| SciPy | 1.18.1 | BSD-3-Clause（同梱数値ライブラリは各ライセンス） |
| Pillow | 12.2.0 | MIT-CMU |
| OpenCV headless | 4.13.0.92 | Apache-2.0（同梱物は個別通知参照） |
| imageio / imageio-ffmpeg | 2.37.3 / 0.6.0 | BSD-2-Clause、FFmpegバイナリはそのビルド条件に従う |
| ReportLab | 5.1.0 | BSD |
| pytest / pypdf | 9.2.0 / 6.21.4 | MIT / BSD-3-Clause |

依存をソースに複製せず、`uv.lock` で再現する。FFmpegはH.264動画の生成・検査に使用し、実行バイナリは提出物へ同梱しない。

## 日本語フォント

提出用PDFとAPI実行動画の字幕には **Noto Sans JP Regular / Bold** を使用し、再生成用のTrueTypeフォントを `assets/fonts/` に同梱する。

- 出典：[Google Fonts / Noto Sans JP](https://github.com/google/fonts/tree/66a36c8c94b1a5d992ee4e7f392fccfe4945767c/ofl/notosansjp)
- 参照コミット：`66a36c8c94b1a5d992ee4e7f392fccfe4945767c`
- ライセンス：SIL Open Font License 1.1。原文を `assets/fonts/OFL-NotoSansJP.txt` に同梱。
- 原本の可変TTFからfontTools 4.60.1の `instantiateVariableFont` でウェイト400と700を固定。`updateFontNames=True` を指定し、全グリフを保持した。取得元URL・原本と派生ファイルのSHA-256は `assets/fonts/source.json` に記録。
- PDFには使用グリフを埋め込む。別フォントへ変更する場合は `PAI_REPORT_FONT` と `PAI_REPORT_FONT_BOLD` に日本語対応TTFを指定する。
- 過去の補足動画は収録時のArial Unicode字幕を保持する。OS付属フォント自体は同梱しない。

## 参照した一次資料

- [LeRobot Robot API](https://huggingface.co/docs/lerobot/main/api/robots)
- [LeRobot Action Representations](https://huggingface.co/docs/lerobot/action_representations)
- [ChatGPT / Codex MCP](https://learn.chatgpt.com/docs/extend/mcp)
- [Web版ChatGPTカスタムMCP](https://developers.openai.com/api/docs/guides/custom-mcp-server)
- [OpenCV 4.13 カメラ校正](https://docs.opencv.org/4.13.0/dc/dbb/tutorial_py_calibration.html)
- [OpenCV calib3d / solvePnP](https://docs.opencv.org/4.13.0/d9/d0c/group__calib3d.html)

最終確認日：2026-11-01。課題PDFにある締切・提出形式を確認し、公開前のレビュー用成果物として準備した。提出元の課題PDFや過去の学習データ・映像は同梱しない。
