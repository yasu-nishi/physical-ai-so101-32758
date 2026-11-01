"""Offline staged-trial audit. Run only after ALL host decisions are closed."""
import csv
import json
from pathlib import Path
import sys
import tempfile

import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from so101_mcp.sim import SimBackend
from so101_mcp.evaluation import evaluate_sim

LABELS={
    'staged_01':'接近後に微小な関節限界超過で拒否。把持なし。',
    'staged_01b':'把持後、閉指令を保持する実装の問題を発見し中止。',
    'staged_01c':'赤を箱へ。搬送中に青を倒したため対象のみ成功。',
    'staged_02':'青の移設、赤の積上げは再試行で達成。続く取外しは関節限界で未着手。',
    'staged_03':'高さの画素誤差範囲が重なり、動作せず判別保留。',
    'staged_04':'上の赤の試験持上げで落下。画像から再推定・再把持し別置き達成。',
    'staged_05':'高い青を箱へ、低い赤を空き机面へ。落下なしで両方達成。',
}
INITIAL_COLOR_ORDER={
    'staged_01':['red'],'staged_01b':['red'],'staged_01c':['red'],
    'staged_02':['blue','red'],'staged_03':['blue','red'],
    'staged_04':['blue','red'],'staged_05':['blue','red'],
}


def table_rest(block):
    return bool(abs(block['bottom_z_m'])<.002 and block['speed_norm']<.02 and not block['gripper_contact_at_end'])


def main():
    cases=json.loads((ROOT/'config/physical_ai/staged_cases.json').read_text())
    if {c['case_id'] for c in cases} != set(LABELS):
        raise SystemExit('This report scores the seven archived runs. Add task-specific evaluation rules and summary counts before scoring new cases.')
    for c in cases:
        if not (ROOT/'artifacts/runs'/c['case_id']/'case_result.json').is_file():
            raise SystemExit('Finish all host decisions before accessing any hidden truth')
    rows=[];initials={}
    for case in cases:
        run=ROOT/'artifacts/runs'/case['case_id']
        host=json.loads((run/'case_result.json').read_text())
        manifest=json.loads((run/'run_manifest.json').read_text())
        events=[json.loads(s) for s in (run/'events.jsonl').read_text().splitlines()]
        obs=[e for e in events if e['event']=='observation'];byoid={o['observation_id']:o for o in obs}
        actions=[e for e in events if e['event']=='action_finished']
        programs=[e for e in events if e['event']=='cartesian_program_prepared']
        measures=[e for e in events if e['event']=='rgb_height_measured']
        assert all(not o['capabilities']['depth_sampling'] and o['physics_during_deliberation'] for o in obs)
        assert not any(e['event'] in ('depth_sampled','program_prepared') for e in events)
        for p in programs:
            assert len(p['trajectory'])==1 and p['automatic_next_step'] is False
            assert byoid[p['observation_id']]['utc'] < p['utc']
            assert (run/p['files']['script']).is_file()
        for action in actions:
            if action['command_status']=='completed':
                assert any(o['utc']>action['utc'] and o['status']['last_action']['action_id']==action['action_id'] for o in obs)
        # Initial reconstruction is only in this offline scorer, AFTER decisions.
        key=(case['seed'],case['config'])
        if key not in initials:
            with tempfile.TemporaryDirectory() as tmp:
                backend=SimBackend(manifest['config'],tmp,seed=case['seed'],record=False)
                try:backend.connect();initials[key]=evaluate_sim(backend)
                finally:backend.close()
        initial=initials[key]['blocks']
        truth_path=run/'evaluation_private.json'
        truth=json.loads(truth_path.read_text()) if truth_path.exists() else None
        b=truth['blocks'] if truth else None
        height_rows=[]
        for color,m in zip(INITIAL_COLOR_ORDER[case['case_id']],measures,strict=False):
            expected=initial[color]['full_size_m'][2]
            lo,hi=m['height_interval_m']
            height_rows.append({'color':color,'measurement_id':m['measurement_id'],
                'estimated_mm':1000*m['estimated_height_m'],'actual_initial_mm':1000*expected,
                'error_mm':1000*(m['estimated_height_m']-expected),'interval_mm':[1000*lo,1000*hi],
                'actual_in_pixel_interval':lo<=expected<=hi})
        physical={}
        cid=case['case_id']
        if cid=='staged_01c':
            physical={'red_in_box':b['red']['inside_box'],
                'blue_toppled':b['blue']['top_z_m']-b['blue']['bottom_z_m']<initial['blue']['full_size_m'][2]-.01}
        elif cid=='staged_02':
            checkpoint=json.loads((run/'checkpoint_stack_private.json').read_text())['blocks']
            r,blue=checkpoint['red'],checkpoint['blue']
            physical={'red_stacked_on_blue':bool(abs(r['bottom_z_m']-blue['top_z_m'])<.002
                and np.linalg.norm(np.array(r['position_m'][:2])-blue['position_m'][:2])<.01
                and r['speed_norm']<.02 and not r['gripper_contact_at_end'] and table_rest(blue)),
                'blue_relocated_m':float(np.linalg.norm(np.array(blue['position_m'][:2])-initial['blue']['position_m'][:2])),
                'unstack_completed':False}
        elif cid=='staged_04':
            physical={'red_on_table':table_rest(b['red']),'blue_on_table':table_rest(b['blue']),
                'separation_m':float(np.linalg.norm(np.array(b['red']['position_m'][:2])-b['blue']['position_m'][:2])),
                'red_destination_error_m':float(np.linalg.norm(np.array(b['red']['position_m'][:2])-[.2490066199,.0204394362])),
                'blue_displacement_m':float(np.linalg.norm(np.array(b['blue']['position_m'][:2])-initial['blue']['position_m'][:2])),
                'drop_observed_by_host':True}
        elif cid=='staged_05':
            taller=max(initial,key=lambda c:initial[c]['full_size_m'][2]);shorter='red' if taller=='blue' else 'blue'
            physical={'actual_taller':taller,'taller_in_box':b[taller]['inside_box'],
                'shorter_on_table':table_rest(b[shorter]),'shorter_outside_box':not b[shorter]['inside_box'],
                'shorter_displacement_m':float(np.linalg.norm(np.array(b[shorter]['position_m'][:2])-initial[shorter]['position_m'][:2])),
                'shorter_destination_error_m':float(np.linalg.norm(np.array(b[shorter]['position_m'][:2])-[.2490066199,.0204394362]))}
        clients=[json.loads(p.read_text()) for p in sorted((run/'client').glob('*.json'))]
        row={'case_id':cid,'category':case['category'],'seed':case['seed'],'instruction':case['instruction'],
            'host_outcome':host['host_outcome'],'host_reason':host['host_reason'],'summary_ja':LABELS[cid],
            'commands':len(actions),'commands_completed':sum(a['command_status']=='completed' for a in actions),
            'command_failures':sum(a['command_status']=='failed' for a in actions),
            'prepared_programs':len(programs),'measurements':len(measures),
            'trajectory_s':round(sum(a['trajectory_s'] for a in actions),3),
            'controller_wall_s':round(sum(a['wall_s'] for a in actions),3),
            'last_observation_sim_s':round(obs[-1]['simulation_time_s'],3),
            'initial_height_checks':height_rows,'physical_checks':physical,
            'before_image':str((run/f'observe_{obs[0]["observation_id"]}.png').relative_to(ROOT)),
            'after_image':str((run/f'observe_{obs[-1]["observation_id"]}.png').relative_to(ROOT)),
            'video':str((run/'simulation.mp4').relative_to(ROOT)),
            'ai_verdicts':[e for e in events if e['event']=='ai_visual_verdict'],
            'rejections':[t for c in clients if c['isError'] for t in c['text']],
            'physical_evidence':truth,'initial_state_offline_only':initial}
        rows.append(row)
    unique=[h for r in rows if r['case_id'] not in ('staged_01b','staged_01c') for h in r['initial_height_checks']]
    byid={r['case_id']:r for r in rows}
    assert byid['staged_01c']['physical_checks']['red_in_box']
    assert byid['staged_02']['physical_checks']['red_stacked_on_blue']
    assert byid['staged_04']['physical_checks']['red_on_table'] and byid['staged_04']['physical_checks']['blue_on_table']
    assert byid['staged_05']['physical_checks']['taller_in_box'] and byid['staged_05']['physical_checks']['shorter_on_table']
    data={'date':'2026-11-01','scope':'development evidence, not a held-out benchmark',
        'runs':len(rows),'distinct_scenes':len(initials),'clean_full_task_successes':1,
        'recovered_full_task_final_states':1,'target_only_with_collateral_motion':1,
        'partial_chains':1,'development_stops':2,'ambiguous_no_motion':1,
        'real_connection':'unverified','real_motion':'unverified','host_model':'current Codex session; exact model ID unrecorded; not an Astra benchmark',
        'depth_used':False,'physical_contact_only':True,'physics_during_deliberation':True,
        'all_decisions_finished_before_offline_scoring':True,
        'commands':sum(r['commands'] for r in rows),'commands_completed':sum(r['commands_completed'] for r in rows),
        'trajectory_s':sum(r['trajectory_s'] for r in rows),
        'initial_height_accuracy':{'unique_objects':len(unique),
            'mean_absolute_error_mm':float(np.mean([abs(h['error_mm']) for h in unique])),
            'maximum_absolute_error_mm':max(abs(h['error_mm']) for h in unique),
            'covered_by_pixel_only_interval':sum(h['actual_in_pixel_interval'] for h in unique),
            'range_actual_mm':[min(h['actual_initial_mm'] for h in unique),max(h['actual_initial_mm'] for h in unique)]},
        'limitations':['Upright rectangular visible vertical edges, calibrated ideal RGB and known support plane',
            'Pixel intervals exclude calibration/support uncertainty and wrong correspondences',
            'Placement used a conditional held-object top plane from measured size and robot FK; slip/rotation can invalidate it',
            'One-step IK uses joint interpolation; no general obstacle avoidance or force sensing',
            'Several controller fixes occurred during trials; failed runs preserved',
            'Close heights were ambiguous; unknown arbitrary geometry and general task success not demonstrated'],
        'cases':rows}
    (ROOT/'artifacts/staged_experiments.json').write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n')
    keys=['case_id','category','seed','host_outcome','commands','commands_completed','command_failures','trajectory_s','controller_wall_s','last_observation_sim_s','summary_ja']
    with (ROOT/'artifacts/staged_experiments.csv').open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=keys);w.writeheader();w.writerows({k:r[k] for k in keys} for r in rows)
    lines=['# 未知高さ・段階生成の実験記録','',
        '7 run・5初期シーンを実行。開発中の修正を含むため、一般的な成功率やAstraのベンチマークとは扱わない。全ケースのホスト判断終了後にだけ独立物理評価を実行した。',
        '', '| 試行 | 指令完了/実行 | 動作軌道 s | 観測までのsim時間 s | 結果 |','|---|---:|---:|---:|---|']
    for r in rows:lines.append(f"| {r['case_id']} | {r['commands_completed']}/{r['commands']} | {r['trajectory_s']:.2f} | {r['last_observation_sim_s']:.2f} | {r['summary_ja']} |")
    lines += ['', '完全な一連のタスクを落下なしで達成したのは高さ選別の1 run。落下後に最終状態を達成した取外しが1 run、他部品転倒を伴う対象搬送が1 run、積上げまでの部分達成が1 run。開発停止2 run、判別保留1 run。これらを一括の成功数にはしない。',
        '', 'staged_04は最初から重ねた別シーン。staged_02の取外し工程の継続成功ではない。staged_05は判別保留したstaged_03を消さずに追加した配置。',
        '', '## 初期画像からの高さ推定と、終了後の独立照合','',
        '| 試行 | 色 | RGB推定 mm | 独立正解 mm | 誤差 mm | 画素±2pxのみの区間 mm |','|---|---|---:|---:|---:|---|']
    for r in rows:
        if r['case_id'] in ('staged_01b','staged_01c'):continue
        for h in r['initial_height_checks']:
            lines.append(f"| {r['case_id']} | {h['color']} | {h['estimated_mm']:.2f} | {h['actual_initial_mm']:.2f} | {h['error_mm']:+.2f} | {h['interval_mm'][0]:.2f} - {h['interval_mm'][1]:.2f} |")
    acc=data['initial_height_accuracy']
    lines += ['',f"重複配置を除く{acc['unique_objects']}対象。絶対誤差平均{acc['mean_absolute_error_mm']:.2f}mm、最大{acc['maximum_absolute_error_mm']:.2f}mm。カメラ校正が理想的なsimの値で、実機精度を示さない。初期寸法は判断終了後にseedとmanifestから初期シーンを再構築して照合した。",'',
        'staged_04の青は上面中央が隠れたため、見える上端をcenter_pixelにも渡した旧呼出し。高さだけを使用し、そのXYは使っていない。現APIではcenter_pixel=nullで高さのみを明示できる。台の推定高さの誤差は次の高さ区間へ伝播していない。',
        '', '## 物理・制御と記録の区別','',
        '- AI入力はPNG・関節・開度・カメラ校正・関節からのrobot-only FK。物体ID/正解座標/深度は入力にしていない。',
        '- 各programは1経由点、automatic_next_step=false。生成Pythonを実行して新しいRGBを表示し、その都度ホストが次を選んだ。',
        '- 搬送・積上げ・落下は接触物理。初期配置以外で物体座標を書き換えていない。',
        '- AI待機中もidle_physics=trueで物理が進む。動作中は通常計算速度、待機中は概ね実時間。動画はsim時間1倍速。思考時間は制御計算時間とは別。',
        '- 指令完了数はタスク成功数ではない。青の転倒、積上げ失敗、取外し時の落下を視覚ログに保持。',
        '- staged_01の微小な実測限界超過、01bの把持閉指令保持、02の初期補間点の丸め超過を修正し回帰試験を追加。',
        '- この7 runはシミュレーションのみの集計。後続の実機接続・部分動作・搬送未達は[実機試験](CARTESIAN_VALIDATION.md)に別記する。配布用real_staged.jsonは全検証フラグfalseで、機体ごとの確認が必要。',
        '', '## 再実行','',
        'scripts/staged_session.py start staged_05 で1ケースを起動し、call observe / measure_height、move、画像確認を人または接続AIが順に行う。moveは次の動作を選ばない。既存runは上書きしないので、新規試験は別case_idとrunディレクトリで行う。本集計は収録7 run専用で、新しいタスク・caseを追加する場合は対応する独立評価条件と集計件数も更新する。全ホスト判断の終了後だけ集計する。',
        '', '詳細：[段階生成の構成と実機条件](STAGED.md)。元データ：artifacts/runs/staged_*、集計：artifacts/staged_experiments.json / .csv。']
    (ROOT/'docs/physical_ai/STAGED_RESULTS.md').write_text('\n'.join(lines)+'\n')
    print(json.dumps({k:v for k,v in data.items() if k not in ('cases','limitations')},ensure_ascii=False,indent=2))


if __name__=='__main__':main()
