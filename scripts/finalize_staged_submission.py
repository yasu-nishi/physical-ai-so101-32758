"""Verify the review package and write a scoped delivery manifest. Never publish."""
import hashlib
import json
from pathlib import Path
import subprocess
import zipfile

from pypdf import PdfReader

ROOT=Path(__file__).resolve().parents[1]


def main():
    review=json.loads((ROOT/'submission/review_manifest.json').read_text())
    for item in review['files']:
        file=ROOT/'submission/public_repo'/item['path']
        assert file.stat().st_size==item['bytes']
        assert hashlib.sha256(file.read_bytes()).hexdigest()==item['sha256'],item['path']
        assert item['bytes']<100_000_000, 'Split large files before ordinary GitHub upload'
    with zipfile.ZipFile(ROOT/'submission/public_repo.zip') as archive:
        assert archive.testzip() is None
        assert len(archive.infolist())==review['file_count']
        for item in review['files']:
            assert hashlib.sha256(archive.read('public_repo/'+item['path'])).hexdigest()==item['sha256']
    experiment=json.loads((ROOT/'artifacts/staged_experiments.json').read_text())
    tests=json.loads((ROOT/'artifacts/test_results.json').read_text())
    real=json.loads((ROOT/'artifacts/real_connection_verification.json').read_text())
    commissioning=json.loads((ROOT/'artifacts/real_commissioning_verification.json').read_text())
    supply=json.loads((ROOT/'artifacts/real_supply_verification.json').read_text())
    assert supply['motor_register_writes']==0 and not supply['automatic_motion_restart']
    recent_supply=json.loads((ROOT/'artifacts/real_connector_press_verification.json').read_text())
    recovery=json.loads((ROOT/'artifacts/real_standby_recovery_verification.json').read_text())
    speed=json.loads((ROOT/'artifacts/real_speed_configuration_verification.json').read_text())
    assert recent_supply['motor_register_writes']==0 and recent_supply['all_observed_torque_off']
    assert not recovery['waiting_pose_reached'] and not recent_supply['restored_motion']
    calibration=json.loads((ROOT/'artifacts/real_camera_calibration_verification.json').read_text())
    assert real['connection_verified'] and real['mcp_image_content_verified']
    assert real['mode']=='dry_run' and real['physical_motion']=='not_performed'
    assert real['motor_goal_writes']==real['calibration_writes']==real['torque_writes']==0
    # The preceding assertions apply only to the historical read-only MCP check.
    assert commissioning['jogs_completed']==0 and commissioning['physical_pick_place_trials']==0
    assert commissioning['motor_goal_writes']==sum(r['acknowledged_register_writes'].get('Goal_Position',0) for r in commissioning['runs'])
    assert not commissioning['task_verification_flags_changed'] and not commissioning['sim_modified']
    edl=json.loads((ROOT/'submission/staged_video_edit.json').read_text())
    assert edl==json.loads((ROOT/'artifacts/staged_video_edit.json').read_text())
    validation=json.loads((ROOT/'artifacts/staged_video_validation.json').read_text())
    for v in validation:
        assert v['passed'] and hashlib.sha256((ROOT/v['path']).read_bytes()).hexdigest()==v['sha256']
        if v['path']=='submission/staged_unknown_height_demo.mp4':
            assert v['all_cut_midpoints_visually_checked']
    pdf='output/pdf/staged_unknown_height_review.pdf'
    pdf_validation=json.loads((ROOT/'artifacts/staged_pdf_validation.json').read_text())
    assert pdf_validation['all_pages_rendered_and_visually_checked']
    assert not pdf_validation['layout_defects_observed']
    assert hashlib.sha256((ROOT/pdf).read_bytes()).hexdigest()==pdf_validation['sha256']
    assert len(PdfReader(ROOT/pdf).pages)==pdf_validation['pages']
    video='submission/staged_unknown_height_demo.mp4'
    probe=json.loads(subprocess.check_output(['ffprobe','-v','error','-show_entries','format=duration','-of','json',str(ROOT/video)]))
    duration=float(probe['format']['duration'])
    assert 60<=duration<=180 and abs(duration-edl['duration_s'])<.05
    api_video=json.loads((ROOT/'artifacts/api_video_validation.json').read_text())
    assert api_video['all_frames_decoded'] and api_video['visual_review']
    assert api_video['decoded_frames']==api_video['source_frames']+300
    assert 60<=api_video['duration_s']<=180
    for key in ['path','source']:
        hash_key='sha256' if key=='path' else 'source_sha256'
        assert hashlib.sha256((ROOT/api_video[key]).read_bytes()).hexdigest()==api_video[hash_key]
    paths=[pdf,video,api_video['path'],'submission/public_repo.zip','submission/staged_video_edit.json',
           'artifacts/api_live_verification.json','artifacts/api_video_validation.json']
    camera=json.loads((ROOT/'artifacts/camera_view_experiments.json').read_text())
    camera_video=json.loads((ROOT/'artifacts/camera_video_edit.json').read_text())
    for entry in json.loads((ROOT/'artifacts/camera_video_validation.json').read_text()):
        assert entry['passed'] and hashlib.sha256((ROOT/entry['path']).read_bytes()).hexdigest()==entry['sha256']
        if entry['path']==camera_video['video']:
            assert entry['all_cut_midpoints_visually_checked'], 'Inspect the edited camera video before finalizing'
    assert hashlib.sha256((ROOT/camera_video['video']).read_bytes()).hexdigest()==camera_video['sha256']
    paths += [camera_video['video'],'artifacts/camera_view_experiments.json','artifacts/real_connection_verification.json',
              'artifacts/real_commissioning_verification.json','artifacts/real_camera_calibration_verification.json',
              'artifacts/real_supply_verification.json',
              'artifacts/real_standby_recovery_verification.json',
              'artifacts/real_speed_configuration_verification.json',
              'artifacts/real_connector_press_verification.json']
    data={'review_only':True,'published':False,'updated_date':'2026-11-01',
        'api_agent':json.loads((ROOT/'artifacts/api_client_verification.json').read_text()),
        'api_agent_live':json.loads((ROOT/'artifacts/api_live_verification.json').read_text()),
        'primary_pdf':{'path':pdf,'pages':len(PdfReader(ROOT/pdf).pages),'all_pages_visually_checked':True},
        'primary_video':{'path':video,'duration_s':duration,'fps':25,'speed':'1x simulation time; AI waiting periods cut',
            'all_frames_decoded':True,'cuts':len(edl['segments']),'visual_review':'All cut midpoints checked; caption update states partial real motion and zero real transfer success'},
        'staged_development':{k:v for k,v in experiment.items() if k not in ('cases','limitations')},
        'scope_limits':experiment['limitations'],
        'legacy_known_30mm_pick_place':{'trials':17,'successes':17,'included_in_staged_results':False,
            'detail':'fixed 4, continuous XY 4, RGB revalidation 1, additional conditions 8; legacy depth diagnostic excluded'},
        'camera_position_addendum':{k:v for k,v in camera.items() if k not in ('cases','limitations')},
        'camera_review_video':{'path':camera_video['video'],'duration_s':camera_video['duration_s'],
            'scope':'four additional camera cases; two drops retained; not included in original seven-run PDF/video'},
        'real':{'connection':'verified: six motors, matching existing calibration, measured joints and real RGB via MCP',
            'motion':'wrist/opening and four short Cartesian segments completed; standby return interrupted by ID6 voltage error; zero pick/place successes',
            'read_only_verification':real,
            'historical_initial_pose_commissioning':commissioning,
            'historical_supply_diagnostics':supply,
            'latest_supply_diagnostics':recent_supply,
            'standby_recovery':recovery,
            'speed_configuration_correction':speed,
            'current_constraint':'no spare supply or voltmeter; operator cannot perform direct-servo wiring comparison; motion server stopped',
            'pdf_video_scope':'Primary PDF page 4 and final video captions reflect latest real status. All video footage is simulation; see CARTESIAN_VALIDATION.md and SUPPLY_DIAGNOSIS.md.',
            'camera_calibration':'fixed RGB calibration and image/model registration used for partial motion; independent metric end-effector accuracy remains unverified',
            'historical_camera_calibration_evidence':calibration,
            'kinematics_calibration':'encoder-based FK changes observed; not independent metric position measurements',
            'cartesian_code':'shared API prepared; requires calibrated fixed RGB, verified mapping/workspace/model and attended ARM',
            'moving_wrist_camera':'unsupported for Cartesian steps; dynamic hand-eye update not implemented'},
        'tests':{'source_suite_passed':tests['tests'],'source_failures':tests['failures'],
            'exported_package_passed':tests['exported_package_tests']['passed'],
            'hardware_observation_tests':True,'hardware_motion_tests':'partial joint and Cartesian motion observed; recovery incomplete',
            'hardware_pick_place_successes':0,'warning':tests['warning']},
        'package':{'files':review['file_count'],'hashes_and_zip_verified':True,
            'largest_file_bytes':max(v['bytes'] for v in review['files']),
            'privacy_scan':review['scan'],'offline_staged_scoring_reproduced':True},
        'artifacts':[{'path':p,'bytes':(ROOT/p).stat().st_size,'sha256':hashlib.sha256((ROOT/p).read_bytes()).hexdigest()} for p in paths]}
    data['conversation_video_supplement']=data['primary_video']
    data['primary_video']={k:api_video[k] for k in ('path','duration_s','fps','speed','all_frames_decoded','visual_review')}
    data['primary_video']['scope']='Actual gpt-6-astra API run; all recorded source frames, six-second intro and outro, post-edited captions. Earlier token-budget interruption also disclosed.'
    data['real']['pdf_video_scope']='PDF page 4 records partial real motion and zero real transfer success. Primary API video is simulation only; API real motion is unverified. Conversation video supplement retains prior real-status captions.'
    (ROOT/'submission/deliverables.json').write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps({'status':'review_only_not_published','files_verified':review['file_count'],
        'pdf_pages':data['primary_pdf']['pages'],'video_s':data['primary_video']['duration_s'],'source_tests':tests['tests'],
        'exported_tests':tests['exported_package_tests']['passed']},indent=2))


if __name__=='__main__':main()
