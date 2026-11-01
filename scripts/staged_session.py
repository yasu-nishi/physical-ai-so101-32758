"""Mechanical bridge for host-decided one-step experiments. Never chooses targets."""
import argparse
import asyncio
import json

import scenario_suite as runner

runner.STATE=runner.ROOT/'tmp/staged_session.local.json'
runner.CASES=runner.ROOT/'config/physical_ai/staged_cases.json'


def compact(response):
    if response['isError']:return response
    keys=('observation_id','program_id','action_id','state','last_action','stop_latched',
          'estimated_height_m','estimated_top_z_m','estimated_top_center_m','height_interval_m',
          'vertical_edge_residual_m','measurement_id','planned_duration_s','command_status',
          'work_position_m','assumed_plane_z_m',
          'gripper_open','ai_visual_verdict','reason','physics_during_deliberation','simulation_time_s')
    rows=[]
    for data in response['text']:
        if not isinstance(data,dict):rows.append(data);continue
        row={k:data[k] for k in keys if k in data}
        if 'sensors' in data:row['tool']=data['sensors']['end_effector']
        rows.append(row)
    return {**response,'text':rows}


runner.compact=compact


def main():
    p=argparse.ArgumentParser()
    sub=p.add_subparsers(dest='command',required=True)
    q=sub.add_parser('start');q.add_argument('case_id')
    q=sub.add_parser('call');q.add_argument('tool');q.add_argument('--args',default='{}')
    q=sub.add_parser('run');q.add_argument('program_id')
    q=sub.add_parser('move');q.add_argument('--args',required=True)
    q=sub.add_parser('finish');q.add_argument('outcome');q.add_argument('reason')
    a=p.parse_args()
    if a.command=='start':runner.start(a.case_id)
    elif a.command=='call':print(json.dumps(compact(asyncio.run(runner.call(a.tool,json.loads(a.args)))),ensure_ascii=False))
    elif a.command=='run':runner.run_program(a.program_id)
    elif a.command=='move':
        args=json.loads(a.args)
        args.setdefault('instruction',json.loads(runner.STATE.read_text())['case']['instruction'])
        result=asyncio.run(runner.call('prepare_cartesian_step',args))
        if result['isError']:print(json.dumps(result,ensure_ascii=False));return
        runner.run_program(result['text'][0]['program_id'])
    else:runner.finish(a.outcome,a.reason)


if __name__=='__main__':main()
