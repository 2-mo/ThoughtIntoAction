"""Live X5 control with a trained OpenPI PyTorch/JAX checkpoint and its own statistics."""
import argparse
import json
import os
from pathlib import Path
import signal
import sys
import time
from embodied.policy.pi05_sft.checkpoint import ROOT,DEFAULT_CHECKPOINT,inspect_checkpoint,DEFAULT_JAX_CHECKPOINT,inspect_jax_checkpoint
import numpy as np
from embodied.core.model import RobotModel
from embodied.core.project import load_config,resolve_path
from embodied.sensors.client import SharedCamera
from embodied.core.collection_motion import collection_motion
from legacy.robot_control.client import ArmClient
from embodied.policy.pi05_zeroshot.real import move_pregrasp,stop_and_hold
from embodied.policy.pi05_zeroshot.rollout import connect,control_loop,capture_owned,DEFAULT_PROMPT
from embodied.policy.pi05_zeroshot.worker_client import PolicyWorker


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--checkpoint',type=Path)
    p.add_argument('--backend',choices=('pytorch','jax'),default='pytorch')
    p.add_argument('--jax-horizon',type=int,default=15,help='JAX 训练 action_horizon，按项目适配器默认 15')
    p.add_argument('--prompt',default=DEFAULT_PROMPT)
    p.add_argument('--seconds',type=float,default=60)
    p.add_argument('--chunk-steps',type=int,default=10)
    p.add_argument('--speed',type=float,default=2.,help='关节速度上限 rad/s，默认及最大 2')
    p.add_argument('--gripper-speed',type=float,default=.03)
    p.add_argument('--state-quantile-limit',type=float,default=1.1,
                   help='状态统计超出量上限，默认 1.1；不是机械限位')
    p.add_argument('--preview',action='store_true',help='先到预抓取位并张开夹爪，再预测；不执行模型动作')
    p.add_argument('--output',type=Path)
    args=p.parse_args()
    if not np.isfinite(args.state_quantile_limit) or args.state_quantile_limit <= 0:
        p.error('state-quantile-limit 必须为有限正数')
    args.checkpoint = (args.checkpoint or (DEFAULT_JAX_CHECKPOINT if args.backend == 'jax' else DEFAULT_CHECKPOINT)).expanduser()
    spec=inspect_jax_checkpoint(args.checkpoint,horizon=args.jax_horizon) if args.backend == 'jax' else inspect_checkpoint(args.checkpoint)
    args.checkpoint=Path(spec['checkpoint'])
    if (not args.prompt.strip() or not np.isfinite([args.seconds,args.speed,args.gripper_speed]).all()
        or not 0<args.seconds<=300 or not 1<=args.chunk_steps<=spec['model']['action_horizon']
        or not 0<args.speed<=2. or not 0<args.gripper_speed<=.05):
        p.error('参数越界：seconds≤300, chunk-steps≤horizon, speed≤2, gripper-speed≤0.05，均须正值')
    output=args.output or ROOT/'outputs/infer_after_sft'/time.strftime('%Y%m%d-%H%M%S')
    output.mkdir(parents=True,exist_ok=False)
    (output/'checkpoint.json').write_text(json.dumps(spec,ensure_ascii=False,indent=2)+'\n')
    (output/'norm_stats.json').write_bytes(Path(spec['stats']).read_bytes())
    python=Path(os.environ.get('PI05_JAX_PYTHON' if args.backend == 'jax' else 'PI05_SFT_PYTHON',
                               str(ROOT/'outputs/envs'/('pi05-jax' if args.backend == 'jax' else 'pi05-sft')/'bin/python')))
    if not python.is_file():
        raise RuntimeError('请先运行 bash src/embodied/policy/pi05_sft/' + ('setup_jax.sh' if args.backend == 'jax' else 'setup.sh'))
    print(f'{args.backend.upper()} SFT step={spec["step"]}，horizon={spec["model"]["action_horizon"]}，统计={spec["asset_id"]}\n任务：{args.prompt}\n输出：{output}',flush=True)
    # Inference opens only the wrist camera; X5Inputs zero-fills and masks the base image.
    if spec['cameras'] is None:
        print('警告：checkpoint 未记录训练时的相机集合；推理只打开腕部相机。若训练用了外部相机，模型收到的外部图像为全零（mask=False）。',flush=True)
    elif spec['cameras'] != ['wrist_rgb']:
        print(f'警告：模型训练相机为 {spec["cameras"]}，推理只打开腕部相机；外部图像为全零（mask=False），继续运行。',flush=True)
    # Detect an unreleased controller before spending time loading/compiling the model.
    if resolve_path(load_config()['arm_service']['socket']).exists():
        probe=ArmClient(RobotModel(resolve_path(load_config()['robot']['urdf'])),hardware=True,actuate=False)
        try:
            state=probe.snapshot()
            (output/'service_before.json').write_text(json.dumps(probe.info,ensure_ascii=False,indent=2)+'\n')
            if probe.info['owner'] is not None or probe.info['mode']=='stopping':
                raise RuntimeError(f'机械臂服务尚不可用：mode={probe.info["mode"]}, owner={probe.info["owner"]}；'
                                   '请先处理原控制会话；未加载模型或开始运动')
        finally:probe.close()
    worker=PolicyWorker(output/'norm_stats.json',python=python,module='embodied.policy.pi05_sft.worker',
                        extra_args=('--checkpoint',str(args.checkpoint.resolve()),'--backend',args.backend,
                                    '--jax-horizon',str(args.jax_horizon)),
                        startup_timeout=600 if args.backend == 'jax' else 180)
    arm=None
    try:
        arm=connect(RobotModel(resolve_path(load_config()['robot']['urdf'])),output)
        arm.input_name='pi05-sft'
        arm.speed,arm.motion=collection_motion()
        (output/'pregrasp_motion.json').write_text(json.dumps(
            {'speed_rad_s':arm.speed,'motion':arm.motion},ensure_ascii=False,indent=2)+'\n')
        move_pregrasp(arm,open_gripper=True)
        if not args.preview:
            arm.speed=args.speed
            arm.motion.update(max_joint_acceleration_rad_s2=1.,max_joint_jerk_rad_s3=4.,max_command_lead_rad=.08)
            (output/'motion.json').write_text(json.dumps(
                {'speed_rad_s':arm.speed,'motion':arm.motion,'chunk_steps':args.chunk_steps},
                ensure_ascii=False,indent=2)+'\n')
            print(f'模型执行限速 {arm.speed:g} rad/s，加速度 1 rad/s²，jerk 4 rad/s³，'
                  f'每块执行 {args.chunk_steps} 步。',flush=True)
        c=load_config()['cameras']['wrist_rgb']
        with SharedCamera(c['serial'],c['width'],c['height'],c['fps']) as camera:
            if args.preview:
                cycle=output/'preview';cycle.mkdir()
                capture_owned(arm,camera,cycle)
                pred=worker.predict(cycle,args.prompt)
                print(f'预抓取位推理完成：{pred["action_shape"]}，{pred["inference_seconds"]:.3f}s；未执行模型动作。',flush=True)
            else:
                print('开始连续控制机械臂和夹爪；Ctrl+C 停止并保持。',flush=True)
                arm.start_control()
                try:
                    result=control_loop(arm,worker,camera,output,args.prompt,seconds=args.seconds,
                                        chunk_steps=args.chunk_steps,grip_speed=args.gripper_speed,
                                        state_quantile_limit=args.state_quantile_limit)
                    print(f'结束：{result["status"]}，已发 {result["commands"]} 步；服务保持。',flush=True)
                finally:stop_and_hold(arm)
    except BaseException:
        if arm is not None:
            try:
                arm.snapshot()
                (output/'service_error.json').write_text(json.dumps(arm.info,ensure_ascii=False,indent=2)+'\n')
            except Exception:pass  # Preserve the original control/stop failure.
        raise
    finally:
        if arm is not None:arm.close()
        worker.close()


if __name__=='__main__':
    def interrupt(signum,frame):raise KeyboardInterrupt
    signal.signal(signal.SIGTERM,interrupt)
    try:main()
    except KeyboardInterrupt:
        print('已中断本轮。',flush=True)
        sys.exit(130)
