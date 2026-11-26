"""Cross-platform command line; no shell, ROS clock or hardcoded workspace."""
import argparse
import json
from pathlib import Path
import sys


def main(argv=None):
    parser=argparse.ArgumentParser(prog='msim',description='msim 便携核心：回放、物理后端与原有完整仿真入口')
    sub=parser.add_subparsers(dest='command',required=True)
    demo=sub.add_parser('demo',help='生成确定性的测试数据，不进行物理仿真')
    demo.add_argument('--output',type=Path,required=True);demo.add_argument('--duration-s',type=float,default=24.)
    demo.add_argument('--seed',type=int,default=7);demo.add_argument('--no-attack',action='store_true')
    replay=sub.add_parser('replay',help='回放、运行检测并导出评价及独立 HTML 报告')
    replay.add_argument('--input',type=Path,required=True);replay.add_argument('--output',type=Path,required=True)
    replay.add_argument('--rate',type=float,default=0.,help='0 为尽快运行，1 为按记录时间运行')
    replay.add_argument('--align-lio-translation',action='store_true',help='仅评价时对位姿做首个匹配平移对齐')
    replay.add_argument('--assume-aligned-axes',action='store_true',help='显式确认不同位姿坐标系轴向一致；不进行旋转拟合')
    replay.add_argument('--maximum-match-ms',type=float,default=50.)
    replay.add_argument('--gps-sigma-m',type=float,default=.7);replay.add_argument('--acceleration-sigma-mps2',type=float,default=.25)
    replay.add_argument('--verify-clouds',action='store_true',help='逐帧读取并校验点云载荷；默认只回放元信息')
    inspect=sub.add_parser('inspect',help='流式检查便携记录和消息数量');inspect.add_argument('--input',type=Path,required=True)
    bag=sub.add_parser('import-bag',help='使用可选 rosbags 导入现有 msim ROS1 录包，无需安装 ROS')
    bag.add_argument('--input',type=Path,required=True);bag.add_argument('--output',type=Path,required=True)
    bag.add_argument('--duration-s',type=float);bag.add_argument('--include-clouds',action='store_true')
    bag.add_argument('--antenna-offset',type=float,nargs=3,default=[.05,0.,.04],metavar=('X','Y','Z'))
    sim=sub.add_parser('simulate',help='实时物理仿真；默认保留原有 Gazebo/PX4/FAST-LIO 链路')
    sim.add_argument('--backend',choices=('gazebo','bullet'),default='gazebo')
    sim.add_argument('--root',type=Path,help='Gazebo 完整后端的源码仓库路径')
    sim.add_argument('--dry-run',action='store_true',help='仅显示选择结果，不启动仿真')
    sim.add_argument('--output',type=Path);sim.add_argument('--world',type=Path)
    sim.add_argument('--model-root',type=Path,action='append',default=[])
    sim.add_argument('--sensor-profile',type=Path)
    sim.add_argument('--lidar-model',choices=('vlp16','hdl32','os1-64','mid360'))
    sim.add_argument('--duration-s',type=float,default=30.)
    sim.add_argument('--spawn',type=float,nargs=3,default=[0.,-20.,.08],metavar=('E','N','U'))
    sim.add_argument('--height-m',type=float,default=5.)
    sim.add_argument('--flight-pattern',choices=('hover','route'),default='route')
    sim.add_argument('--route-distance-m',type=float,default=15.)
    sim.add_argument('--control-source',choices=('ideal','gnss'),default='ideal',help='仅参考控制器：理想状态或延迟 GPS 水平/高程位置')
    sim.add_argument('--spoof-start-s',type=float,default=float('inf'))
    sim.add_argument('--spoof-rate-enu',type=float,nargs=3,default=[0.,0.,0.],metavar=('E','N','U'))
    sim.add_argument('--gps-rate-hz',type=float,default=5.)
    sim.add_argument('--gps-delay-s',type=float,default=.2)
    sim.add_argument('--real-time-factor',type=float,default=1.,help='1 按实时时间推进，0 尽快运行；不跳过物理步')
    sim.add_argument('--web',action='store_true');sim.add_argument('--port',type=int,default=8782)
    sim.add_argument('--serve-after-s',type=float,default=0.,help='任务完成后保留只读预览的秒数')
    sim.add_argument('gazebo_args',nargs=argparse.REMAINDER,help='原有完整仿真参数放在 -- 后，原样传递')
    args=parser.parse_args(argv)
    if args.command=='simulate':
        raw=list(sys.argv[1:] if argv is None else argv)
        prefix=raw[:raw.index('--')] if '--' in raw else raw
        args._provided_simulation_options=[v.split('=')[0] for v in prefix if v.startswith('--')]
    try:
        if args.command=='simulate':
            from .simulation import simulate
            result=simulate(args)
            if result.get('return_code',0):return result['return_code']
        elif args.command=='demo':
            from .demo import generate
            generate(args.output,args.duration_s,args.seed,not args.no_attack);result=dict(recording=str(args.output))
        elif args.command=='replay':
            from .replay import replay
            result=replay(args.input,args.output,args.rate,args.align_lio_translation,args.assume_aligned_axes,
                          args.maximum_match_ms,args.gps_sigma_m,args.acceleration_sigma_mps2,args.verify_clouds)
        elif args.command=='import-bag':
            from .bag_import import import_bag
            result=import_bag(args.input,args.output,args.duration_s,args.include_clouds,args.antenna_offset)
        else:
            from .recording import Recording
            from collections import Counter
            recording=Recording(args.input);counts=Counter(event.stream for event in recording.events())
            result=dict(metadata=recording.metadata,actual_counts=dict(counts),manifest_counts_match=dict(counts)==recording.metadata.get('counts'))
        print(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False))
        return 0
    except KeyboardInterrupt:
        parser.exit(130,'msim: 回放已中止，输入记录未修改；输出可能尚未完成。\n')
    except (ValueError,OSError,KeyError,TypeError) as error:
        parser.exit(2,f'msim: {error}\n')
