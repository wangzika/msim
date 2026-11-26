"""Deterministic availability-order replay; the detector never receives truth."""
import csv
import json
import math
from collections import Counter,deque
from pathlib import Path
import time

from .clock import ReplayClock
from .detector import PlanarInnovationDetector,horizontal_acceleration_values
from .evaluation import PositionEvaluator
from .recording import Recording
from .report import ReportSamples,write_report

FIELDS=['stamp_ns','stream','frame','east_m','north_m','up_m','truth_east_m','truth_north_m','truth_up_m','error_east_m','error_north_m','error_up_m']


def replay(source,destination,rate=0.,align_lio_translation=False,assume_aligned_axes=False,
           maximum_match_ms=50.,gps_sigma_m=.7,acceleration_sigma_mps2=.25,verify_clouds=False):
    if not math.isfinite(maximum_match_ms) or maximum_match_ms<0:raise ValueError('Invalid matching tolerance')
    if any(not math.isfinite(v) or v<=0 for v in (gps_sigma_m,acceleration_sigma_mps2)):raise ValueError('Detector sigmas must be positive')
    if assume_aligned_axes and not align_lio_translation:raise ValueError('--assume-aligned-axes requires --align-lio-translation')
    clock=ReplayClock(rate);recording=Recording(source)
    output=Path(destination);output.mkdir(parents=True,exist_ok=False)
    counts=Counter();skips=Counter();imu=deque(maxlen=512);detector=PlanarInnovationDetector(gps_sigma_m,acceleration_sigma_mps2)
    last_gnss=None;valid=alarms=0;first_alarm=None;first=last=None;samples=ReportSamples()
    labelled_normal=labelled_attack=false_alarms=attack_alarms=0
    attack_start=recording.metadata.get('labels',{}).get('attack_start_ns','unknown')
    began=time.perf_counter()
    with (output/'position-errors.csv').open('w',encoding='utf-8',newline='') as error_file,(output/'detector.jsonl').open('w',encoding='utf-8',newline='\n') as detection_file:
        errors=csv.DictWriter(error_file,fieldnames=FIELDS);errors.writeheader()
        def position_row(row):errors.writerow(row);samples.add(row)
        evaluator=PositionEvaluator(position_row,recording.metadata,int(maximum_match_ms*1e6),align_lio_translation,assume_aligned_axes)
        for event in recording.events():
            clock.advance(event.arrival_ns);counts[event.stream]+=1
            first=event.arrival_ns if first is None else first;last=event.arrival_ns
            if event.kind=='imu':
                if event.frame!='body_flu':
                    skips['imu_frame_not_body_flu']+=1;continue
                acceleration=horizontal_acceleration_values(event.data['acceleration_mps2'],event.data.get('orientation_xyzw'))
                if acceleration is not None:imu.append((event.stamp_ns,acceleration))
            elif event.kind=='gnss' and event.stream=='gnss':
                if event.frame!='enu':
                    skips['gnss_frame_not_enu']+=1;evaluator.observe(event);continue
                candidates=[item for item in imu if 0<=event.stamp_ns-item[0]<=50_000_000]
                if not candidates:skips['no_past_imu_within_50ms']+=1
                elif last_gnss is not None and event.stamp_ns<=last_gnss:skips['duplicate_or_reversed_gnss_time']+=1
                else:
                    stamp,acceleration=max(candidates,key=lambda item:item[0])
                    row=detector.step(event.stamp_ns,*event.data['position_m'][:2],acceleration)
                    last_gnss=event.stamp_ns;valid+=1;alarm=row['detector_status']=='alarm';alarms+=alarm
                    if alarm and first_alarm is None:first_alarm=event.stamp_ns
                    if attack_start!='unknown' and row['detector_status']!='initializing':
                        active=attack_start is not None and event.stamp_ns>=attack_start
                        labelled_attack+=active;labelled_normal+=not active
                        attack_alarms+=active and alarm;false_alarms+=(not active) and alarm
                    row.update(arrival_ns=event.arrival_ns,imu_stamp_ns=stamp,imu_age_ms=(event.stamp_ns-stamp)/1e6)
                    detection_file.write(json.dumps(row,allow_nan=False)+'\n')
            elif event.kind=='cloud' and verify_clouds:recording.cloud(event)
            # Evaluation is a separate sink and cannot update detector state.
            evaluator.observe(event)
        evaluator.flush(final=True)
    if first is None:raise ValueError('Recording contains no events')
    summary=dict(schema_version=1,source=str(Path(source).resolve()),events=sum(counts.values()),counts=dict(counts),
        replay_rate=rate,duration_s=(last-first)/1e9,processing_wall_s=time.perf_counter()-began,
        evaluation=evaluator.summary(),detector=dict(algorithm='legacy planar IMU/GNSS innovation baseline',
            gps_sigma_m=gps_sigma_m,acceleration_sigma_mps2=acceleration_sigma_mps2,valid_updates=valid,
            alarm_count=int(alarms),first_alarm_stamp_ns=first_alarm,skipped=dict(skips),truth_used=False,
            labels_available=attack_start!='unknown',false_alarm_fraction=false_alarms/labelled_normal if labelled_normal else None,
            attack_alarm_fraction=attack_alarms/labelled_attack if labelled_attack else None),
        cloud_payloads_verified=verify_clouds,
        scope='replay existing sensors and poses; no physics, FAST-LIO execution or flight control')
    (output/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    write_report(output/'report.html',summary,samples)
    return summary
