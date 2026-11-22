#!/usr/bin/env python3
"""Build a local FAST-LIO copy with read-only matching diagnostics and sampling controls."""
import argparse
import hashlib
import json
from pathlib import Path
import shlex
import subprocess


GLOBALS = r'''
// Local experiment instrumentation; no truth or GNSS is used by the estimator.
Eigen::Matrix3d msim_last_geometry = Eigen::Matrix3d::Zero();
bool msim_plane_spread_ok(const PointVector &near) {
    const char *setting=std::getenv("MSIM_PLANE_MIN_SPREAD");
    if (!setting || std::atof(setting)<=0) return true;
    Eigen::Vector3d center=Eigen::Vector3d::Zero();
    for(const auto &p:near) center+=Eigen::Vector3d(p.x,p.y,p.z);
    center/=near.size();Eigen::Matrix3d covariance=Eigen::Matrix3d::Zero();
    for(const auto &p:near) {
        Eigen::Vector3d delta=Eigen::Vector3d(p.x,p.y,p.z)-center;
        covariance+=delta*delta.transpose();
    }
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> e(covariance);
    return e.eigenvalues()[1]/std::max(1e-15,e.eigenvalues()[2])>=std::atof(setting);
}
void msim_record_geometry(const Eigen::MatrixXd &h) {
    if (h.cols() >= 3) msim_last_geometry = h.leftCols(3).transpose() * h.leftCols(3);
}
void msim_write_diagnostics(double solve_seconds) {
    const char *path = std::getenv("MSIM_LIO_DIAGNOSTICS");
    if (!path) return;
    static std::ofstream file(path);
    static bool first = true;
    if (first) {
        file << "stamp_s,prevoxel_points,voxel_points,effective_points,map_points,"
             << "normal_E,normal_N,normal_U,lambda_min,lambda_mid,lambda_max,"
             << "weak_E,weak_N,weak_U,info_E,info_N,info_U,mean_abs_residual,"
             << "mean_abs_residual_E,mean_abs_residual_N,mean_abs_residual_U,"
             << "gyro_bias_x,gyro_bias_y,gyro_bias_z,accel_bias_x,accel_bias_y,accel_bias_z,"
             << "pos_var_E,pos_var_N,pos_var_U,solve_ms,"
             << "line_neighborhood_fraction,line_neighborhood_U_fraction,"
             << "middle_largest_ratio_median,ls_pca_normal_angle_median_deg\n";
        first = false;
    }
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen(msim_last_geometry);
    int count[3] = {0,0,0}; double sum[3] = {0,0,0};
    for (int i=0; i<effct_feat_num; ++i) {
        const auto &n = corr_normvect->points[i];
        Eigen::Vector3d normal(n.x,n.y,n.z); Eigen::Index axis;
        normal.cwiseAbs().maxCoeff(&axis);
        count[axis]++; sum[axis] += std::abs(n.intensity);
    }
    const auto state = kf.get_x(); const auto p = kf.get_P();
    file << std::setprecision(12) << lidar_end_time << ',' << feats_undistort->size()
         << ',' << feats_down_size << ',' << effct_feat_num << ',' << ikdtree.validnum();
    for (int v:count) file << ',' << v;
    for (int i=0;i<3;++i) file << ',' << eigen.eigenvalues()[i];
    for (int i=0;i<3;++i) file << ',' << eigen.eigenvectors()(i,0);
    for (int i=0;i<3;++i) file << ',' << msim_last_geometry(i,i);
    file << ',' << res_mean_last;
    for (int i=0;i<3;++i) file << ',' << (count[i] ? sum[i]/count[i] : 0);
    for (int i=0;i<3;++i) file << ',' << state.bg[i];
    for (int i=0;i<3;++i) file << ',' << state.ba[i];
    for (int i=0;i<3;++i) file << ',' << p(i,i);
    file << ',' << solve_seconds*1000;
    int line_count=0, line_u=0, total_u=0;
    std::vector<double> ratios, angles;
    for (int i=0; i<feats_down_size; ++i) {
        if (!point_selected_surf[i] || Nearest_Points[i].size()<3) continue;
        const auto &near=Nearest_Points[i];
        Eigen::Vector3d center=Eigen::Vector3d::Zero();
        for (const auto &point:near) center+=Eigen::Vector3d(point.x,point.y,point.z);
        center/=near.size(); Eigen::Matrix3d covariance=Eigen::Matrix3d::Zero();
        for (const auto &point:near) {
            Eigen::Vector3d delta=Eigen::Vector3d(point.x,point.y,point.z)-center;
            covariance+=delta*delta.transpose();
        }
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> e(covariance);
        double ratio=e.eigenvalues()[1]/std::max(1e-15,e.eigenvalues()[2]);
        ratios.push_back(ratio);
        const auto &n=normvec->points[i]; Eigen::Vector3d normal(n.x,n.y,n.z);
        double angle=acos(std::min(1.0,std::abs(normal.dot(e.eigenvectors().col(0)))))*180/M_PI;
        angles.push_back(angle);
        bool vertical=std::abs(n.z)>=std::max(std::abs(n.x),std::abs(n.y));
        if(vertical) ++total_u;
        if(ratio<0.01) {++line_count;if(vertical) ++line_u;}
    }
    std::sort(ratios.begin(),ratios.end());std::sort(angles.begin(),angles.end());
    file << ',' << (ratios.empty()?0:double(line_count)/ratios.size())
         << ',' << (total_u?double(line_u)/total_u:0)
         << ',' << (ratios.empty()?0:ratios[ratios.size()/2])
         << ',' << (angles.empty()?0:angles[angles.size()/2]) << '\n';
    file.flush();
}
'''

SAMPLER = r'''
#include <cstdlib>
#include <cstdint>
namespace {
uint64_t msim_scan_index = 0;
bool msim_keep_point(int index, int stride) {
    const char *offset_env = std::getenv("MSIM_SAMPLE_OFFSET");
    int offset = offset_env ? std::atoi(offset_env) : 0;
    const char *mode = std::getenv("MSIM_SAMPLE_MODE");
    if (mode && std::string(mode) == "hash") {
        const char *seed_env = std::getenv("MSIM_SAMPLE_SEED");
        uint64_t seed = seed_env ? std::strtoull(seed_env,nullptr,10) : 17;
        uint64_t x = (msim_scan_index*1000003ULL + index) ^ (seed*0x9e3779b97f4a7c15ULL);
        x += 0x9e3779b97f4a7c15ULL;
        x = (x ^ (x >> 30)) * 0xbf58476d1ce4e5b9ULL;
        x = (x ^ (x >> 27)) * 0x94d049bb133111ebULL;
        x ^= x >> 31;
        return x % stride == 0;
    }
    return index % stride == offset;
}
}
'''


def replace_once(text, old, new):
    if text.count(old) != 1:
        raise ValueError(f'expected unique instrumentation anchor: {old[:100]}')
    return text.replace(old, new)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--fast-glio', type=Path, default=Path('/home/wzb/FAST_GLIO'))
    args = parser.parse_args()
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=True)
    fast = args.fast_glio.resolve()
    source = fast / 'FAST_LIO/src/laserMapping.cpp'
    mapping = source.read_text()
    # All globals needed by the logger have been declared before this function.
    mapping = replace_once(mapping, 'void h_share_model(state_ikfom &s,',
                           GLOBALS+'\nvoid h_share_model(state_ikfom &s,')
    mapping = replace_once(mapping,
        '    update_lidar_directional_observation(ekfom_data.h_x.leftCols(3));',
        '    msim_record_geometry(ekfom_data.h_x);\n'
        '    update_lidar_directional_observation(ekfom_data.h_x.leftCols(3));')
    mapping = replace_once(mapping, 'if (esti_plane(pabcd, points_near, 0.1f))',
                           'if (msim_plane_spread_ok(points_near) && esti_plane(pabcd, points_near, 0.1f))')
    mapping = replace_once(mapping, '                apply_dynamic_covariance(lidar_end_time);',
        '                msim_write_diagnostics(omp_get_wtime()-t_update_start);\n'
        '                apply_dynamic_covariance(lidar_end_time);')
    (out / 'laserMapping.cpp').write_text(mapping)
    preprocess_source = fast / 'FAST_LIO/src/preprocess.cpp'
    preprocess = preprocess_source.read_text()
    preprocess = replace_once(preprocess, '#include "preprocess.h"',
                               '#include "preprocess.h"\n'+SAMPLER)
    preprocess = replace_once(preprocess,
        'void Preprocess::velodyne_handler(const sensor_msgs::PointCloud2::ConstPtr &msg)\n{',
        'void Preprocess::velodyne_handler(const sensor_msgs::PointCloud2::ConstPtr &msg)\n{\n'
        '    ++msim_scan_index;')
    preprocess = replace_once(preprocess, 'if (i % point_filter_num == 0)',
                               'if (msim_keep_point(i, point_filter_num))')
    (out / 'preprocess.cpp').write_text(preprocess)
    build = fast / 'build/FAST_LIO/CMakeFiles/fastlio_mapping.dir'
    values = dict(line.split(' = ', 1) for line in (build / 'flags.make').read_text().splitlines()
                  if line.startswith('CXX_'))
    flags = [str(fast / '.pixi/envs/default/bin/c++')]
    for key in ('CXX_DEFINES', 'CXX_INCLUDES', 'CXX_FLAGS'):
        flags += shlex.split(values[key])
    runtime = out / 'runtime'
    (runtime / 'Log').mkdir(parents=True, exist_ok=True)
    (runtime / 'PCD').mkdir(exist_ok=True)
    flags = [f'-DROOT_DIR="{runtime}/"' if value.startswith('-DROOT_DIR=') else value
             for value in flags]
    flags += ['-I'+str(fast / 'FAST_LIO/src')]
    for filename in ('laserMapping', 'preprocess'):
        print('Compiling '+filename, flush=True)
        subprocess.run(flags+['-c', str(out / f'{filename}.cpp'), '-o', str(out / f'{filename}.o')], check=True)
    link = shlex.split((build / 'link.txt').read_text())
    replacements = {
        'CMakeFiles/fastlio_mapping.dir/src/laserMapping.cpp.o': str(out / 'laserMapping.o'),
        'CMakeFiles/fastlio_mapping.dir/src/preprocess.cpp.o': str(out / 'preprocess.o'),
        str(fast / 'devel/lib/fast_lio/fastlio_mapping'): str(out / 'fastlio_mapping_diagnostic'),
    }
    link = [replacements.get(arg, arg) for arg in link if not arg.startswith('-Wl,--dependency-file=')]
    subprocess.run(link, cwd=fast / 'build/FAST_LIO', check=True)
    manifest = {str(p): hashlib.sha256(p.read_bytes()).hexdigest()
                for p in (source, preprocess_source, out/'laserMapping.cpp', out/'preprocess.cpp')}
    (out / 'source-hashes.json').write_text(json.dumps(manifest, indent=2)+'\n')
    print(out / 'fastlio_mapping_diagnostic')


if __name__ == '__main__':
    main()
