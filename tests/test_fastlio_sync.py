"""Compile the actual sync_packages function against small message/container stubs.

This checks timestamp boundaries and buffering without pretending to run ROS or LIO.
Run: python tests/test_fastlio_sync.py (needs a C++14 compiler on PATH).
"""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile

workspace = Path(__file__).resolve().parents[1]
source = (workspace / 'src/27_rc_localization/src/FAST_LIO/src/laserMapping.cpp').read_text(encoding='utf-8')
function = source[source.index('bool sync_packages('):source.index('\nint process_increments')]
compiler = shutil.which('g++') or shutil.which('clang++')
assert compiler, 'A C++14 compiler is required'
prefix = r'''
#include <cassert>
#include <deque>
#include <iostream>
#include <memory>
#include <vector>
struct Point { double curvature = 0.0; };
struct Cloud { std::vector<Point> points; };
struct Imu { struct { double stamp; } header; };
struct MeasureGroup {
    std::shared_ptr<Cloud> lidar;
    double lidar_beg_time = 0, lidar_end_time = 0;
    std::deque<std::shared_ptr<Imu>> imu;
};
double get_time_sec(double stamp) { return stamp; }
std::deque<std::shared_ptr<Cloud>> lidar_buffer;
std::deque<double> time_buffer;
std::deque<std::shared_ptr<Imu>> imu_buffer;
bool lidar_pushed = false, instantaneous_scan = true;
double lidar_end_time = 0, lidar_mean_scantime = 0;
double last_timestamp_imu = 0;
int scan_num = 0;
void reset() {
    lidar_buffer.clear(); time_buffer.clear(); imu_buffer.clear();
    lidar_pushed = false; lidar_mean_scantime = 0; scan_num = 0;
}
void cloud(double stamp) {
    auto p = std::make_shared<Cloud>(); p->points.resize(6);
    lidar_buffer.push_back(p); time_buffer.push_back(stamp);
}
void imu(double stamp) {
    auto p = std::make_shared<Imu>(); p->header.stamp = stamp;
    imu_buffer.push_back(p); last_timestamp_imu = stamp;
}
'''
tests = r'''
int main() {
    MeasureGroup measure;
    assert(!sync_packages(measure));
    cloud(1.0); imu(0.9);
    assert(!sync_packages(measure));  // Wait until IMU covers the frame.
    assert(lidar_buffer.size() == 1 && lidar_pushed);
    imu(1.0); imu(1.01);
    assert(sync_packages(measure));
    assert(measure.lidar_beg_time == 1.0 && measure.lidar_end_time == 1.0);
    assert(measure.imu.size() == 2);  // Include the exact frame boundary.
    assert(imu_buffer.size() == 1 && imu_buffer.front()->header.stamp == 1.01);
    assert(lidar_buffer.empty() && time_buffer.empty() && !lidar_pushed);

    reset(); cloud(2.0); imu(2.01);
    assert(!sync_packages(measure));  // No IMU at/before frame: skip, don't reuse.
    assert(measure.imu.empty() && lidar_buffer.empty() && imu_buffer.size() == 1);
    cloud(2.1); imu(2.1);
    assert(sync_packages(measure) && measure.imu.size() == 2);

    reset(); instantaneous_scan = false; cloud(3.0);
    lidar_buffer.front()->points.back().curvature = 100.0;
    imu(3.05); imu(3.1); imu(3.11);
    assert(sync_packages(measure));  // Real scan's 100 ms offsets remain supported.
    assert(measure.lidar_end_time == 3.1 && measure.imu.size() == 2);
    assert(imu_buffer.size() == 1);
    std::cout << "PASS: actual FAST-LIO sync function; instantaneous/real timing, boundary, buffering\n";
}
'''
environment = os.environ.copy()
environment['PATH'] = str(Path(compiler).parent) + os.pathsep + environment['PATH']
with tempfile.TemporaryDirectory(prefix='rc_lio_sync_') as temporary:
    folder = Path(temporary)
    cpp = folder / 'sync.cpp'
    executable = folder / ('sync.exe' if os.name == 'nt' else 'sync')
    cpp.write_text(prefix + function + tests, encoding='utf-8')
    subprocess.run([compiler, '-std=c++14', str(cpp), '-o', str(executable)],
                   check=True, env=environment)
    subprocess.run([str(executable)], check=True, env=environment)
