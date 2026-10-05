"""Compile the edited C++ functions with small stubs; no ROS/Gazebo runtime."""
import importlib.util
import math
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
SIM = ROOT / 'src/27_rc_simulation'
LIO = ROOT / 'src/27_rc_localization/src/FAST_LIO'


def function(path, signature):
    source = path.read_text(encoding='utf-8')
    start = source.index(signature)
    brace = source.index('{', start)
    depth = 1
    end = brace + 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[start:end]


def compile_run(source):
    compiler = shutil.which('g++') or shutil.which('clang++')
    if not compiler:
        raise RuntimeError('g++ or clang++ is required')
    env = os.environ.copy()
    env['PATH'] = str(Path(compiler).parent) + os.pathsep + env['PATH']
    with tempfile.TemporaryDirectory(prefix='rc_motion_test_') as temporary:
        folder = Path(temporary)
        cpp = folder / 'test.cpp'
        executable = folder / ('test.exe' if os.name == 'nt' else 'test')
        cpp.write_text(source, encoding='utf-8')
        subprocess.run([compiler, '-std=c++17', '-pthread', str(cpp), '-o', str(executable)],
                       check=True, env=env)
        subprocess.run([str(executable)], check=True, env=env, cwd=folder)


class MotionTests(unittest.TestCase):
    def test_actual_ray_update_block_holds_one_pose_for_all_rays(self):
        code = function(SIM / 'livox_laser_simulation_RO2/src/livox_points_plugin.cpp',
                        'void LivoxPointsPlugin::OnNewLaserScans()')
        start = code.index('        {', code.index('builtin_interfaces::msg::Time stamp;'))
        end = code.index('// 创建自定义消息', start)
        block = code[start:end]
        compile_run(r'''
#include <cassert>
#include <atomic>
#include <chrono>
#include <mutex>
#include <thread>
#include <utility>
#include <vector>
namespace boost {struct recursive_mutex:std::recursive_mutex {
 using scoped_lock=std::lock_guard<recursive_mutex>;};}
struct Time {int tick;};
namespace builtin_interfaces {namespace msg {struct Time {int tick=0;};}}
namespace gazebo_ros {template<class T>T Convert(Time t){T result;result.tick=t.tick;return result;}}
struct Physics {boost::recursive_mutex mutex;
 auto GetPhysicsUpdateMutex(){return &mutex;}};
struct World {struct Physics physics;int pose=0;Time SimTime(){return {pose};}
 auto Physics(){return &physics;}};
struct Shape {World* world;std::vector<int> observed;
 void Update(){for(int i=0;i<10;++i){observed.push_back(world->pose);
 std::this_thread::sleep_for(std::chrono::milliseconds(1));}
 boost::recursive_mutex::scoped_lock nested(world->physics.mutex);}};
struct Scan {};
struct LaserMsg {Time time;Time* mutable_time(){return &time;}};
namespace msgs {void Set(Time* out,Time t){*out=t;}}
struct AviaRotateInfo {};
int main(){
 World w;auto world=&w;Shape shape{&w,{}};auto rayShape=&shape;
 LaserMsg laserMsg;Scan s;auto scan=&s;
 std::vector<std::pair<int,AviaRotateInfo>> points_pair;
 builtin_interfaces::msg::Time stamp;
 auto InitializeRays=[](auto&,auto*){};auto InitializeScan=[](auto*){};
 std::atomic<bool> stop{false},ready{false};
 std::thread physics([&]{ready=true;while(!stop){
 {boost::recursive_mutex::scoped_lock lock(w.physics.mutex);++w.pose;}
 std::this_thread::sleep_for(std::chrono::milliseconds(1));}});
 while(!ready)std::this_thread::yield();
''' + block + r'''
 stop=true;physics.join();
 for(int pose:shape.observed)assert(pose==stamp.tick);
 assert(laserMsg.time.tick==stamp.tick);
}
''')

    def test_actual_controller_preserves_passive_wheel_spin_and_gravity(self):
        code = function(SIM / 'robocon2027_gazebo/src/rc_planar_move.cpp',
                        'void RcPlanarMovePrivate::OnUpdate(')
        compile_run(r'''
#include <cassert>
#include <cmath>
#include <mutex>
namespace ignition { namespace math {
struct Vector3d { double x=0,y=0,z=0; Vector3d()=default;
 Vector3d(double a,double b,double c):x(a),y(b),z(c){}
 double X()const{return x;} double Y()const{return y;} double Z()const{return z;} };
struct Rotation { double yaw=0; double Yaw()const{return yaw;} };
struct Pose3d { Rotation rotation; Rotation Rot()const{return rotation;} };
}}
namespace gazebo { namespace common {
struct Time {double t=0; Time(double a=0):t(a){} double Double()const{return t;}
 Time operator-(const Time& b)const{return Time(t-b.t);} };
struct UpdateInfo {Time simTime;};
}}
namespace geometry_msgs { namespace msg {
struct Twist {ignition::math::Vector3d linear,angular;};
}}
struct Link {
 ignition::math::Vector3d linear{0,0,-0.03}, angular{0.01,-0.02,0.3};
 auto WorldLinearVel(){return linear;} auto WorldAngularVel(){return angular;}
 void SetLinearVel(ignition::math::Vector3d v){linear=v;}
 void SetAngularVel(ignition::math::Vector3d v){angular=v;}
};
struct Model {ignition::math::Pose3d pose;
 auto WorldPose(){return pose;}
};
struct Publisher {int count=0; void publish(int){++count;}};
struct RcPlanarMovePrivate {
 std::mutex lock_; Model *model_; Link *chassis_link_;
 double update_period_=0,cmd_timeout_=0.5,publish_period_=0.1;
 gazebo::common::Time last_update_time_,last_cmd_time_,last_publish_time_;
 geometry_msgs::msg::Twist target_cmd_vel_;
 bool publish_odom_=true,publish_odom_tf_=false;
 Publisher *odometry_pub_; int odom_=0;
 void UpdateOdometry(gazebo::common::Time){++odom_;}
 void PublishOdometryTf(gazebo::common::Time){}
 void OnUpdate(const gazebo::common::UpdateInfo &);
};
''' + code + r'''
int main(){
 Model model; Link body, wheel; Publisher publisher; RcPlanarMovePrivate c;
 model.pose.rotation.yaw=1.5707963267948966;
 c.model_=&model;c.chassis_link_=&body;c.odometry_pub_=&publisher;
 c.target_cmd_vel_.linear.x=0.5; wheel.angular.y=8.0;
 c.OnUpdate({0.001});
 assert(std::abs(body.linear.x)<1e-7 && std::abs(body.linear.y-0.5)<1e-7);
 assert(body.linear.z==-0.03 && body.angular.x==0.01 && body.angular.y==-0.02);
 assert(body.angular.z==0 && wheel.angular.y==8.0);
 body.angular.z=0.1; c.OnUpdate({0.002}); assert(body.angular.z==0);
 c.target_cmd_vel_.angular.z=0.2;c.OnUpdate({0.003});assert(body.angular.z==0.2);
 c.OnUpdate({0.1});assert(publisher.count==1);
 c.OnUpdate({0.501});assert(body.linear.x==0 && body.linear.y==0 && body.angular.z==0);
 assert(body.linear.z==-0.03 && wheel.angular.y==8.0);
}
''')

    def test_actual_imu_window_accepts_contact_jitter_and_rejects_motion(self):
        code = function(LIO / 'src/IMU_Processing.hpp', 'bool ImuProcess::SimulationImuReady(')
        process = function(LIO / 'src/IMU_Processing.hpp', 'void ImuProcess::Process(')
        compile_run(r'''
#include <algorithm>
#include <cassert>
#include <cmath>
#include <deque>
#include <fstream>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <vector>
constexpr double G_m_s2=9.81;
using std::ios;
constexpr int MAX_INI_COUNT=10;
double omp_get_wtime(){return 0;}
#define DEBUG_FILE_DIR(name) std::string(name)
struct V3D {double x=0,y=0,z=0; V3D()=default;V3D(double a,double b,double c):x(a),y(b),z(c){}
 double squaredNorm()const{return x*x+y*y+z*z;}
 double norm()const{return std::sqrt(squaredNorm());}
 V3D operator/(double n)const{return {x/n,y/n,z/n};}
 V3D operator-(const V3D&b)const{return {x-b.x,y-b.y,z-b.z};}
 V3D&operator+=(const V3D&b){x+=b.x;y+=b.y;z+=b.z;return *this;}
 V3D&operator*=(double n){x*=n;y*=n;z*=n;return *this;}
 std::string transpose()const{return "stub vector";}
};
const V3D Zero3d{0,0,0};
namespace rclcpp {struct Time {double t;Time(double v):t(v){}double seconds()const{return t;}};}
struct Imu {struct {double stamp;} header;struct Vec {double x=0,y=0,z=0;};
 Vec linear_acceleration,angular_velocity;};
struct Cloud {void clear(){}};
struct PointCloudXYZI {using Ptr=std::shared_ptr<Cloud>;};
struct MeasureGroup {std::deque<std::shared_ptr<Imu>> imu;std::shared_ptr<Cloud>lidar=std::make_shared<Cloud>();};
struct state_ikfom{};
struct input_ikfom{};
namespace esekfom {template<class T,int N,class U=input_ikfom>struct esekf {T get_x(){return {};}};}
struct ImuProcess {double initialization_stationary_seconds=1;
 std::deque<std::shared_ptr<Imu>> initialization_imu_;
 double initialization_max_gyro=0,initialization_max_acc_error=0,initialization_stable_seconds=0;
 double initialization_mean_gyro=0,initialization_mean_acc_error=0,initialization_gyro_rms=0,initialization_acc_rms=0;
 double initialization_mean_gyro_limit=0.02,initialization_gyro_rms_limit=0.08;
 double initialization_mean_acc_error_limit=0.5,initialization_acc_rms_limit=2.0;
 const char* initialization_status="collecting IMU samples";
 bool instantaneous_scan=true,imu_need_init_=true,b_first_frame_=true;
 int init_iter_num=1;size_t initialized_samples=0;
 V3D mean_gyr,mean_acc,cov_acc,cov_acc_scale,cov_gyr,cov_gyr_scale;
 std::shared_ptr<Imu>last_imu_;std::ofstream fout_imu;
 bool SimulationImuReady(const MeasureGroup&);
 void Process(const MeasureGroup&,esekfom::esekf<state_ikfom,12>&,PointCloudXYZI::Ptr);
 void IMU_init(const MeasureGroup&m,esekfom::esekf<state_ikfom,12>&,int&count){
   // Model the actual IMU_init Reset: it clears the window being accumulated.
   initialization_imu_.clear();initialized_samples=m.imu.size();
   mean_acc=Zero3d;mean_gyr=Zero3d;
   for(const auto&p:m.imu){const auto&a=p->linear_acceleration;const auto&w=p->angular_velocity;
     mean_acc+=V3D(a.x,a.y,a.z)/m.imu.size();mean_gyr+=V3D(w.x,w.y,w.z)/m.imu.size();}
   count=static_cast<int>(m.imu.size())+1;
 }
 void UndistortPcl(const MeasureGroup&,esekfom::esekf<state_ikfom,12>&,Cloud&){}
};
MeasureGroup batch(double start,double acc=9.81,double gyro=0){
 MeasureGroup m;for(int i=0;i<10;++i){auto p=std::make_shared<Imu>();
 p->header.stamp=start+i*0.01;p->linear_acceleration.z=acc;
 p->angular_velocity.x=gyro;m.imu.push_back(p);}return m;}
''' + code + process + r'''
bool window(double acc_offset=0,double rotation=0,double gyro_jitter=0.05,double acc_jitter=1.8){
 ImuProcess p;bool ready=false;
 for(int n=0;n<12;++n){auto m=batch(n*0.1);
  for(int i=0;i<10;++i){auto&r=m.imu[i];const double sign=((n*10+i)%2)?-1:1;
   r->linear_acceleration.z=9.81+acc_offset+sign*acc_jitter;
   r->angular_velocity.x=rotation+sign*gyro_jitter;}
  ready=p.SimulationImuReady(m);
 }
 return ready;
}
int main(){
 ImuProcess p;assert(!p.SimulationImuReady(batch(0,0)));
 for(int n=1;n<=12;++n)p.SimulationImuReady(batch(n*0.1));
 assert(p.SimulationImuReady(batch(1.3)));
 assert(!p.SimulationImuReady(batch(0.1)));
 assert(!p.SimulationImuReady(batch(0.2,std::numeric_limits<double>::quiet_NaN())));
 assert(!p.SimulationImuReady(batch(0.3,9.81,0.3)));
 assert(!p.SimulationImuReady(batch(0.4,20.0)));
 assert(!p.SimulationImuReady(batch(1.0))); // Gap cannot complete the window.
 assert(window()); // Similar amplitude to the measured Gazebo contact jitter.
 assert(!window(0,0.04)); // Sustained slow rotation is not zero-mean vibration.
 assert(!window(1.0,0)); // Sustained acceleration.
 assert(!window(0,0,0.18)); // Excessive oscillation with zero mean gyro.
 assert(!window(0,0,0.05,3.5)); // Excessive acceleration vibration.
 ImuProcess init;esekfom::esekf<state_ikfom,12>kf;auto cloud=std::make_shared<Cloud>();
 for(int n=0;n<12 && init.imu_need_init_;++n){auto m=batch(n*0.1);
   for(int i=0;i<10;++i){double sign=((n*10+i)%2)?-1:1;
     m.imu[i]->angular_velocity.x=sign*0.05;
     m.imu[i]->linear_acceleration.z=9.81+sign*1.8;}
   init.Process(m,kf,cloud);
 }
 assert(!init.imu_need_init_ && init.initialized_samples>=100);
 assert(init.mean_gyr.norm()<0.001 && std::abs(init.mean_acc.norm()-9.81)<0.03);
 ImuProcess real;real.instantaneous_scan=false;
 real.Process(batch(0),kf,cloud);assert(!real.imu_need_init_ && real.initialized_samples==10);
}
''')

    def test_diagnostic_removes_constant_frame_rotation_but_detects_drift(self):
        spec = importlib.util.spec_from_file_location('motion', ROOT / 'tools/diagnose_motion.py')
        motion = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(motion)
        ground = [(0, math.radians(170)), (.1, math.radians(179)), (.2, math.radians(-172))]
        estimated = [(0, 0), (.1, math.radians(9)), (.2, math.radians(18))]
        self.assertAlmostEqual(motion.compare_yaw(ground, estimated)['max_relative_yaw_error_deg'], 0)
        estimated[-1] = (.2, math.radians(23))
        self.assertAlmostEqual(motion.compare_yaw(ground, estimated)['final_relative_yaw_error_deg'], 5)
        self.assertIsNone(motion.compare_yaw([], estimated))

    def test_imu_summary_matches_stationary_gate_and_flags_nonfinite_samples(self):
        spec = importlib.util.spec_from_file_location('motion', ROOT / 'tools/diagnose_motion.py')
        motion = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(motion)
        summary = motion.summarize_imu([
            [0, 0, 9.81, 0, 0], [.01, 0, 9.81, -.0397, 0],
            [.02, 0, 0, 0, 0], [.03, math.nan, 9.81, 0, 0]])
        self.assertEqual(summary['finite_samples'], 3)
        self.assertEqual(summary['nonfinite_samples'], 1)
        self.assertAlmostEqual(summary['max_gyro_norm_rad_s'], .0397)
        self.assertAlmostEqual(summary['gyro_above_0_02_percent'], 100/3)
        self.assertAlmostEqual(summary['acc_error_above_0_5_percent'], 100/3)
        self.assertEqual(summary['longest_stationary_sim_seconds'], 0)
        self.assertIsNone(motion.summarize_imu([]))


if __name__ == '__main__':
    unittest.main()
