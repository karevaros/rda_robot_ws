// 실기 진단: 현재 자세를 servo_j 로 100Hz 스트리밍하며 송신 간격·robot_state 전이를 잰다(Simulation 모드·활성화 아니면 중단).
// 빌드: g++ -std=c++17 -O2 servo_hold_test.cpp -I ~/robot_ws/thirdparty/install/include -L ~/robot_ws/thirdparty/install/lib -lrbpodo -lpthread -o servo_hold_test
// 사용: ./servo_hold_test <t2> <초>   (2026-10-29: t2=0.03 → 30초에 2~5회 끊김 / 0.1·0.2 → 0회)
#include <rbpodo/rbpodo.hpp>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <thread>
using namespace rb; using clk = std::chrono::steady_clock;
int main(int argc, char** argv){
  double t2 = argc > 1 ? atof(argv[1]) : 0.03, dur = argc > 2 ? atof(argv[2]) : 30;
  const char* ip = "192.168.35.100";
  podo::CobotData data(ip);
  auto s0 = data.request_data(1.0);
  if(!s0){ puts("no data"); return 1; }
  if(s0->sdata.real_vs_simulation_mode != 1 || s0->sdata.init_state_info != 6){
    printf("ABORT: sim=%d init=%d\n", s0->sdata.real_vs_simulation_mode, s0->sdata.init_state_info); return 1; }
  std::array<double,6> q; for(int i=0;i<6;i++) q[i]=s0->sdata.jnt_ang[i];
  podo::Cobot robot(ip); podo::ResponseCollector rc;
  robot.disable_waiting_ack(rc);
  std::atomic<bool> stop{false}; std::atomic<int> state{1}; std::atomic<int> drops{0};
  auto t0 = clk::now();
  auto ms = [&](clk::time_point t){ return std::chrono::duration<double,std::milli>(t-t0).count(); };
  std::atomic<double> last_send_ms{0};
  std::thread mon([&]{ podo::CobotData d2(ip); int prev=1;
    while(!stop){ auto s=d2.request_data(0.5); if(s){ int r=s->sdata.robot_state; state=r;
      if(r!=prev){ printf("[%8.1fms] robot_state %d -> %d (마지막 송신 %.1fms 전)\n", ms(clk::now()), prev, r, ms(clk::now())-last_send_ms.load());
        if(r==1) drops++; prev=r; } }
      std::this_thread::sleep_for(std::chrono::milliseconds(5)); } });
  auto next = clk::now(); double maxgap=0, prevt=-1; int n=0, over30=0;
  while(ms(clk::now()) < dur*1000){
    std::this_thread::sleep_until(next); next += std::chrono::milliseconds(10);
    double t = ms(clk::now());
    if(prevt>=0){ double g=t-prevt; if(g>maxgap) maxgap=g; if(g>30) over30++; }
    prevt=t; last_send_ms=t;
    robot.move_servo_j(rc, q, 0.01, t2, 0.5, 0.5); n++;
    if(n%100==0) rc.clear();
  }
  stop=true; mon.join();
  printf("t2=%.3f  송신 %d회  최대 송신간격 %.1fms  30ms 초과 %d회  상태 1 로 복귀 %d회  최종 state=%d\n", t2, n, maxgap, over30, drops.load(), state.load());
}
