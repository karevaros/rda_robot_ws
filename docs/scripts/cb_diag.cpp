// 제어박스 상태 진단(데이터 채널 5001 · 읽기 전용): 전원·E-stop 비트, Simulation/Real, 활성화 단계·오류, 관절 각도·상태 비트.
// 빌드: g++ -std=c++17 cb_diag.cpp -I ~/robot_ws/thirdparty/install/include -L ~/robot_ws/thirdparty/install/lib -lrbpodo -o ~/robot_ws/build/cb_diag
// ⚠ IP 가 192.168.35.100 으로 박혀 있다(rb850_old). 다른 로봇이면 고칠 것.
#include <rbpodo/rbpodo.hpp>
#include <iostream>
#include <bitset>
int main(){
  rb::podo::CobotData data("192.168.35.100");
  auto r = data.request_data(1.0);
  if(!r){ std::cout<<"no data\n"; return 1; }
  auto& s = r->sdata;
  int c = s.information_chunk_1;
  std::cout<<"chunk1 bits: 48Vin="<<((c>>0)&1)<<" 48Vout="<<((c>>1)&1)<<" 24Vin="<<((c>>2)&1)
           <<" Estop1="<<((c>>3)&1)<<" UserSW="<<((c>>4)&1)<<" Estop2="<<((c>>5)&1)
           <<" ArmPower="<<((c>>6)&1)<<" DTbtn="<<((c>>7)&1)<<"\n";
  std::cout<<"sim_mode="<<s.real_vs_simulation_mode<<" init_state="<<s.init_state_info<<" init_error="<<s.init_error
           <<" robot_state="<<s.robot_state<<" sos="<<s.op_stat_sos_flag<<" ems="<<s.op_stat_ems_flag<<" selfcol="<<s.op_stat_self_collision<<"\n";
  for(int i=0;i<6;i++) std::cout<<"J"<<i<<" ang="<<s.jnt_ang[i]<<" info="<<std::bitset<16>(s.jnt_info[i])<<"\n";
}
