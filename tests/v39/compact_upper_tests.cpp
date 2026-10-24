#define main regression_main
#include "host_tests.cpp"
#undef main

static bool upperPixel(bool inverted,unsigned r,unsigned c) {
  const unsigned m=inverted?1:0;
  unsigned pr=inverted?7-r:r,pc=inverted?7-c:c;
  // Photo calibration: the near module is physically reversed.
  if(m==0){pr=7-pr;pc=7-pc;}
  return (maxRegs[m][pr+1] & (1U<<(7-pc)))!=0;
}
static void supportedPile(bool inverted) {
  boot(inverted?-1.0f:1.0f);
  require(upperPixel(inverted,7,7),"upper sand floats: the lower corner is empty");
  for(unsigned r=0;r<8;++r) for(unsigned c=0;c<8;++c)
    require(grains[inverted?1:0][r][c] == (r+c>=7),
            "initial upper pile is detached from the lower funnel walls");
  running(); advance(800-elapsedMs);
  require(upperPixel(inverted,7,7),"starting creates a hollow space beneath the pile");
}
static void visibleAvalanche(bool inverted) {
  boot(inverted?-1.0f:1.0f);running();advance(1730-elapsedMs);
  const unsigned top=inverted?1:0;
  uint8_t prior[8]; std::memcpy(prior,frame[top],8);
  unsigned movementFrames=0;
  for(unsigned i=0;i<12;++i) {
    advance(60); unsigned changes=0;
    for(unsigned r=0;r<8;++r)for(unsigned c=0;c<8;++c) {
      const unsigned physicalR=top==0?7-r:r,physicalC=top==0?7-c:c;
      const unsigned lr=inverted?7-physicalR:physicalR,lc=inverted?7-physicalC:physicalC;
      if(lr==0 && lc==0)continue; // Ready/running status heartbeat is not sand.
      changes+=((prior[r]^frame[top][r])>>(7-c))&1;
    }
    require(changes<=2,"upper pile collapses in one instant instead of one grain per frame");
    if(changes==2)++movementFrames;
    std::memcpy(prior,frame[top],8);
    require(grainCount(top)==35 && grainCount(1-top)==1,"avalanche changed transfer schedule");
    require(upperPixel(inverted,7,7),"upper avalanche leaves the complete lower tip empty");
  }
  require(movementFrames>=3,"no visible gradual upper-pile refill after a grain leaves");
}
static void upperTiltResponse() {
  boot();running();acceleration(.6f,.8f,0);advance(1000);
  unsigned lowSide=0,opposite=0;
  for(unsigned r=0;r<8;++r)for(unsigned c=0;c<8;++c) if(grains[0][r][c]) {
    lowSide+=r;opposite+=c;
  }
  require(lowSide>opposite+36,"upper sand responds too slowly to a substantial tilt");
  acceleration(.6f,-.8f,0);advance(1000);
  lowSide=opposite=0;
  for(unsigned r=0;r<8;++r)for(unsigned c=0;c<8;++c)if(grains[0][r][c]){lowSide+=c;opposite+=r;}
  require(lowSide>opposite+30,"upper-pile throttling prevents opposite tilt response");
}
int main(int argc,char**argv) {
  const std::map<std::string,std::function<void()>> cases={
    {"supported_upright",[]{supportedPile(false);}},
    {"supported_inverted",[]{supportedPile(true);}},
    {"avalanche_upright",[]{visibleAvalanche(false);}},
    {"avalanche_inverted",[]{visibleAvalanche(true);}},
    {"upper_tilt_response",upperTiltResponse}
  };
  if(argc!=2){for(const auto&c:cases)std::cout<<c.first<<'\n';return 2;}
  try{auto i=cases.find(argv[1]);require(i!=cases.end(),"unknown case");i->second();std::cout<<"PASS "<<argv[1]<<'\n';return 0;}
  catch(const std::exception&e){std::cerr<<"FAIL "<<argv[1]<<": "<<e.what()<<" elapsed="<<elapsedMs<<'\n';return 1;}
}
