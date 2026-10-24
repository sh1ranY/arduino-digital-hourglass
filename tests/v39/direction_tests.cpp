#define main regression_main
#include "host_tests.cpp"
#undef main

// Independent reference from IMG_6793, not the candidate calibration constants:
// M0 raw00=B, raw77=T, raw76=T toward R; M1 raw00=T, raw77=B, raw76=B toward L.
static void photoRaw(unsigned m,unsigned r,unsigned c,bool inverted,unsigned &pr,unsigned &pc) {
  const bool reverse=(m==0)!=inverted;
  pr=reverse?7-r:r; pc=reverse?7-c:c;
}
static void allPixels(bool inverted) {
  orientationHigh=!inverted;
  for(unsigned m=0;m<2;++m)for(unsigned r=0;r<8;++r)for(unsigned c=0;c<8;++c){
    clearFrame();putPixel(m,r,c,true);flushFrame();unsigned pr,pc;
    photoRaw(m,r,c,inverted,pr,pc);
    require(pixels(m)==1 && pixels(1-m)==0,"mapping duplicated or leaked a pixel");
    require((maxRegs[m][pr+1]&(1U<<(7-pc)))!=0,"pixel not aligned to photographed assembly at module="+std::to_string(m)+" logical="+std::to_string(r)+","+std::to_string(c));
  }
}
static void necks() {
  orientationHigh=true;clearFrame();
  putPixel(0,7,7,true);putPixel(1,0,0,true);flushFrame();
  require(maxRegs[0][1]==128 && pixels(0)==1,"M0 outlet does not point to its physical lower tip");
  require(maxRegs[1][1]==128 && pixels(1)==1,"M1 entry does not point to its physical upper tip");
}
static void handedness() {
  orientationHigh=true;clearFrame();
  putPixel(0,7,6,true);putPixel(1,7,6,true);flushFrame();
  require(maxRegs[0][1]==64,"M0 lower neighbor is mirrored onto wrong wall");
  require(maxRegs[1][8]==2,"M1 lower neighbor is mirrored onto wrong wall");
}
int main(int argc,char**argv){
 const std::map<std::string,std::function<void()>> cases={
  {"photographed_necks",necks},{"photographed_handedness",handedness},
  {"all_pixels_upright",[]{allPixels(false);}},{"all_pixels_inverted",[]{allPixels(true);}}
 };
 if(argc!=2){for(const auto&x:cases)std::cout<<x.first<<'\n';return 2;}
 try{auto i=cases.find(argv[1]);require(i!=cases.end(),"unknown case");i->second();std::cout<<"PASS "<<argv[1]<<'\n';return 0;}
 catch(const std::exception&e){std::cerr<<"FAIL "<<argv[1]<<": "<<e.what()<<'\n';return 1;}
}
