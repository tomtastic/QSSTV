#ifndef CSOUNDOUT_H
#define CSOUNDOUT_H
#include "GlobalDefinitions.h"
#include "soundinterface.h"

class CSoundOut : public CSoundOutInterface {
 public:
  CSoundOut();
  ~CSoundOut() override;
  //  void Init(int iNewBufferSize, bool bNewBlocking);
  bool Write(CVector<_SAMPLE>& psData) override;
};

#endif  // CSOUNDOUT_H
