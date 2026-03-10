#ifndef NCITPAPI_H
#define NCITPAPI_H

#include "ncitpdatadef.h"

#ifdef __cplusplus
extern "C" {
#endif

extern void ItpSetFile(const char *filePath, int startLine = 0);

extern void ItpSetContent(const char *content, int startLine = 0);

extern void ItpSetOriginMacPos(double x, double y, double z, double a);

extern void ItpNextLine();

extern bool ItpIsOver();

extern int ItpError();

extern void ItpReset();

extern int ItpGetDataPacket(NcItpDataPacket *packet);

extern void ItpCfgGetMacroVarCB(GetMacroVarCB cb);
extern void ItpCfgSetMacroVarCB(SetMacroVarCB cb);

/**
 * @brief ItpGetMotionData
 * @param data 参考ncitpdatadef.h
 * @return
 */
extern int ItpGetMotionData(NcItpData *data);

extern int ItpGetLineNumber(int *num);

extern int ItpGetState(NcItpState *state);

#ifdef __cplusplus
}
#endif

#endif // NCITPAPI_H
