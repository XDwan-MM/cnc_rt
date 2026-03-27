#ifndef MASTER_H
#define MASTER_H

#ifdef __cplusplus
extern "C" {
#endif

void Master_Run();
void Master_Error(int code);
void Master_Reset();

extern int master_error_code;
#ifdef __cplusplus
}
#endif
#endif // MASTER_H

