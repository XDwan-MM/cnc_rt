#ifndef MASTER_H
#define MASTER_H

#ifdef __cplusplus
extern "C" {
#endif

extern int master_error_code;

int Master_Start();
void Master_Run();
void Master_Error(int code);
void Master_Reset();

#ifdef __cplusplus
}
#endif
#endif // MASTER_H

