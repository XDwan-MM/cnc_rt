#ifndef ITPPARAM_H
#define ITPPARAM_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    char read = 6;
    char write = 6;
    bool valid = false;
    double varDouble = 0;
} MacroVar;

#ifdef __cplusplus
}
#endif

#endif // ITPPARAM_H
