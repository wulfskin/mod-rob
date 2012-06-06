#ifndef __MACRO_H
#define __MACRO_H

#define SET(REGISTER,BIT) REGISTER|=(1<<BIT)
#define CLEAR(REGISTER,BIT) REGISTER &= ~(1<<BIT)
#define TOGGLE(REGISTER,BIT) REGISTER^=(1<<BIT)

#endif /* __MACRO_H */