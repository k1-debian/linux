
#ifndef __CCU_H__
#define __CCU_H__

#include <core_base.h>
#define CCU_RESET_ENTRY     0xbfc00000

#define get_ccu_cscr()          inl(CCU_IO_BASE + 0)
#define set_ccu_cscr(val)       outl(val, CCU_IO_BASE + 0)

#define get_ccu_cssr()          inl(CCU_IO_BASE + 0x20)

#define get_ccu_csrr()          inl(CCU_IO_BASE + 0x40)
#define set_ccu_csrr(val)       outl(val, CCU_IO_BASE + 0x40)

#define get_ccu_pipr()          inl(CCU_IO_BASE + 0x100)

#define get_ccu_pimr()          inl(CCU_IO_BASE + 0x120)
#define set_ccu_pimr(val)       outl(val, CCU_IO_BASE + 0x120)

#define get_ccu_mipr()          inl(CCU_IO_BASE + 0x140)

#define get_ccu_mimr()          inl(CCU_IO_BASE + 0x160)
#define set_ccu_mimr(val)       outl(val, CCU_IO_BASE + 0x160)

#define get_ccu_oipr()          inl(CCU_IO_BASE + 0x180)

#define get_ccu_oimr()          inl(CCU_IO_BASE + 0x1a0)
#define set_ccu_oimr(val)       outl(val, CCU_IO_BASE + 0x1a0)

#define get_ccu_rer()           inl(CCU_IO_BASE + 0xf00)
#define set_ccu_rer(val)        outl(val, CCU_IO_BASE + 0xf00)

#define get_ccu_cslr()          inl(CCU_IO_BASE + 0xff8)
#define set_ccu_cslr(val)       outl(val, CCU_IO_BASE + 0xff8)

#define get_ccu_csar()          inl(CCU_IO_BASE + 0xffc)
#define set_ccu_csar(val)       outl(val, CCU_IO_BASE + 0xffc)

//#define get_ccu_mbr(cpu_id)           inl(CCU_IO_BASE + 0x1000 + cpu_id*4)
//#define set_ccu_mbr(cpu_id, val)  outl(val, CCU_IO_BASE + 0x1000 + cpu_id*4)
#endif
