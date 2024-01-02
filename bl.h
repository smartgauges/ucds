#ifndef BL_H
#define BL_H

/* pages: 0-3 16KB */
#define ADDR_BL       0x08000000
#define SIZE_BL       0x00004000

/* pages: 4-63 112KB */
#define ADDR_APP      (ADDR_BL + SIZE_BL)
#define ADDR_APP_END  0x0801ffff

void start_app(void);

#endif

