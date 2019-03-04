#include "cordicTrig.h" 

/*******************************************************************************
* Function Name : CORDICsincos
* Description :   ������� ���������� �������� ������ � �������� ����.
*     ��������� � ������������ �������� - 32-� ������ ����� ����� �� ������.
*     ��� ������ ��������� � ������������� �������� - GLOBAL_Q.
*			����: ���� 'a' (���), ��������� 'm'
*     �����: ����� 's', ������� 'c'
*     �������� �������� ��� ����������: ���� �������� ���� ������.
*     ����� ���������� ...
*******************************************************************************/

static inline void CORDICsincos(int32_t a, int32_t m, int32_t *s, int32_t *c) {
 int32_t k, tx, x=m, y=0, z=a, fl=0;
 if (z>+CORDIC_HALFPI) { fl=+1; z = (+CORDIC_PI) - z; }
 else if (z<-CORDIC_HALFPI) { fl=+1; z = (-CORDIC_PI) - z; }
 for (k=0; k<CORDIC_MAXITER; k++) {
 tx = x;
 if (z>=0) { x -= (y>>k); y += (tx>>k); z -= CORDIC_ZTBL[k]; }
 else { x += (y>>k); y -= (tx>>k); z += CORDIC_ZTBL[k]; } }
 if (fl) x=-x;
 *c = x; // m*cos(a) multiplied by gain F and factor M
 *s = y; // m*sin(a) multiplied by gain F and factor M
}

/*******************************************************************************
* Function Name : CORDICatan2sqrt
* Description :   ������� ���������� �����������.
*     ��������� � ������������ �������� - 32-� ������ ����� ����� �� ������.
*     ��� ������ ��������� � ������������� �������� - GLOBAL_Q.
*			����: ����� 's', ������� 'c'
*     �����: ���� 'a' (���), ��������� 'm'
*     ����� ���������� ...
*******************************************************************************/
static inline void CORDICatan2sqrt(int32_t *a, int32_t *m, int32_t y, int32_t x) {
 int32_t k, tx, z=0, fl=0;
 if (x<0) { fl=((y>0)?+1:-1); x=-x; y=-y; }
 for (k=0; k<CORDIC_MAXITER; k++) {
 tx = x;
 if (y<=0) { x -= (y>>k); y += (tx>>k); z -= CORDIC_ZTBL[k]; }
 else { x += (y>>k); y -= (tx>>k); z += CORDIC_ZTBL[k]; } }
 if (fl!=0) { z += fl*CORDIC_PI; }
 *a = z; // radians multiplied by factor M
 *m = x; // sqrt(x^2+y^2) multiplied by gain F
}
