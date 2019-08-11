/*=====================================================================================
Ôóíêöèÿ ïîäñ÷åòà êîíñòàíò äëÿ ìîäåëè ÄÏÒ ñ ÍÂ    
-------------------------------------------------------------------------------------*/

#include "bdcm_const.h"

void bdcm_const_calc(BDCM_CONST *v)
{	
float Le_base,La_base,Le_pu,La_pu,Ra_pu,Re_pu;
float W_base,Tm,M_base;
// ñ÷èòàåì çàâèñèìûå îòíîñèòåëüíûå âåëè÷èíû	
// äëÿ îáìîòêè âîçáóæäåíèÿ    
    Le_base=v->Ue_base*v->T_base/v->Ie_base;
    Le_pu = v->Le/Le_base;
    v->Re_base=v->Ue_base/v->Ie_base;
    Re_pu = v->Re/v->Re_base; 
    v->Te_pu=Le_pu/Re_pu;
// äëÿ îáìîòêèÿ ÿêîğÿ    
    La_base=v->Ua_base/v->Ia_base;
    La_pu = v->La*v->T_base/La_base;
    v->Ra_base=v->Ua_base/v->Ia_base;    
    Ra_pu = v->Ra/v->Ra_base;
    v->Ta_pu=La_pu/Ra_pu;

// äğóãèå    
    W_base=v->Ua_base/v->k_fi_nom;
    M_base=v->k_fi_nom*v->Ia_base;
    Tm=v->J*W_base/M_base;
    v->Tm_pu=Tm/v->T_base;

// ñ÷èòàåì êîıôèöèåíòû äëÿ ìîäåëè   
    v->K1 = v->Ts/(Re_pu*(v->Te_pu+v->Ts));  
    v->K2 = v->Te_pu/(v->Te_pu+v->Ts);
    
    v->K3 = v->Ts/(Ra_pu*(v->Ts+v->Ta_pu));  
    v->K4 = v->Ta_pu/(v->Ta_pu+v->Ts);
    
    v->K5 = v->Ts/v->Tm_pu;  
}


