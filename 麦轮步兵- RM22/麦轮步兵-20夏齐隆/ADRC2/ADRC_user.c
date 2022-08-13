#include "ADRC_core.h"
#include "ADRC_user.h"

void ADRC_init(ADRC_t *ADRC_type , float *a)
{
    ADRC_type->r = a[0];
    ADRC_type->h = a[1];

    ADRC_type->b        = a[2];
    ADRC_type->delta    = a[3];
    ADRC_type->belta01  = a[4];
    ADRC_type->belta02  = a[5];
    ADRC_type->belta03  = a[6];

    ADRC_type->alpha1 = a[7];
    ADRC_type->alpha2 = a[8];
    ADRC_type->belta1 = a[9];
    ADRC_type->belta2 = a[10];

    ADRC_type->set = 0.0f;
    ADRC_type->rel = 0.0f;
    ADRC_type->out = 0.0f;
    
}
