#include "ADRC_core.h"
#include "ADRC_user.h"
#include "arm_math.h"

static float sign(float x);
static float fhan(float x1,float x2,float r,float h);
static float fal(float e,float alpha,float delta);

static float sign(float x)
{
	if(x>0)
		return 1;
	else if(x<0)
		return -1;
	else
		return 0;
}


/*******************************fhan函数**********************************/
float fhan(float x1,float x2,float r,float h)
{
	float a    = 0,
		    a0   = 0,
		    y    = 0,
		    d		 = 0,
		    d0   = 0,
		    absy = 0,
		    absa = 0,
	      fhan = 0;
	absy =  fabsf(y);
  absa =  fabsf(a);
  d    =  r*h;
	d0   =  h*d;
  y    =  x1+h*x2;
  a0   =  sqrtf(d*d+8*r*absy);
	if(absy > d0)
		a = x2 + (a0 - d)/2 * sign(y);
	else
		a = x2 + y / h;
	if(absa > d)
		fhan = -1*r*sign(a);
	else
		fhan = -1*a/d;
  return fhan;
}

/*******************************fal函数**********************************/
float fal(float e,float alpha,float delta)
{
  float result = 0,fabsf_e = 0;
  fabsf_e = fabsf(e);
  if(delta>=fabsf_e)
    result = e/pow(delta,1-alpha);
  else if(delta<fabsf_e)
    result = sign(e)*pow(fabsf_e,alpha);
	return result;    
}


//中间变量区，不需要用户管理以及赋值
/****************TD*******************/
float x1 = 0,//跟踪输入
      x2 = 0,//跟踪输入的微分
/****************ESO******************/
      e  = 0,//误差
			z1 = 0,//跟踪反馈值
			z2 = 0,//跟踪反馈值的而微分
			z3 = 0,//跟踪系统的扰动（总扰动）
/**************NLSEF******************/
      u  = 0;//输出值
float u0 = 0,//非线性组合的输出
			e1 = 0,//z1和x1的误差
			e2 = 0;//z2和x2的误差

/********************************ADRC************************************/
float ADRC(ADRC_t *ADRC,float v,float y)
{

/******************************TD****************************************/
    x1 = x1 + ADRC->h*x2;
    x2 = x2 + ADRC->h*fhan(x1-v,x2,ADRC->r,ADRC->h);
/******************************ESO***************************************/
    e = z1 - y;
    z1 = z1 + ADRC->h*(z2 - ADRC->belta01*e );
    z2 = z2 + ADRC->h*(z3 - ADRC->belta02*fal(e,0.5,ADRC->delta) + ADRC->b*u);
    z3 = z3 + ADRC->h*(   - ADRC->belta03*fal(e,0.25,ADRC->delta));
/******************限幅，ADRC正常的话不会达到限幅条件********************/
    if(z1>=30000) z1=30000;
    if(z1<=-30000) z1 = -30000;
    if(z2>=30000) z2=30000;
    if(z2<=-30000) z2 = -30000;
    if(z3>=30000) z3=30000;
    if(z3<=-30000) z3 = -30000;
/******************************NLSEF*************************************/
    e1 = x1 - z1;
    e2 = x2 - z2;
  
    u0 = ADRC->belta1*fal(e1,ADRC->alpha1,ADRC->delta) + ADRC->belta2*fal(e2,ADRC->alpha2,ADRC->delta);//其中0<alpha1<1<alpha2
  
    u = u0 - z3/ADRC->b;
  
    return u;
}



