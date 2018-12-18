#ifndef _UTILS_H_
#define _UTILS_H_

#define SR_SAFE_DELETE(p)			if(p) { delete (p); (p) = NULL; }
#define SR_SAFE_DESTROY_WINDOW(p) if(p) { p->DestroyWindow(); delete (p); (p) = NULL; }
#define SR_SAFE_DELETE_AR(p)		if(p) { delete [] p; (p) = NULL; }
#define SR_SAFE_RELEASE(p)		if(p) { (p)->Release(); (p) = NULL; }

class srUtils
{

};

#endif // _UTILS_H_
