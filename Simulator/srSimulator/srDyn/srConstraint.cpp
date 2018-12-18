#include "srDyn/srConstraint.h"
#include "srDyn/srSystem.h"

//**********************************************************************//
// Constraint
srSystem* Constraint::UF_Find_System_PathCompression(srSystem* pSystem) // Path Compression
{
	while (pSystem->m_UF_id != pSystem)
	{
		pSystem->m_UF_id = pSystem->m_UF_id->m_UF_id;
		pSystem = pSystem->m_UF_id;
	}
	return pSystem;
}

srSystem* Constraint::UF_Find_System(srSystem* pSystem)
{
	while (pSystem->m_UF_id != pSystem)
	{
		pSystem = pSystem->m_UF_id;
	}
	return pSystem;
}
//**********************************************************************//

