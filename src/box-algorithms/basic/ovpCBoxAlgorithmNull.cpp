#include "ovpCBoxAlgorithmNull.h"

#include <iostream>

using namespace OpenViBE;
using namespace OpenViBE::Kernel;
using namespace OpenViBEPlugins::SignalProcessing;

void CBoxAlgorithmNull::release(void)
{
	delete this;
}

boolean CBoxAlgorithmNull::processInput(uint32 ui32InputIndex)
{
	getBoxAlgorithmContext()->markAlgorithmAsReadyToProcess();
	return true;
}

boolean CBoxAlgorithmNull::process(void)
{
	const IBox* l_pStaticBoxContext=getBoxAlgorithmContext()->getStaticBoxContext();
	IBoxIO* l_pDynamicBoxContext=getBoxAlgorithmContext()->getDynamicBoxContext();

	for(uint32_t i=0; i<l_pStaticBoxContext->getInputCount(); i++)
	{
		for(uint32_t j=0; j<l_pDynamicBoxContext->getInputChunkCount(i); j++)
		{
			l_pDynamicBoxContext->markInputAsDeprecated(i, j);
		}
	}

	return true;
}
