#include "ovpCBoxAlgorithmERSPAverage.h"
#include <cstdlib>

using namespace OpenViBE;
using namespace OpenViBE::Kernel;
using namespace OpenViBE::Plugins;

using namespace OpenViBEPlugins;
using namespace OpenViBEPlugins::SignalProcessing;

boolean CBoxAlgorithmERSPAverage::initialize(void)
{
	m_epochingStim = FSettingValueAutoCast(*this->getBoxAlgorithmContext(), 0);
	m_computeStim = FSettingValueAutoCast(*this->getBoxAlgorithmContext(), 1);

	m_oDecoderSpectrum.initialize(*this,0);
	m_oDecoderStimulations.initialize(*this,1);

	m_oEncoder.initialize(*this,0);


	return true;
}

boolean CBoxAlgorithmERSPAverage::uninitialize(void)
{
	m_oEncoder.uninitialize();
	m_oDecoderSpectrum.uninitialize();
	m_oDecoderStimulations.uninitialize();

	for(auto matrixVector : m_cachedSpectra)
	{
		for(auto matrixPtr : matrixVector)
		{
			delete matrixPtr;
		}
	}
	m_cachedSpectra.clear();

	return true;
}

boolean CBoxAlgorithmERSPAverage::processInput(uint32 ui32InputIndex)
{
	getBoxAlgorithmContext()->markAlgorithmAsReadyToProcess();
	return true;
}

boolean CBoxAlgorithmERSPAverage::process(void)
{
	IBoxIO& l_rDynamicBoxContext=this->getDynamicBoxContext();

	for(uint32 chunk=0; chunk<l_rDynamicBoxContext.getInputChunkCount(1); chunk++)
	{
		m_oDecoderStimulations.decode(chunk);
		if(m_oDecoderStimulations.isBufferReceived())
		{
			auto stims = m_oDecoderStimulations.getOutputStimulationSet();
			for(uint32_t i=0;i<stims->getStimulationCount();i++)
			{
				if(stims->getStimulationIdentifier(i) == m_epochingStim)
				{
					m_currentChunk = 0;
					m_numTrials++;
				}
				if(stims->getStimulationIdentifier(i) == m_computeStim)
				{
					computeAndSend();
				}
			}
		}
	}

	for(uint32 chunk=0; chunk<l_rDynamicBoxContext.getInputChunkCount(0); chunk++)
	{
		m_oDecoderSpectrum.decode(chunk);
		
		if(m_oDecoderSpectrum.isHeaderReceived())
		{
			const uint64_t samplingRate = m_oDecoderSpectrum.getOutputSamplingRate();
			m_oEncoder.getInputSamplingRate() = samplingRate;
			OpenViBEToolkit::Tools::Matrix::copy(*m_oEncoder.getInputFrequencyAbscissa(), *m_oDecoderSpectrum.getOutputFrequencyAbscissa());
			OpenViBEToolkit::Tools::Matrix::copyDescription(*m_oEncoder.getInputMatrix(), *m_oDecoderSpectrum.getOutputMatrix());
			m_oEncoder.encodeHeader();
			l_rDynamicBoxContext.markOutputAsReadyToSend(0, l_rDynamicBoxContext.getInputChunkStartTime(0, chunk), l_rDynamicBoxContext.getInputChunkEndTime(0, chunk));
		}

		if(m_oDecoderSpectrum.isBufferReceived())
		{
			const IMatrix* l_pInput = m_oDecoderSpectrum.getOutputMatrix();
			appendChunk(*l_pInput, l_rDynamicBoxContext.getInputChunkStartTime(0, chunk), l_rDynamicBoxContext.getInputChunkEndTime(0, chunk));
		}

		if(m_oDecoderSpectrum.isEndReceived())
		{
			m_oEncoder.encodeEnd();
			l_rDynamicBoxContext.markOutputAsReadyToSend(0, l_rDynamicBoxContext.getInputChunkStartTime(0, chunk), l_rDynamicBoxContext.getInputChunkEndTime(0, chunk));
		}
	}

	return true;
}

bool CBoxAlgorithmERSPAverage::appendChunk(const IMatrix& chunk, uint64_t startTime, uint64_t endTime)
{
	if(m_cachedSpectra.size() <= m_currentChunk)
	{
		m_cachedSpectra.resize(m_currentChunk+1);
		m_timestamps.resize(m_currentChunk+1);
	}

	CMatrix* matrixCopy = new CMatrix();
	OpenViBEToolkit::Tools::Matrix::copy(*matrixCopy, chunk);
	m_cachedSpectra[m_currentChunk].push_back(matrixCopy);
	m_timestamps[m_currentChunk].m_startTime = startTime;
	m_timestamps[m_currentChunk].m_endTime = endTime;

	m_currentChunk++;

	return true;
}

bool CBoxAlgorithmERSPAverage::computeAndSend(void)
{
	IBoxIO& l_rDynamicBoxContext=this->getDynamicBoxContext();
//	uint64_t now = this->getPlayerContext().getCurrentTime();

	float64* outptr = m_oEncoder.getInputMatrix()->getBuffer();

	this->getLogManager() << LogLevel_Info << "Counted " << m_numTrials << " trials and " << m_cachedSpectra.size() << " spectra per trial.\n";

	for(size_t i=0;i<m_cachedSpectra.size();i++)
	{
		// Compute average for each slice
		const float64 divider = 1.0 / m_cachedSpectra[i].size();

		OpenViBEToolkit::Tools::Matrix::clearContent(*m_oEncoder.getInputMatrix());

		for(auto mat : m_cachedSpectra[i])
		{
			const float64* inptr = mat->getBuffer();
			for(size_t p = 0;p<mat->getBufferElementCount(); p++)
			{
				outptr[p] += divider*inptr[p];
			}
			delete mat;
		}

		m_oEncoder.encodeBuffer();
//		uint64_t duration = m_timestamps[i].m_endTime - m_timestamps[i].m_startTime;
//		l_rDynamicBoxContext.markOutputAsReadyToSend(0, now+(i-1)*duration, now+i*duration);
		l_rDynamicBoxContext.markOutputAsReadyToSend(0, m_timestamps[i].m_startTime - m_timestamps[0].m_startTime, m_timestamps[i].m_endTime - m_timestamps[0].m_startTime);
	}

	m_numTrials = 0;
	m_currentChunk = 0;
	m_cachedSpectra.clear();

	return true;
}

