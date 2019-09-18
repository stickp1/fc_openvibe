#if defined TARGET_HAS_ThirdPartyITPP

//#define __OpenViBEPlugins_BoxAlgorithm_IFFTbox_CPP__
// to get ifft:
#include <itpp/itsignal.h>
#include "ovpCBoxAlgorithmIFFTbox.h"


using namespace OpenViBE;
using namespace OpenViBE::Kernel;
using namespace OpenViBE::Plugins;

using namespace OpenViBEPlugins;
using namespace OpenViBEPlugins::SignalProcessingBasic;


bool CBoxAlgorithmIFFTbox::initialize(void)
{
	// Spectrum stream real part decoder
	m_oAlgo0_SpectrumDecoder[0].initialize(*this,0);
	// Spectrum stream imaginary part decoder
	m_oAlgo0_SpectrumDecoder[1].initialize(*this,1);
	// Signal stream encoder
	m_oAlgo1_SignalEncoder.initialize(*this,0);
	
	m_ui32SampleCount = 0;

	m_bHeaderSent = false;

	return true;
}
/*******************************************************************************/

bool CBoxAlgorithmIFFTbox::uninitialize(void)
{

	m_oAlgo0_SpectrumDecoder[0].uninitialize();
	m_oAlgo0_SpectrumDecoder[1].uninitialize();
	m_oAlgo1_SignalEncoder.uninitialize();
	
	return true;
}

bool CBoxAlgorithmIFFTbox::processInput(uint32_t ui32InputIndex)
{
	const IBox& l_rStaticBoxContext = this->getStaticBoxContext();
	IDynamicBoxContext& l_rDynamicBoxContext=this->getDynamicBoxContext();

	if(l_rDynamicBoxContext.getInputChunkCount(0) == 0)
	{
		return true;
	}
	uint64_t l_ui64StartTime=l_rDynamicBoxContext.getInputChunkStartTime(0, 0);
	uint64_t l_ui64EndTime=l_rDynamicBoxContext.getInputChunkEndTime(0, 0);
	for(uint32_t i=1; i<l_rStaticBoxContext.getInputCount(); i++)
	{
		if(l_rDynamicBoxContext.getInputChunkCount(i)==0)
		{
			return true;
		}

		if(l_ui64StartTime != l_rDynamicBoxContext.getInputChunkStartTime(i, 0)
			|| l_ui64EndTime != l_rDynamicBoxContext.getInputChunkEndTime(i, 0))
		{
			OV_WARNING_K("Chunk dates mismatch, check stream structure or parameters");
			return false;
		}
	}

	getBoxAlgorithmContext()->markAlgorithmAsReadyToProcess();

	return true;
}
/*******************************************************************************/

bool CBoxAlgorithmIFFTbox::process(void)
{
	
	// the static box context describes the box inputs, outputs, settings structures
	const IBox& l_rStaticBoxContext = this->getStaticBoxContext();
	// the dynamic box context describes the current state of the box inputs and outputs (i.e. the chunks)
	IBoxIO& l_rDynamicBoxContext=this->getDynamicBoxContext();
	
	uint32_t l_ui32HeaderCount=0;
	uint32_t l_ui32BufferCount=0;
	uint32_t l_ui32EndCount=0;

	for(uint32_t i=0; i<l_rStaticBoxContext.getInputCount(); i++)
	{
		m_oAlgo0_SpectrumDecoder[i].decode(0);
		if(m_oAlgo0_SpectrumDecoder[i].isHeaderReceived())
		{
			//detect if header of other input is already received 
			if(0==l_ui32HeaderCount)
			{
				// Header received. This happens only once when pressing "play". For example with a StreamedMatrix input, you now know the dimension count, sizes, and labels of the matrix
				// ... maybe do some process ...
				m_channelsNumber=m_oAlgo0_SpectrumDecoder[i].getOutputMatrix()->getDimensionSize(0);
				m_ui32SampleCount = m_oAlgo0_SpectrumDecoder[i].getOutputMatrix()->getDimensionSize(1);
				OV_ERROR_UNLESS_KRF(
					m_channelsNumber > 0 && m_ui32SampleCount > 0,
					"Both dims of the input matrix must have positive size",
					OpenViBE::Kernel::ErrorType::BadProcessing);

				m_ui32SampleCount = (m_ui32SampleCount-1)*2;
				if(m_ui32SampleCount == 0) 
				{
					m_ui32SampleCount = 1;
				}
			}
			else
			{
				OV_ERROR_UNLESS_KRF(
					OpenViBEToolkit::Tools::Matrix::isDescriptionSimilar(*m_oAlgo0_SpectrumDecoder[0].getOutputMatrix(), *m_oAlgo0_SpectrumDecoder[i].getOutputMatrix(), false),
					"The matrix components of the two streams have different properties, check stream structures or parameters",
					OpenViBE::Kernel::ErrorType::BadProcessing);

				OV_ERROR_UNLESS_KRF(
					OpenViBEToolkit::Tools::Matrix::isDescriptionSimilar(*m_oAlgo0_SpectrumDecoder[0].getOutputFrequencyAbscissa(), *m_oAlgo0_SpectrumDecoder[i].getOutputFrequencyAbscissa(), false),
					"The frequencies abscissas descriptors of the two streams have different properties, check stream structures or parameters",
					OpenViBE::Kernel::ErrorType::BadProcessing);

				OV_ERROR_UNLESS_KRF(
					m_oAlgo0_SpectrumDecoder[0].getOutputMatrix()->getDimensionSize(1) == m_oAlgo0_SpectrumDecoder[i].getOutputFrequencyAbscissa()->getDimensionSize(0),
					"Frequencies abscissas count " <<  m_oAlgo0_SpectrumDecoder[i].getOutputFrequencyAbscissa()->getDimensionSize(0)
						<< " does not match the corresponding matrix chunk size " << m_oAlgo0_SpectrumDecoder[0].getOutputMatrix()->getDimensionSize(1)
						<< ", check stream structures or parameters",
					OpenViBE::Kernel::ErrorType::BadProcessing);

				OV_ERROR_UNLESS_KRF(
					m_oAlgo0_SpectrumDecoder[0].getOutputSamplingRate(),
					"Sampling rate must be positive, check stream structures or parameters",
					OpenViBE::Kernel::ErrorType::BadProcessing);

				OV_ERROR_UNLESS_KRF(
					m_oAlgo0_SpectrumDecoder[0].getOutputSamplingRate() == m_oAlgo0_SpectrumDecoder[i].getOutputSamplingRate(),
					"Sampling rates don't match (" <<  m_oAlgo0_SpectrumDecoder[0].getOutputSamplingRate()
						<< " != " << m_oAlgo0_SpectrumDecoder[i].getOutputSamplingRate() << "), please check stream structures or parameters",
					OpenViBE::Kernel::ErrorType::BadProcessing);
			}

			l_ui32HeaderCount++;
		}
		if(m_oAlgo0_SpectrumDecoder[i].isBufferReceived()) l_ui32BufferCount++;
		if(m_oAlgo0_SpectrumDecoder[i].isEndReceived()) l_ui32EndCount++;
	}
	
	if((l_ui32HeaderCount && l_ui32HeaderCount!=l_rStaticBoxContext.getInputCount())
	|| (l_ui32BufferCount && l_ui32BufferCount!=l_rStaticBoxContext.getInputCount())
	|| (l_ui32EndCount && l_ui32EndCount!=l_rStaticBoxContext.getInputCount()))
	{
		OV_WARNING_K("Stream structure mismatch");
		return false;
	}
	
	if(l_ui32BufferCount)
	{
		OV_ERROR_UNLESS_KRF(
			m_ui32SampleCount,
			"Received buffer before header, shouldn't happen\n",
			OpenViBE::Kernel::ErrorType::BadProcessing);

		if(!m_bHeaderSent) 
		{
			m_signalBuffer.set_size(m_ui32SampleCount);
			m_frequencyBuffer.set_size(m_ui32SampleCount);

			m_oAlgo1_SignalEncoder.getInputSamplingRate() = m_oAlgo0_SpectrumDecoder[0].getOutputSamplingRate();
			m_oAlgo1_SignalEncoder.getInputMatrix()->setDimensionCount(2);
			m_oAlgo1_SignalEncoder.getInputMatrix()->setDimensionSize(0,m_channelsNumber);
			m_oAlgo1_SignalEncoder.getInputMatrix()->setDimensionSize(1,m_ui32SampleCount);
			for(uint32_t channel=0; channel< m_channelsNumber; channel++)
			{
				m_oAlgo1_SignalEncoder.getInputMatrix()->setDimensionLabel(0, channel, 
														m_oAlgo0_SpectrumDecoder[0].getOutputMatrix()->getDimensionLabel(0, channel));
			}

			// Pass the header to the next boxes, by encoding a header on the output 0:
			m_oAlgo1_SignalEncoder.encodeHeader();
			// send the output chunk containing the header. The dates are the same as the input chunk:
			l_rDynamicBoxContext.markOutputAsReadyToSend(0, l_rDynamicBoxContext.getInputChunkStartTime(0, 0), l_rDynamicBoxContext.getInputChunkEndTime(0, 0));
				
			m_bHeaderSent = true;
		}

		const float64* bufferInput0= m_oAlgo0_SpectrumDecoder[0].getOutputMatrix()->getBuffer();
		const float64* bufferInput1= m_oAlgo0_SpectrumDecoder[1].getOutputMatrix()->getBuffer();

		for(uint32_t channel=0; channel< m_channelsNumber; channel++)
		{
			for(uint32_t j=0; j< m_ui32SampleCount; j++)
			{
				m_frequencyBuffer[j].real(bufferInput0[channel * m_ui32SampleCount + j]);
				m_frequencyBuffer[j].imag(bufferInput1[channel * m_ui32SampleCount + j]);
			}

			m_signalBuffer = itpp::ifft_real(m_frequencyBuffer);

			// Test block
			// std::cout << "Iy: " << m_frequencyBuffer[channel].size() << ", y=" << m_frequencyBuffer[channel] << "\n";
			// std::cout << "Ix: " << m_signalBuffer[channel].size() << ", x'=" << m_signalBuffer[channel] << "\n";

			float64* bufferOutput = m_oAlgo1_SignalEncoder.getInputMatrix()->getBuffer();
			for(uint32_t j=0; j< m_ui32SampleCount; j++)
			{
				bufferOutput[channel*m_ui32SampleCount+j] = m_signalBuffer[j];
			}
		}
		// Encode the output buffer :
		m_oAlgo1_SignalEncoder.encodeBuffer();
		// and send it to the next boxes :
		l_rDynamicBoxContext.markOutputAsReadyToSend(0, l_rDynamicBoxContext.getInputChunkStartTime(0, 0), 	l_rDynamicBoxContext.getInputChunkEndTime(0, 0));
		
	}																										
	if(l_ui32EndCount)
	{
		// End of stream received. This happens only once when pressing "stop". Just pass it to the next boxes so they receive the message :
		m_oAlgo1_SignalEncoder.encodeEnd();
		l_rDynamicBoxContext.markOutputAsReadyToSend(0, l_rDynamicBoxContext.getInputChunkStartTime(0, 0), l_rDynamicBoxContext.getInputChunkEndTime(0, 0));
	}																								

	return true;
}

#endif //TARGET_HAS_ThirdPartyITPP
