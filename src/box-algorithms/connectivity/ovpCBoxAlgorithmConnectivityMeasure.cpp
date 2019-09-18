#if defined(TARGET_HAS_ThirdPartyEIGEN)

#include "ovpCBoxAlgorithmConnectivityMeasure.h"

#include <cstdio>
#include <iostream>

using namespace OpenViBE;
using namespace OpenViBE::Kernel;
using namespace OpenViBE::Plugins;

using namespace OpenViBEPlugins;
using namespace OpenViBEPlugins::SignalProcessing;

using namespace OpenViBEToolkit;

using namespace std;

/**********************************************************************************************************************
 * _find_channel_ and the method to split the channel list was taken from the channel selector created by Yann Renard *
 **********************************************************************************************************************/

namespace
{
	inline uint32_t _find_channel_(const IMatrix& rMatrix, const CString& rChannel, const CIdentifier& rMatchMethodIdentifier, uint32_t uiStart=0)
	{
		uint32_t i, l_ui32Result = uint32_t(-1);

		if(rMatchMethodIdentifier==OVP_TypeId_MatchMethod_Name)
		{
			for(i=uiStart; i<rMatrix.getDimensionSize(0); i++)
			{
				if(Tools::String::isAlmostEqual(rMatrix.getDimensionLabel(0, i), rChannel, false))
				{
					l_ui32Result=i;
				}
			}
		}
		else if(rMatchMethodIdentifier==OVP_TypeId_MatchMethod_Index)
		{
			unsigned int value;
			if(::sscanf(rChannel.toASCIIString(), "%u", &value)==1)
			{
				value--; // => makes it 0-indexed !
				if(uiStart <= uint32_t(value) && uint32_t(value) < rMatrix.getDimensionSize(0))
				{
					l_ui32Result=uint32_t(value);
				}
			}
		}
		else if(rMatchMethodIdentifier==OVP_TypeId_MatchMethod_Smart)
		{
			if(l_ui32Result==uint32_t(-1)) l_ui32Result=_find_channel_(rMatrix, rChannel, OVP_TypeId_MatchMethod_Name, uiStart);
			if(l_ui32Result==uint32_t(-1)) l_ui32Result=_find_channel_(rMatrix, rChannel, OVP_TypeId_MatchMethod_Index, uiStart);
		}
		return l_ui32Result;

	}

};



bool CBoxAlgorithmConnectivityMeasure::initialize(void)
{
	const IBox& l_rStaticBoxContext = this->getStaticBoxContext();

	m_ui32InputCount = l_rStaticBoxContext.getInputCount();
	m_ui32OutputCount = l_rStaticBoxContext.getOutputCount();

	// Retrieve algorithm chosen by the user
	CIdentifier l_oConnectivityAlgorithmClassIdentifier;
	CString l_sConnectivityAlgorithmClassIdentifier = FSettingValueAutoCast(*this->getBoxAlgorithmContext(), 0);
	l_oConnectivityAlgorithmClassIdentifier=this->getTypeManager().getEnumerationEntryValueFromName(OVP_ClassId_ConnectivityAlgorithm, l_sConnectivityAlgorithmClassIdentifier);

	// Display an error message if the algorithm is not recognized
	OV_ERROR_UNLESS_KRF(
		l_oConnectivityAlgorithmClassIdentifier != OV_UndefinedIdentifier,
		"Unknown connectivity algorithm [" << l_sConnectivityAlgorithmClassIdentifier << "]",
		OpenViBE::Kernel::ErrorType::BadSetting);

	// Create algorithm instance of type ConnectivityAlgorithm
	m_pConnectivityMethod = &this->getAlgorithmManager().getAlgorithm(this->getAlgorithmManager().createAlgorithm(l_oConnectivityAlgorithmClassIdentifier));
	m_pConnectivityMethod->initialize();

	// Initialize
	ip_pMatrix1.initialize(m_pConnectivityMethod->getInputParameter(OVP_Algorithm_Connectivity_InputParameterId_InputMatrix1));
	ip_pMatrix2.initialize(m_pConnectivityMethod->getInputParameter(OVP_Algorithm_Connectivity_InputParameterId_InputMatrix2));
	ip_pChannelTable.initialize(m_pConnectivityMethod->getInputParameter(OVP_Algorithm_Connectivity_InputParameterId_LookupMatrix));
	op_pMatrix.initialize(m_pConnectivityMethod->getOutputParameter(OVP_Algorithm_Connectivity_OutputParameterId_OutputMatrix));

	// Signal stream decoder and encoder initialization
	m_oAlgo0_SignalDecoder.initialize(*this,0);
	m_oAlgo1_SignalEncoder.initialize(*this,0);
	
	// if an input was added, creation of the corresponding decoder
	if(m_ui32InputCount==2)
	{
		m_oAlgo2_SignalDecoder.initialize(*this,1);
	}
	// The box can't process more than two input
	else if(m_ui32InputCount > 2)
	{
		this->getLogManager() << LogLevel_ImportantWarning << "Incorrect number of inputs\n";
		return false;
	}

	// if an output was added, creation of the corresponding encoder
	if(m_ui32OutputCount>1)
	{
		m_oAlgo3_SpectrumEncoder.initialize(*this,1);
	}

	// if MScoherence is computed, manage corresponding settings and output
	if(l_oConnectivityAlgorithmClassIdentifier == OVP_TypeId_Algorithm_MagnitudeSquaredCoherence)
	{
		op_pMatrix2.initialize(m_pConnectivityMethod->getOutputParameter(OVP_Algorithm_MagnitudeSquaredCoherence_OutputParameterId_OutputMatrixSpectrum));
		ip_ui64SamplingRate1.initialize(m_pConnectivityMethod->getInputParameter(OVP_Algorithm_Connectivity_InputParameterId_ui64SamplingRate1));
		op_pFrequencyVector.initialize(m_pConnectivityMethod->getOutputParameter(OVP_Algorithm_MagnitudeSquaredCoherence_OutputParameterId_FreqVector));
		
		// Retrieve windowing method specified by the user
		CIdentifier l_oWindowMethodIdentifier;
		CString l_sWindowMethodIdentifier = FSettingValueAutoCast(*this->getBoxAlgorithmContext(), 3);
		l_oWindowMethodIdentifier=this->getTypeManager().getEnumerationEntryValueFromName(OVP_TypeId_WindowType, l_sWindowMethodIdentifier);

		// Initialize other settings
		ip_ui64WindowMethod.initialize(m_pConnectivityMethod->getInputParameter(OVP_Algorithm_MagnitudeSquaredCoherence_InputParameterId_Window));
		ip_ui64SegmentsLength.initialize(m_pConnectivityMethod->getInputParameter(OVP_Algorithm_MagnitudeSquaredCoherence_InputParameterId_SegLength));
		ip_ui64Overlap.initialize(m_pConnectivityMethod->getInputParameter(OVP_Algorithm_MagnitudeSquaredCoherence_InputParameterId_Overlap));

		//ip_ui64WindowMethod = l_oWindowMethodIdentifier.toUInteger();
		ip_ui64SegmentsLength = FSettingValueAutoCast(*this->getBoxAlgorithmContext(), 5);
		ip_ui64Overlap = FSettingValueAutoCast(*this->getBoxAlgorithmContext(), 4);

		m_oAlgo3_SpectrumEncoder.getInputMatrix().setReferenceTarget(op_pMatrix2);
		m_oAlgo3_SpectrumEncoder.getInputFrequencyAbscissa().setReferenceTarget(op_pFrequencyVector);
		m_oAlgo3_SpectrumEncoder.getInputSamplingRate().setReferenceTarget(m_oAlgo0_SignalDecoder.getOutputSamplingRate());
	}
	
	
	//if Imcoherence is computed, manage corresponding settings and output
	if(l_oConnectivityAlgorithmClassIdentifier == OVP_TypeId_Algorithm_ImCoherence)
	{
		op_pMatrix2.initialize(m_pConnectivityMethod->getOutputParameter(OVP_Algorithm_ImCoherence_OutputParameterId_OutputMatrixSpectrum));
		ip_ui64SamplingRate1.initialize(m_pConnectivityMethod->getInputParameter(OVP_Algorithm_Connectivity_InputParameterId_ui64SamplingRate1));
		op_pFrequencyVector.initialize(m_pConnectivityMethod->getOutputParameter(OVP_Algorithm_ImCoherence_OutputParameterId_FreqVector));

		// Retrieve windowing method specified by the user
		CIdentifier l_oWindowMethodIdentifier;
		CString l_sWindowMethodIdentifier = FSettingValueAutoCast(*this->getBoxAlgorithmContext(), 3);
		l_oWindowMethodIdentifier=this->getTypeManager().getEnumerationEntryValueFromName(OVP_TypeId_WindowType, l_sWindowMethodIdentifier);

		// Initialize other settings
		ip_ui64WindowMethod.initialize(m_pConnectivityMethod->getInputParameter(OVP_Algorithm_ImCoherence_InputParameterId_Window));
		ip_ui64SegmentsLength.initialize(m_pConnectivityMethod->getInputParameter(OVP_Algorithm_ImCoherence_InputParameterId_SegLength));
		ip_ui64Overlap.initialize(m_pConnectivityMethod->getInputParameter(OVP_Algorithm_ImCoherence_InputParameterId_Overlap));

		//ip_ui64WindowMethod = l_oWindowMethodIdentifier.toUInteger();	
		ip_ui64SegmentsLength = FSettingValueAutoCast(*this->getBoxAlgorithmContext(), 5);
		ip_ui64Overlap = FSettingValueAutoCast(*this->getBoxAlgorithmContext(), 3);

		m_oAlgo3_SpectrumEncoder.getInputMatrix().setReferenceTarget(op_pMatrix2);
		m_oAlgo3_SpectrumEncoder.getInputFrequencyAbscissa().setReferenceTarget(op_pFrequencyVector);
		m_oAlgo3_SpectrumEncoder.getInputSamplingRate().setReferenceTarget(m_oAlgo0_SignalDecoder.getOutputSamplingRate());
	}


	//if PSI is computed, manage corresponding settings and output
	if(l_oConnectivityAlgorithmClassIdentifier == OVP_TypeId_Algorithm_PhaseSlopeIndex)
	{
		op_pMatrix2.initialize(m_pConnectivityMethod->getOutputParameter(OVP_Algorithm_PhaseSlopeIndex_OutputParameterId_OutputMatrixSpectrum));
		ip_ui64SamplingRate1.initialize(m_pConnectivityMethod->getInputParameter(OVP_Algorithm_Connectivity_InputParameterId_ui64SamplingRate1));

		// Retrieve windowing method specified by the user
		CIdentifier l_oWindowMethodIdentifier;
		CString l_sWindowMethodIdentifier = FSettingValueAutoCast(*this->getBoxAlgorithmContext(), 3);
		l_oWindowMethodIdentifier=this->getTypeManager().getEnumerationEntryValueFromName(OVP_TypeId_WindowType, l_sWindowMethodIdentifier);

		// Initialize other settings
		ip_ui64WindowMethod.initialize(m_pConnectivityMethod->getInputParameter(OVP_Algorithm_PhaseSlopeIndex_InputParameterId_Window));
		ip_ui64SegmentsLength.initialize(m_pConnectivityMethod->getInputParameter(OVP_Algorithm_PhaseSlopeIndex_InputParameterId_SegLength));
		ip_ui64Overlap.initialize(m_pConnectivityMethod->getInputParameter(OVP_Algorithm_PhaseSlopeIndex_InputParameterId_Overlap));
		ip_ui64FreqWindow.initialize(m_pConnectivityMethod->getInputParameter(OVP_Algorithm_PhaseSlopeIndex_InputParameterId_FreqWindow));

		//ip_ui64WindowMethod = l_oWindowMethodIdentifier.toUInteger();
		ip_ui64SegmentsLength = FSettingValueAutoCast(*this->getBoxAlgorithmContext(), 4);
		ip_ui64FreqWindow = FSettingValueAutoCast(*this->getBoxAlgorithmContext(), 5);
		ip_ui64Overlap = FSettingValueAutoCast(*this->getBoxAlgorithmContext(), 6);

		m_oAlgo3_SpectrumEncoder.getInputMatrix().setReferenceTarget(op_pMatrix2);
		m_oAlgo3_SpectrumEncoder.getInputFrequencyAbscissa().setReferenceTarget(m_pConnectivityMethod->getOutputParameter(OVP_Algorithm_PhaseSlopeIndex_OutputParameterId_FreqVector));
		m_oAlgo3_SpectrumEncoder.getInputSamplingRate().setReferenceTarget(m_oAlgo0_SignalDecoder.getOutputSamplingRate());
	}



	ip_pChannelTable->setDimensionCount(2);
	ip_pChannelTable->setDimensionSize(1,1);

	// Set reference target
	ip_pMatrix1.setReferenceTarget(m_oAlgo0_SignalDecoder.getOutputMatrix());
	ip_ui64SamplingRate1.setReferenceTarget(m_oAlgo0_SignalDecoder.getOutputSamplingRate());

	if( m_ui32InputCount == 2)
	{
		ip_pMatrix2.setReferenceTarget(m_oAlgo2_SignalDecoder.getOutputMatrix());
	}
	else
	{
		ip_pMatrix2.setReferenceTarget(m_oAlgo0_SignalDecoder.getOutputMatrix());
	}

	m_oAlgo1_SignalEncoder.getInputMatrix().setReferenceTarget(op_pMatrix);
	m_oAlgo1_SignalEncoder.getInputSamplingRate().setReferenceTarget(m_oAlgo0_SignalDecoder.getOutputSamplingRate());

	return true;
}
/*******************************************************************************/

bool CBoxAlgorithmConnectivityMeasure::uninitialize(void)
{
	m_pConnectivityMethod->uninitialize();
	this->getAlgorithmManager().releaseAlgorithm(*m_pConnectivityMethod);

	m_oAlgo0_SignalDecoder.uninitialize();
	m_oAlgo1_SignalEncoder.uninitialize();

	// if a second decoder algorithm was created
	if(m_ui32InputCount==2)
	{
		m_oAlgo2_SignalDecoder.uninitialize();
	}
	if(m_ui32OutputCount>1)
	{
		m_oAlgo3_SpectrumEncoder.uninitialize();
	}

	return true;
}


bool CBoxAlgorithmConnectivityMeasure::processInput(uint32_t ui32InputIndex)
{

		const IBox& l_rStaticBoxContext = this->getStaticBoxContext();
		IDynamicBoxContext& l_rDynamicBoxContext=this->getDynamicBoxContext();

		if(l_rDynamicBoxContext.getInputChunkCount(0) == 0)
		{
			return true;
		}

		uint64 l_ui64StartTime=l_rDynamicBoxContext.getInputChunkStartTime(0, 0);
		uint64 l_ui64EndTime=l_rDynamicBoxContext.getInputChunkEndTime(0, 0);

		for(uint32_t i=1; i<l_rStaticBoxContext.getInputCount(); i++)
		{
			if(l_rDynamicBoxContext.getInputChunkCount(i)==0)
			{
				return true;
			}
			else
			{
				bool l_bValidDates=true;
				if(l_ui64StartTime!=l_rDynamicBoxContext.getInputChunkStartTime(i, 0)) { l_bValidDates=false; }
				if(l_ui64EndTime!=l_rDynamicBoxContext.getInputChunkEndTime(i, 0)) { l_bValidDates=false; }
				if(!l_bValidDates)
				{
					OV_WARNING_K("Chunk dates mismatch, check stream structure or parameters");
					return l_bValidDates;
				}
			}
		}

	// ready to process !
	getBoxAlgorithmContext()->markAlgorithmAsReadyToProcess();

	return true;
}
/*******************************************************************************/

bool CBoxAlgorithmConnectivityMeasure::process(void)
{
	
	// the static box context describes the box inputs, outputs, settings structures
//	IBox& l_rStaticBoxContext=this->getStaticBoxContext();
	// the dynamic box context describes the current state of the box inputs and outputs (i.e. the chunks)
	IBoxIO& l_rDynamicBoxContext=this->getDynamicBoxContext();

	bool l_bHeaderReceived = false;
	bool l_bBufferReceived = false;
	bool l_bEndReceived = false;

	// we decode the input signal chunks
	for(uint32_t i=0; i<l_rDynamicBoxContext.getInputChunkCount(0); i++)
	{
		m_oAlgo0_SignalDecoder.decode(i);

		if(m_ui32InputCount==2)
		{
			m_oAlgo2_SignalDecoder.decode(i);

			if(m_oAlgo0_SignalDecoder.isHeaderReceived() && m_oAlgo2_SignalDecoder.isHeaderReceived())
			{
				l_bHeaderReceived = true;
			}

			if(m_oAlgo0_SignalDecoder.isBufferReceived() && m_oAlgo2_SignalDecoder.isBufferReceived())
			{
				l_bBufferReceived = true;
			}

			if(m_oAlgo0_SignalDecoder.isEndReceived() && m_oAlgo2_SignalDecoder.isEndReceived())
			{
				l_bEndReceived = true;
			}

		}
		else
		{
			if(m_oAlgo0_SignalDecoder.isHeaderReceived()){l_bHeaderReceived = true;}
			if(m_oAlgo0_SignalDecoder.isBufferReceived()){l_bBufferReceived = true;}
			if(m_oAlgo0_SignalDecoder.isEndReceived()){l_bEndReceived = true;}
		}

		// If header is received
		if(l_bHeaderReceived)
		{

			//______________________________________________________________________________________________________________________________________
			//
			// Splits the channel list in order to identify channel pairs to process ***** Code adapted from channel selector *****
			//_______________________________________________________________________________________________________________________________________
			//

			// Retrieve string setting giving channel pairs
			CString l_sChannelList = FSettingValueAutoCast(*this->getBoxAlgorithmContext(), 1);
			uint64 l_ui64MatchMethodIdentifier = FSettingValueAutoCast(*this->getBoxAlgorithmContext(), 2);

			m_vChannelTable.clear();

			std::vector < CString > l_sPairs;
			uint32_t l_ui32PairsCount = Tools::String::split(l_sChannelList, Tools::String::TSplitCallback < std::vector < CString > >(l_sPairs), OV_Value_EnumeratedStringSeparator);

			for(uint32_t pair=0; pair<l_ui32PairsCount; pair++)
			{
				m_bRange1 = false;
				m_bRange2 = false;
				std::vector < CString > l_sChannel;
				uint32_t l_ui32ChannelCount = Tools::String::split(l_sPairs[pair], Tools::String::TSplitCallback < std::vector < CString > >(l_sChannel), OVP_Value_CoupledStringSeparator);
				
				for(uint32_t chan=0; chan<l_ui32ChannelCount; chan=chan+2)
				{
					std::vector < CString > l_sSubChannel;
					std::vector < CString > l_sSubChannel2;

					uint32_t range1 = Tools::String::split(l_sChannel[chan], OpenViBEToolkit::Tools::String::TSplitCallback < std::vector < CString > >(l_sSubChannel), OV_Value_RangeStringSeparator);
					uint32_t range2 = Tools::String::split(l_sChannel[chan+1], OpenViBEToolkit::Tools::String::TSplitCallback < std::vector < CString > >(l_sSubChannel2), OV_Value_RangeStringSeparator);
					
					uint32_t l_ui32RangeStartIndex = uint32(-1);
					uint32_t l_ui32RangeEndIndex = uint32(-1);
					uint32_t l_ui32RangeStartIndex2 = uint32(-1);
					uint32_t l_ui32RangeEndIndex2 = uint32(-1);
					
					if (Tools::String::isAlmostEqual(l_sSubChannel[0], "*", false)) // If all channels selected
					{
						// Select all the channels
						l_ui32RangeStartIndex = 0;
						l_ui32RangeEndIndex = ip_pMatrix1->getDimensionSize(0)-1;
					}
					else // Else find channels selected
					{
						l_ui32RangeStartIndex=::_find_channel_(*ip_pMatrix1, l_sSubChannel[0], l_ui64MatchMethodIdentifier);
						l_ui32RangeEndIndex=::_find_channel_(*ip_pMatrix1, l_sSubChannel[range1-1], l_ui64MatchMethodIdentifier);
					}


					if (Tools::String::isAlmostEqual(l_sSubChannel2[0], "*", false))
					{
						//Select all the channels
						l_ui32RangeStartIndex2 = 0;
						l_ui32RangeEndIndex2 = ip_pMatrix1->getDimensionSize(0)-1;
					}
					else
					{
						l_ui32RangeStartIndex2 =::_find_channel_(*ip_pMatrix1, l_sSubChannel2[0], l_ui64MatchMethodIdentifier);
						l_ui32RangeEndIndex2=::_find_channel_(*ip_pMatrix1, l_sSubChannel2[range2-1], l_ui64MatchMethodIdentifier);
					}
					

					// When first or second part is not found but associated token is empty, don't consider this as an error
					if(l_ui32RangeStartIndex == uint32(-1) && l_sSubChannel[0] == CString("")) l_ui32RangeStartIndex = 0;
					if(l_ui32RangeEndIndex == uint32(-1) && l_sSubChannel[range1-1] == CString("")) l_ui32RangeEndIndex = ip_pMatrix1->getDimensionSize(0)-1;

					if(l_ui32RangeStartIndex2 == uint32(-1) && l_sSubChannel2[0] == CString("")) l_ui32RangeStartIndex2 = 0;
					if(l_ui32RangeEndIndex2 == uint32(-1) && l_sSubChannel2[range2-1] == CString("")) l_ui32RangeEndIndex2 = ip_pMatrix1->getDimensionSize(0)-1;

					// After these corrections, if either first or second token were not found, or if start index is greater than start index, consider this an error and invalid range
					if(l_ui32RangeStartIndex == uint32(-1) || l_ui32RangeEndIndex  == uint32(-1) || l_ui32RangeStartIndex > l_ui32RangeEndIndex)
					{
						OV_WARNING_K("Invalid channel or range [" << l_sChannel[chan] << "]");
					}
					else if(l_ui32RangeStartIndex2 == uint32(-1) || l_ui32RangeEndIndex2 == uint32(-1) || l_ui32RangeStartIndex2 > l_ui32RangeEndIndex2)
					{
						OV_WARNING_K("Invalid channel or range [" << l_sChannel[chan+1] << "]");
					}
					else
					{
					// The ranges are valid so selects all the channels in those range
						this->getLogManager() << LogLevel_Trace << "For range [" << l_sChannel[chan] << "] :\n";
						for(uint32_t k=l_ui32RangeStartIndex; k<=l_ui32RangeEndIndex; k++)
						{
							for(uint32_t j=l_ui32RangeStartIndex2; j<=l_ui32RangeEndIndex2; j++)
							{
								m_vChannelTable.push_back(k);
								m_vChannelTable.push_back(j);

								this->getLogManager() << LogLevel_Trace << "  Selected channels [" << k+1 << ","<< j+1 <<"]\n";

							}

						}
					}
				}
			}

			ip_pChannelTable->setDimensionSize(0,m_vChannelTable.size());


			// Copy the look up vector into the parameterHandler in order to pass it to the algorithm
			for(uint32_t cpt=0;cpt<m_vChannelTable.size();cpt++)
			{
				ip_pChannelTable->getBuffer()[cpt] = m_vChannelTable[cpt];
			}

			// Make sure the algo initialization was successful
			if(!m_pConnectivityMethod->process(OVP_Algorithm_Connectivity_InputTriggerId_Initialize))
			{
				OV_WARNING_K("Initialization was unsuccessful");
				return false;
			}

			// Pass the header to the next boxes, by encoding a header on the output 0:
			m_oAlgo1_SignalEncoder.encodeHeader();

			// send the output chunk containing the header. The dates are the same as the input chunk:
			l_rDynamicBoxContext.markOutputAsReadyToSend(0, l_rDynamicBoxContext.getInputChunkStartTime(0, i), l_rDynamicBoxContext.getInputChunkEndTime(0, i));

			if(m_ui32OutputCount>1)
			{
				m_oAlgo3_SpectrumEncoder.encodeHeader();
				l_rDynamicBoxContext.markOutputAsReadyToSend(1, l_rDynamicBoxContext.getInputChunkStartTime(0, i), l_rDynamicBoxContext.getInputChunkEndTime(0, i));
			}

		}
		if(l_bBufferReceived)
		{
			
			m_pConnectivityMethod->process(OVP_Algorithm_Connectivity_InputTriggerId_Process);

			if(m_pConnectivityMethod->isOutputTriggerActive(OVP_Algorithm_Connectivity_OutputTriggerId_ProcessDone))
			{
				// Encode the output buffer :
				m_oAlgo1_SignalEncoder.encodeBuffer();
				// and send it to the next boxes :
				l_rDynamicBoxContext.markOutputAsReadyToSend(0, l_rDynamicBoxContext.getInputChunkStartTime(0, i), l_rDynamicBoxContext.getInputChunkEndTime(0, i));
				if(m_ui32OutputCount>1)
				{
					m_oAlgo3_SpectrumEncoder.encodeBuffer();
					//for (int freq = 0; freq < 32;freq++)
						//std::cout << m_oAlgo3_SpectrumEncoder.getInputFrequencyAbscissa()->getBuffer()[freq] << std::endl;
					l_rDynamicBoxContext.markOutputAsReadyToSend(1, l_rDynamicBoxContext.getInputChunkStartTime(0, i), l_rDynamicBoxContext.getInputChunkEndTime(0, i));
				}
			}
		}

		if(l_bEndReceived)
		{
			// End of stream received. This happens only once when pressing "stop". Just pass it to the next boxes so they receive the message :
			m_oAlgo1_SignalEncoder.encodeEnd();

			l_rDynamicBoxContext.markOutputAsReadyToSend(0, l_rDynamicBoxContext.getInputChunkStartTime(0, i), l_rDynamicBoxContext.getInputChunkEndTime(0, i));

			if(m_ui32OutputCount>1)
			{
				m_oAlgo3_SpectrumEncoder.encodeEnd();
				l_rDynamicBoxContext.markOutputAsReadyToSend(1, l_rDynamicBoxContext.getInputChunkStartTime(0, i), l_rDynamicBoxContext.getInputChunkEndTime(0, i));
			}
		}

		// The current input chunk has been processed, and automatically discarded
	}

	return true;
}

#endif //#TARGET_HAS_ThirdPartyEIGEN
