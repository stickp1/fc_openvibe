#if defined(TARGET_HAS_ThirdPartyEIGEN)

#ifndef __OpenViBEPlugins_BoxAlgorithm_ConnectivityMeasure_H__
#define __OpenViBEPlugins_BoxAlgorithm_ConnectivityMeasure_H__

#include "../../ovp_defines.h"

#include <openvibe/ov_all.h>
#include <toolkit/ovtk_all.h>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <vector>
#include <iostream>
#include <Eigen/Dense>

#include "../../algorithms/connectivity/ovpCConnectivityAlgorithm.h"

#include <string>

// The unique identifiers for the box and its descriptor.
// Identifier are randomly chosen by the skeleton-generator.
//#define OVP_ClassId_BoxAlgorithm_ConnectivityMeasure OpenViBE::CIdentifier(0x8E3A1AEF, 0x7CACD368)
//#define OVP_ClassId_BoxAlgorithm_ConnectivityMeasureDesc OpenViBE::CIdentifier(0xA20B0A40, 0x1A92D645)

#define OVP_BoxAlgorithm_ConnectivityMeasure_CommonSettingsCount 3

namespace OpenViBEPlugins
{
	namespace SignalProcessing
	{
		/**
		 * \class CBoxAlgorithmConnectivityMeasure
		 * \author Alison Cellard (Inria)
		 * \date Fri Apr 19 11:21:04 2013
		 * \brief The class CBoxAlgorithmConnectivityMeasure describes the box Connectivity Measure.
		 *
		 */
		class CBoxAlgorithmConnectivityMeasure : virtual public OpenViBEToolkit::TBoxAlgorithm < OpenViBE::Plugins::IBoxAlgorithm >
		{
		public:
			virtual void release(void) { delete this; }

			virtual bool initialize(void);
			virtual bool uninitialize(void);
				
			virtual bool processInput(uint32_t ui32InputIndex);
			virtual bool process(void);


			_IsDerivedFromClass_Final_(OpenViBEToolkit::TBoxAlgorithm < OpenViBE::Plugins::IBoxAlgorithm >, OVP_ClassId_BoxAlgorithm_ConnectivityMeasure);

		protected:

			// Signal stream decoder and encoder
			OpenViBEToolkit::TSignalDecoder < CBoxAlgorithmConnectivityMeasure > m_oAlgo0_SignalDecoder; // Decoder for input 1
			OpenViBEToolkit::TSignalDecoder < CBoxAlgorithmConnectivityMeasure > m_oAlgo2_SignalDecoder; // Decoder for input 2 if needed
			OpenViBEToolkit::TSignalEncoder < CBoxAlgorithmConnectivityMeasure > m_oAlgo1_SignalEncoder;
			OpenViBEToolkit::TSpectrumEncoder < CBoxAlgorithmConnectivityMeasure > m_oAlgo3_SpectrumEncoder; // Encoder for output 2 if needed

			OpenViBE::Kernel::IAlgorithmProxy* m_pConnectivityMethod;
			OpenViBE::Kernel::TParameterHandler <OpenViBE::IMatrix*> ip_pMatrix1;
			OpenViBE::Kernel::TParameterHandler <OpenViBE::IMatrix*> ip_pMatrix2;
			OpenViBE::Kernel::TParameterHandler <uint64_t> ip_ui64SamplingRate1;
			OpenViBE::Kernel::TParameterHandler <uint64_t> ip_ui64SamplingRate2;
			OpenViBE::Kernel::TParameterHandler <OpenViBE::IMatrix*> op_pMatrix;// Output matrix, will store the connectivity measure
			OpenViBE::Kernel::TParameterHandler <OpenViBE::IMatrix*> op_pMatrix2; // In case of second input

			OpenViBE::Kernel::TParameterHandler <OpenViBE::IMatrix*> ip_pChannelTable;
			std::vector < uint32_t > m_vChannelTable; // Matrix storing the index of the channels required

			OpenViBE::Kernel::TParameterHandler <uint64_t> ip_ui64WindowMethod;
			OpenViBE::Kernel::TParameterHandler <uint64_t> ip_ui64SegmentsLength;
			OpenViBE::Kernel::TParameterHandler <uint64_t> ip_ui64Overlap;
			OpenViBE::Kernel::TParameterHandler <uint64_t> ip_ui64FreqWindow;
//			OpenViBE::Kernel::TParameterHandler <uint64_t> ip_ui64FFTSize;

			OpenViBE::Kernel::TParameterHandler <OpenViBE::IMatrix*> op_pFrequencyVector;

		private:

			uint32_t m_ui32PairsCount; // Number of pairs of channel to measure connectivity between
			uint32_t m_ui32InputCount; // Number of inputs (1 or 2)
			uint32_t m_ui32OutputCount;
			bool m_bRange1;
			bool m_bRange2;

		};

		class CBoxAlgorithmConnectivityMeasureListener : public OpenViBEToolkit::TBoxListener < OpenViBE::Plugins::IBoxListener >
		{
			public:

			CBoxAlgorithmConnectivityMeasureListener(const uint32_t ui32CustomSettingBase)
                :m_ui32CustomSettingBase(ui32CustomSettingBase)
            {
            }

			virtual bool initialize(void)
			{
				m_oConnectivityAlgorithmClassIdentifier=OV_UndefinedIdentifier;
				m_pConnectivityMethod= nullptr;
				return true;
			}

			virtual bool uninitialize(void)
			{
				if(m_pConnectivityMethod)
				{
					m_pConnectivityMethod->uninitialize();
					this->getAlgorithmManager().releaseAlgorithm(*m_pConnectivityMethod);
					m_pConnectivityMethod= nullptr;
				}
				return true;
			}


			bool onInputAdded(OpenViBE::Kernel::IBox& rBox, const uint32_t ui32Index)
			{
				rBox.setInputType(ui32Index, OV_TypeId_Signal);
				return true;
            }

			virtual bool onInitialized(OpenViBE::Kernel::IBox& rBox)
            {
                return this->onAlgorithmClassIdentifierChanged(rBox);
            }

			virtual bool onSettingValueChanged(OpenViBE::Kernel::IBox& rBox, const uint32_t ui32Index)
            {
            	return ui32Index==0?this->onAlgorithmClassIdentifierChanged(rBox):true;
            }

			virtual bool onAlgorithmClassIdentifierChanged(OpenViBE::Kernel::IBox& rBox)
            {
				OpenViBE::CString l_sConnectivityMethodName;
				OpenViBE::CIdentifier l_oConnectivityMethodIdentifier;
				OpenViBE::CIdentifier l_oOldConnectivityMethodIdentifier=m_oConnectivityAlgorithmClassIdentifier;
				OpenViBE::CIdentifier l_oIdentifier = OV_UndefinedIdentifier;


				rBox.getSettingValue(0, l_sConnectivityMethodName);

				l_oConnectivityMethodIdentifier=this->getTypeManager().getEnumerationEntryValueFromName(OVP_ClassId_ConnectivityAlgorithm, l_sConnectivityMethodName);
				if(l_oConnectivityMethodIdentifier != m_oConnectivityAlgorithmClassIdentifier)
				{
					if(m_pConnectivityMethod)
                	{
						m_pConnectivityMethod->uninitialize();
						this->getAlgorithmManager().releaseAlgorithm(*m_pConnectivityMethod);
						m_pConnectivityMethod= nullptr;
						m_oConnectivityAlgorithmClassIdentifier=OV_UndefinedIdentifier;
                	}
                	if(l_oConnectivityMethodIdentifier != OV_UndefinedIdentifier)
                	{
                		m_pConnectivityMethod=&this->getAlgorithmManager().getAlgorithm(this->getAlgorithmManager().createAlgorithm(l_oConnectivityMethodIdentifier));
                		m_pConnectivityMethod->initialize();
                		m_oConnectivityAlgorithmClassIdentifier=l_oConnectivityMethodIdentifier;
                	}

                	if(l_oOldConnectivityMethodIdentifier != OV_UndefinedIdentifier)
                	{
                		while(rBox.getSettingCount()>m_ui32CustomSettingBase)
                		{
                			rBox.removeSetting(m_ui32CustomSettingBase);
                		}
                		while(rBox.getOutputCount()>1)
                		{
                			rBox.removeOutput(1);
                		}
                	}
				}

				if(m_pConnectivityMethod)
				{
					uint32_t j=1;
					rBox.setOutputName(0, m_pConnectivityMethod->getOutputParameterName(OVP_Algorithm_Connectivity_OutputParameterId_OutputMatrix));
					while((l_oIdentifier=m_pConnectivityMethod->getNextOutputParameterIdentifier(l_oIdentifier))!=OV_UndefinedIdentifier)
					{
						if(l_oIdentifier!=OVP_Algorithm_Connectivity_OutputParameterId_OutputMatrix
								&& l_oIdentifier!=OVP_Algorithm_MagnitudeSquaredCoherence_OutputParameterId_FreqVector
								&& l_oIdentifier!=OVP_Algorithm_ImCoherence_OutputParameterId_FreqVector
								&& l_oIdentifier!=OVP_Algorithm_PhaseSlopeIndex_OutputParameterId_FreqVector)
						{
							OpenViBE::CString l_sOutputName=m_pConnectivityMethod->getOutputParameterName(l_oIdentifier);
							rBox.addOutput(l_sOutputName, OVTK_TypeId_Spectrum);
							j++;
						}
						
					}
					while(j<rBox.getOutputCount())
					{
						rBox.removeOutput(j);
					}


					uint32_t i=m_ui32CustomSettingBase;
					while((l_oIdentifier=m_pConnectivityMethod->getNextInputParameterIdentifier(l_oIdentifier))!=OV_UndefinedIdentifier)
					{
						if((l_oIdentifier!=OVP_Algorithm_Connectivity_InputParameterId_InputMatrix1)
							&& (l_oIdentifier!=OVP_Algorithm_Connectivity_InputParameterId_InputMatrix2)
							&& (l_oIdentifier!=OVP_Algorithm_Connectivity_InputParameterId_LookupMatrix)
							&& (l_oIdentifier!=OVP_Algorithm_Connectivity_InputParameterId_ui64SamplingRate1)
							&& (l_oIdentifier!=OVP_Algorithm_Connectivity_InputParameterId_ui64SamplingRate2))
						{
							OpenViBE::CIdentifier l_oTypeIdentifier;
                			OpenViBE::CString l_sParameterName=m_pConnectivityMethod->getInputParameterName(l_oIdentifier);
                			OpenViBE::Kernel::IParameter* l_pParameter=m_pConnectivityMethod->getInputParameter(l_oIdentifier);
                			OpenViBE::Kernel::TParameterHandler < OpenViBE::int64 > ip_i64Parameter(l_pParameter);
							OpenViBE::Kernel::TParameterHandler < uint64_t > ip_ui64Parameter(l_pParameter);
                			OpenViBE::Kernel::TParameterHandler < OpenViBE::float64 > ip_f64Parameter(l_pParameter);
							OpenViBE::Kernel::TParameterHandler < bool > ip_bParameter(l_pParameter);
                			OpenViBE::Kernel::TParameterHandler < OpenViBE::CString* > ip_sParameter(l_pParameter);
                			char l_sBuffer[1024];
                			bool l_bValid=true;
                			switch(l_pParameter->getType())
                			{
                				case OpenViBE::Kernel::ParameterType_Enumeration:
                					::strcpy(l_sBuffer, this->getTypeManager().getEnumerationEntryNameFromValue(l_pParameter->getSubTypeIdentifier(), ip_ui64Parameter).toASCIIString());
                					l_oTypeIdentifier=l_pParameter->getSubTypeIdentifier();
                					break;

                				case OpenViBE::Kernel::ParameterType_Integer:
                				case OpenViBE::Kernel::ParameterType_UInteger:
									::sprintf(l_sBuffer, "%lli", (uint64_t)ip_ui64Parameter);
                					l_oTypeIdentifier=OV_TypeId_Integer;
                					break;

                				case OpenViBE::Kernel::ParameterType_Boolean:
									::sprintf(l_sBuffer, "%s", ((bool)ip_bParameter)?"true":"false");
                					l_oTypeIdentifier=OV_TypeId_Boolean;
                					break;

                				case OpenViBE::Kernel::ParameterType_Float:
                					::sprintf(l_sBuffer, "%lf", (OpenViBE::float64)ip_f64Parameter);
                					l_oTypeIdentifier=OV_TypeId_Float;
                					break;
                				case OpenViBE::Kernel::ParameterType_String:
                					::sprintf(l_sBuffer, "%s", ((OpenViBE::CString*)ip_sParameter)->toASCIIString());
                					l_oTypeIdentifier=OV_TypeId_String;
                					break;

                				default:
                					l_bValid=false;
                					break;
                			}

                			if(l_bValid)
                			{
                				if(i>=rBox.getSettingCount())
                				{
                					rBox.addSetting(l_sParameterName, l_oTypeIdentifier, l_sBuffer);
                				}
                				else
                				{
                					OpenViBE::CIdentifier l_oOldTypeIdentifier;
                					rBox.getSettingType(i, l_oOldTypeIdentifier);
                					if(l_oOldTypeIdentifier != l_oTypeIdentifier)
                					{
                						rBox.setSettingType(i, l_oTypeIdentifier);
                						rBox.setSettingValue(i, l_sBuffer);
                					}
                						rBox.setSettingName(i, l_sParameterName);
                				}

                				i++;
                			}
						}
					}

					while(i<rBox.getSettingCount())
					{
						rBox.removeSetting(i);
					}
				}
				return true;
            }

            _IsDerivedFromClass_Final_(OpenViBEToolkit::TBoxListener < OpenViBE::Plugins::IBoxListener >, OV_UndefinedIdentifier);

        protected:

			OpenViBE::CIdentifier m_oConnectivityAlgorithmClassIdentifier;
			OpenViBE::Kernel::IAlgorithmProxy* m_pConnectivityMethod;
			const uint32_t m_ui32CustomSettingBase;
		};

		
		/**
		 * \class CBoxAlgorithmConnectivityMeasureDesc
		 * \author Alison Cellard (Inria)
		 * \date Fri Apr 19 11:21:04 2013
		 * \brief Descriptor of the box Connectivity Measure.
		 *
		 */
		class CBoxAlgorithmConnectivityMeasureDesc : virtual public OpenViBE::Plugins::IBoxAlgorithmDesc
		{
		public:

			virtual void release(void) { }

			virtual OpenViBE::CString getName(void) const                { return OpenViBE::CString("Connectivity Measure"); }
			virtual OpenViBE::CString getAuthorName(void) const          { return OpenViBE::CString("Alison Cellard"); }
			virtual OpenViBE::CString getAuthorCompanyName(void) const   { return OpenViBE::CString("Inria"); }
			virtual OpenViBE::CString getShortDescription(void) const    { return OpenViBE::CString("Measure connectivity between pairs of channel"); }
			virtual OpenViBE::CString getDetailedDescription(void) const { return OpenViBE::CString("Measure connectivity between pairs of channel using the kind of measure chosen (PLV, MSC, etc.)"); }
			virtual OpenViBE::CString getCategory(void) const            { return OpenViBE::CString("Signal processing/Connectivity"); }
			virtual OpenViBE::CString getVersion(void) const             { return OpenViBE::CString("1.0"); }
			virtual OpenViBE::CString getStockItemName(void) const       { return OpenViBE::CString("gtk-new"); }

			virtual OpenViBE::CIdentifier getCreatedClass(void) const    { return OVP_ClassId_BoxAlgorithm_ConnectivityMeasure; }
			virtual OpenViBE::Plugins::IPluginObject* create(void)       { return new OpenViBEPlugins::SignalProcessing::CBoxAlgorithmConnectivityMeasure; }
			
            virtual OpenViBE::Plugins::IBoxListener* createBoxListener(void) const               { return new CBoxAlgorithmConnectivityMeasureListener(OVP_BoxAlgorithm_ConnectivityMeasure_CommonSettingsCount); }
			virtual void releaseBoxListener(OpenViBE::Plugins::IBoxListener* pBoxListener) const { delete pBoxListener; }

			
			virtual bool getBoxPrototype(
				OpenViBE::Kernel::IBoxProto& rBoxAlgorithmPrototype) const
			{
				rBoxAlgorithmPrototype.addInput("Input Signal", OV_TypeId_Signal);
				rBoxAlgorithmPrototype.addFlag(OpenViBE::Kernel::BoxFlag_CanAddInput);
//				rBoxAlgorithmPrototype.addFlag(OpenViBE::Kernel::BoxFlag_CanModifyInput);
				
				rBoxAlgorithmPrototype.addOutput("Connectivity measure", OV_TypeId_Signal);
				
                		rBoxAlgorithmPrototype.addSetting("Method",OVP_ClassId_ConnectivityAlgorithm, OVP_TypeId_Algorithm_SingleTrialPhaseLockingValue.toString());
				rBoxAlgorithmPrototype.addSetting("Pairs of channels",OV_TypeId_String,"1-2");
				rBoxAlgorithmPrototype.addSetting("Channel Matching Method",  OVP_TypeId_MatchMethod, OVP_TypeId_MatchMethod_Smart.toString());
//               		rBoxAlgorithmPrototype.addFlag(OpenViBE::Kernel::BoxFlag_CanAddSetting);
				
				rBoxAlgorithmPrototype.addFlag(OV_AttributeId_Box_FlagIsUnstable);
				
				return true;
			}



			_IsDerivedFromClass_Final_(OpenViBE::Plugins::IBoxAlgorithmDesc, OVP_ClassId_BoxAlgorithm_ConnectivityMeasureDesc);
		};
	};
};

#endif // __OpenViBEPlugins_BoxAlgorithm_ConnectivityMeasure_H__
#endif //TARGET_HAS_ThirdPartyEIGEN
