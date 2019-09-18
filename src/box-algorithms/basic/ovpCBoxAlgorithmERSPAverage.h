#ifndef __OpenViBEPlugins_BoxAlgorithm_ERSPAverage_H__
#define __OpenViBEPlugins_BoxAlgorithm_ERSPAverage_H__

#include "../../ovp_defines.h"
#include <openvibe/ov_all.h>
#include <toolkit/ovtk_all.h>


namespace OpenViBEPlugins
{
	namespace SignalProcessing
	{
		class CBoxAlgorithmERSPAverage : virtual public OpenViBEToolkit::TBoxAlgorithm < OpenViBE::Plugins::IBoxAlgorithm >
		{
		public:

			virtual void release(void) { delete this; }

			virtual OpenViBE::boolean initialize(void);
			virtual OpenViBE::boolean uninitialize(void);
			virtual OpenViBE::boolean processInput(OpenViBE::uint32 ui32InputIndex);
			virtual OpenViBE::boolean process(void);

			_IsDerivedFromClass_Final_(OpenViBEToolkit::TBoxAlgorithm < OpenViBE::Plugins::IBoxAlgorithm >, OVP_ClassId_BoxAlgorithm_ERSPAverage);

		protected:

			bool appendChunk(const OpenViBE::IMatrix& chunk, uint64_t startTime, uint64_t endTime);
			bool computeAndSend(void);

			OpenViBEToolkit::TSpectrumDecoder < CBoxAlgorithmERSPAverage > m_oDecoderSpectrum;
			OpenViBEToolkit::TStimulationDecoder < CBoxAlgorithmERSPAverage > m_oDecoderStimulations;
			OpenViBEToolkit::TSpectrumEncoder < CBoxAlgorithmERSPAverage > m_oEncoder;

			struct Timestamp
			{
				uint64_t m_startTime;
				uint64_t m_endTime;			
			};

			std::vector< std::vector< OpenViBE::CMatrix* > > m_cachedSpectra;
			std::vector< Timestamp > m_timestamps;

			uint32_t m_currentChunk = 0;
			uint32_t m_numTrials = 0;

			uint64_t m_epochingStim;
			uint64_t m_computeStim;
			

		};


		class CBoxAlgorithmERSPAverageDesc : virtual public OpenViBE::Plugins::IBoxAlgorithmDesc
		{
		public:

			virtual void release(void) { }

			virtual OpenViBE::CString getName(void) const                { return OpenViBE::CString("ERSP Average"); }
			virtual OpenViBE::CString getAuthorName(void) const          { return OpenViBE::CString("Jussi T. Lindgren"); }
			virtual OpenViBE::CString getAuthorCompanyName(void) const   { return OpenViBE::CString("Inria"); }
			virtual OpenViBE::CString getShortDescription(void) const    { return OpenViBE::CString("Averages a sequence of spectra per trial across multiple trials. The result is a sequence starting from t=0."); }
			virtual OpenViBE::CString getDetailedDescription(void) const { return OpenViBE::CString("Example: Given an input sequence [t,s1,s2,t,s3,s4] for two trials with s* the spectra and t a stimulation denoting the trial start, the box returns 1/2*(s1+s3), 1/2*(s2+s4)."); }
			virtual OpenViBE::CString getCategory(void) const            { return OpenViBE::CString("Signal processing/Basic"); }
			virtual OpenViBE::CString getVersion(void) const             { return OpenViBE::CString("0.1"); }
			virtual OpenViBE::CString getStockItemName(void) const       { return OpenViBE::CString("gtk-sort-ascending"); }

			virtual OpenViBE::CIdentifier getCreatedClass(void) const    { return OVP_ClassId_BoxAlgorithm_ERSPAverage; }
			virtual OpenViBE::Plugins::IPluginObject* create(void)       { return new OpenViBEPlugins::SignalProcessing::CBoxAlgorithmERSPAverage; }

			virtual OpenViBE::boolean getBoxPrototype(
				OpenViBE::Kernel::IBoxProto& rBoxAlgorithmPrototype) const
			{
				rBoxAlgorithmPrototype.addInput  ("Input spectra",   OV_TypeId_Spectrum);
				rBoxAlgorithmPrototype.addInput  ("Control stream",  OV_TypeId_Stimulations);
				rBoxAlgorithmPrototype.addOutput ("Output spectra",  OV_TypeId_Spectrum);

				rBoxAlgorithmPrototype.addSetting("Trial start marker",   OV_TypeId_Stimulation, "OVTK_GDF_Start_Of_Trial",false);
				rBoxAlgorithmPrototype.addSetting("Computation trigger",  OV_TypeId_Stimulation, "OVTK_StimulationId_ExperimentStop",false);

				return true;
			}

			_IsDerivedFromClass_Final_(OpenViBE::Plugins::IBoxAlgorithmDesc, OVP_ClassId_BoxAlgorithm_ERSPAverageDesc);
		};
	};
};

#endif // __OpenViBEPlugins_BoxAlgorithm_ERSPAverage_H__
