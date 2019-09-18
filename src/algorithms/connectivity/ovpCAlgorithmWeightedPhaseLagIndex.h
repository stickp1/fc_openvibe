#if defined(TARGET_HAS_ThirdPartyEIGEN)

#ifndef __OpenViBEPlugins_Algorithm_WeightedPhaseLagIndex_H__
#define __OpenViBEPlugins_Algorithm_WeightedPhaseLagIndex_H__

#include "../../ovp_defines.h"
#include <openvibe/ov_all.h>
#include <toolkit/ovtk_all.h>
#include "ovpCConnectivityAlgorithm.h"
#include <Eigen/Dense>
#include <unsupported/Eigen/FFT>

#include "../basic/ovpCHilbertTransform.h"


namespace OpenViBEPlugins
{
	namespace SignalProcessing
	{
		class CAlgorithmWeightedPhaseLagIndex : public CConnectivityAlgorithm
		{
		public:

			virtual void release(void) { delete this; }

			virtual OpenViBE::boolean initialize(void);
			virtual OpenViBE::boolean uninitialize(void);
			virtual OpenViBE::boolean process(void);


			_IsDerivedFromClass_Final_(OpenViBEPlugins::CConnectivityAlgorithm, OVP_TypeId_Algorithm_WeightedPhaseLagIndex);

		protected:

			OpenViBE::Kernel::IAlgorithmProxy* m_pHilbertTransform;

			OpenViBE::Kernel::TParameterHandler <OpenViBE::IMatrix*> ip_pHilbertInput;
			OpenViBE::Kernel::TParameterHandler <OpenViBE::IMatrix*> op_pHilbertMatrix;
		};

		class CAlgorithmWeightedPhaseLagIndexDesc : public OpenViBEPlugins::CConnectivityAlgorithmDesc
		{
		public:
			virtual void release(void) { }

			virtual OpenViBE::CString getName(void) const                { return OpenViBE::CString("Weighted Phase Lag Index algorithm"); }
			virtual OpenViBE::CString getAuthorName(void) const          { return OpenViBE::CString("Cristiano Berhanu"); }
			virtual OpenViBE::CString getAuthorCompanyName(void) const   { return OpenViBE::CString("LaSEEB, Instituto Superior Técnico, University of Lisbon"); }
			virtual OpenViBE::CString getShortDescription(void) const    { return OpenViBE::CString("Computes the weighted Phase Lag Index"); }
			virtual OpenViBE::CString getDetailedDescription(void) const { return OpenViBE::CString("Implementation of algorithm described in Vinck et al (2011)"); }
			virtual OpenViBE::CString getCategory(void) const            { return OpenViBE::CString("Signal processing/Connectivity"); }
			virtual OpenViBE::CString getVersion(void) const             { return OpenViBE::CString("0.1"); }
			virtual OpenViBE::CString getStockItemName(void) const       { return OpenViBE::CString("gtk-execute"); }

			virtual OpenViBE::CIdentifier getCreatedClass(void) const    { return OVP_TypeId_Algorithm_WeightedPhaseLagIndex; }
			virtual OpenViBE::Plugins::IPluginObject* create(void)       { return new OpenViBEPlugins::SignalProcessing::CAlgorithmWeightedPhaseLagIndex; }

			virtual OpenViBE::boolean getAlgorithmPrototype(
					OpenViBE::Kernel::IAlgorithmProto& rAlgorithmPrototype) const
			{

				rAlgorithmPrototype.addInputParameter(OVP_Algorithm_Connectivity_InputParameterId_InputMatrix1,     "Signal 1", OpenViBE::Kernel::ParameterType_Matrix);
				rAlgorithmPrototype.addInputParameter(OVP_Algorithm_Connectivity_InputParameterId_InputMatrix2,     "Signal 2", OpenViBE::Kernel::ParameterType_Matrix);
				rAlgorithmPrototype.addInputParameter(OVP_Algorithm_Connectivity_InputParameterId_LookupMatrix, "Pairs of channel", OpenViBE::Kernel::ParameterType_Matrix);

				rAlgorithmPrototype.addOutputParameter(OVP_Algorithm_Connectivity_OutputParameterId_OutputMatrix,    "wPLI signal", OpenViBE::Kernel::ParameterType_Matrix);

				rAlgorithmPrototype.addInputTrigger   (OVP_Algorithm_Connectivity_InputTriggerId_Initialize,   "Initialize");
				rAlgorithmPrototype.addInputTrigger   (OVP_Algorithm_Connectivity_InputTriggerId_Process,      "Process");
				rAlgorithmPrototype.addOutputTrigger  (OVP_Algorithm_Connectivity_OutputTriggerId_ProcessDone, "Process done");

				return true;
			}

			_IsDerivedFromClass_Final_(OpenViBEPlugins::CConnectivityAlgorithmDesc, OVP_TypeId_Algorithm_WeightedPhaseLagIndexDesc);
		};
	};  // namespace SignalProcessing
}; // namespace OpenViBEPlugins

#endif //__OpenViBEPlugins_Algorithm_WeightedPhaseLagIndex_H__
#endif //TARGET_HAS_ThirdPartyEIGEN
