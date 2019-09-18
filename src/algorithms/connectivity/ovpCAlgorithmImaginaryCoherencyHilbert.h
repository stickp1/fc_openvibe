#if defined(TARGET_HAS_ThirdPartyEIGEN)

#ifndef __OpenViBEPlugins_Algorithm_ImaginaryCoherencyHilbert_H__
#define __OpenViBEPlugins_Algorithm_ImaginaryCoherencyHilbert_H__

#include "../../ovp_defines.h"
#include <openvibe/ov_all.h>
#include <toolkit/ovtk_all.h>
#include "ovpCConnectivityAlgorithm.h"
#include <Eigen/Dense>
#include <unsupported/Eigen/FFT>

#include "../basic/ovpCHilbertTransform.h"

#define OVP_TypeId_Algorithm_ImaginaryCoherencyHilbert        OpenViBE::CIdentifier(0x09C5244D, 0x109438F3)
#define OVP_TypeId_Algorithm_ImaginaryCoherencyHilbertDesc    OpenViBE::CIdentifier(0xE7FBB4C1, 0x5377381D)

namespace OpenViBEPlugins
{
	namespace SignalProcessing
	{
		class CAlgorithmImaginaryCoherencyHilbert : public CConnectivityAlgorithm
		{
		public:

			virtual void release(void) { delete this; }

			virtual OpenViBE::boolean initialize(void);
			virtual OpenViBE::boolean uninitialize(void);
			virtual OpenViBE::boolean process(void);


			_IsDerivedFromClass_Final_(OpenViBEPlugins::CConnectivityAlgorithm, OVP_TypeId_Algorithm_ImaginaryCoherencyHilbert);

		protected:

			OpenViBE::Kernel::IAlgorithmProxy* m_pHilbertTransform;

			OpenViBE::Kernel::TParameterHandler <OpenViBE::IMatrix*> ip_pHilbertInput;
			OpenViBE::Kernel::TParameterHandler <OpenViBE::IMatrix*> op_pHilbertMatrix;
			
		};

		class CAlgorithmImaginaryCoherencyHilbertDesc : public OpenViBEPlugins::CConnectivityAlgorithmDesc
		{
		public:
			virtual void release(void) { }

			virtual OpenViBE::CString getName(void) const                { return OpenViBE::CString("Imaginary part of Coherency (Hilbert)"); }
			virtual OpenViBE::CString getAuthorName(void) const          { return OpenViBE::CString("Cristiano Berhanu"); }
			virtual OpenViBE::CString getAuthorCompanyName(void) const   { return OpenViBE::CString("Laseeb"); }
			virtual OpenViBE::CString getShortDescription(void) const    { return OpenViBE::CString("Computes the Imaginary Part of Coherency with a Hilbert Transform approach"); }
			virtual OpenViBE::CString getDetailedDescription(void) const { return OpenViBE::CString("requires previous bandpass filtering"); }
			virtual OpenViBE::CString getCategory(void) const            { return OpenViBE::CString("Signal processing/Connectivity"); }
			virtual OpenViBE::CString getVersion(void) const             { return OpenViBE::CString("0.1"); }
			virtual OpenViBE::CString getStockItemName(void) const       { return OpenViBE::CString("gtk-execute"); }

			virtual OpenViBE::CIdentifier getCreatedClass(void) const    { return OVP_TypeId_Algorithm_ImaginaryCoherencyHilbert; }
			virtual OpenViBE::Plugins::IPluginObject* create(void)       { return new OpenViBEPlugins::SignalProcessing::CAlgorithmImaginaryCoherencyHilbert; }

			virtual OpenViBE::boolean getAlgorithmPrototype(
					OpenViBE::Kernel::IAlgorithmProto& rAlgorithmPrototype) const
			{

				rAlgorithmPrototype.addInputParameter(OVP_Algorithm_Connectivity_InputParameterId_InputMatrix1,     "Signal 1", OpenViBE::Kernel::ParameterType_Matrix);
				rAlgorithmPrototype.addInputParameter(OVP_Algorithm_Connectivity_InputParameterId_InputMatrix2,     "Signal 2", OpenViBE::Kernel::ParameterType_Matrix);
				rAlgorithmPrototype.addInputParameter(OVP_Algorithm_Connectivity_InputParameterId_LookupMatrix, "Pairs of channel", OpenViBE::Kernel::ParameterType_Matrix);

				rAlgorithmPrototype.addOutputParameter(OVP_Algorithm_Connectivity_OutputParameterId_OutputMatrix,    "S-PLV signal", OpenViBE::Kernel::ParameterType_Matrix);

				rAlgorithmPrototype.addInputTrigger   (OVP_Algorithm_Connectivity_InputTriggerId_Initialize,   "Initialize");
				rAlgorithmPrototype.addInputTrigger   (OVP_Algorithm_Connectivity_InputTriggerId_Process,      "Process");
				rAlgorithmPrototype.addOutputTrigger  (OVP_Algorithm_Connectivity_OutputTriggerId_ProcessDone, "Process done");

				return true;
			}

			_IsDerivedFromClass_Final_(OpenViBEPlugins::CConnectivityAlgorithmDesc, OVP_TypeId_Algorithm_ImaginaryCoherencyHilbertDesc);
		};
	};  // namespace SignalProcessing
}; // namespace OpenViBEPlugins

#endif //__OpenViBEPlugins_Algorithm_ImaginaryCoherencyHilbert_H__
#endif //TARGET_HAS_ThirdPartyEIGEN
